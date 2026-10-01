#!/usr/bin/env python3
"""How much stack this firmware needs, from the compiler's own call graph.

    scripts/check-stack.sh [BOARD ARCH PART PRODUCT] [--limit BYTES]

Why this exists: the host tests run with eight megabytes of stack, and the
target has whatever is left of its 128 KB of SRAM after the blackbox rings and
the parameter table - about forty-nine kilobytes on the F405 image of
2026-09-17. A stack overflow therefore cannot be seen on this machine by
running anything, and it is one of the failures that looks like "the board
stopped and said nothing" - which is exactly what the F405 did on the bench that
day. The compiler knows the frame size of every function and who calls whom, so
the question is answerable without a board: build with `-fstack-usage` and
`-fcallgraph-info=su`, then walk the graph.

What it is *not*: a proof. GCC's callgraph has no edges for calls through
function pointers - and this firmware is full of those, because the board
contract hands drivers around as tables - so an indirect call is counted as
zero here. The number this prints is therefore a *floor*: the real worst case is
larger by whatever the indirect calls add. What it is good for is the question
"is it close?", and the answer for this firmware is no: the deepest chain the
compiler can see is a couple of kilobytes against tens.

Usage: stack_report.py <build-dir> [--top N]

`--from NAME` asks the other question: not "what is the widest chain anywhere"
- which is the preflight, at boot - but "how deep is the one that can happen
with the aircraft in the air", by rooting the walk at the function that runs
the loop. `--from` can be repeated.
"""

import argparse
import collections
import glob
import os
import re
import sys


def read_stack_usage(root):
    """`name -> bytes`, from every <file>.su GCC wrote under `root`."""
    usage = {}
    where = {}
    for path in glob.glob(os.path.join(root, "**", "*.su"), recursive=True):
        for line in open(path, errors="replace"):
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 2 or not parts[0]:
                continue
            key = parts[0]
            try:
                size = int(parts[1])
            except ValueError:
                continue
            # "file.c:line:col:name" -> name; the last colon field is the name
            # for a static function and the plain name for an external one.
            name = key.rsplit(":", 1)[-1]
            if size > usage.get(name, -1):
                usage[name] = size
                where[name] = key
    return usage, where


def read_call_graph(root):
    """`caller -> {callee}`, the set of nodes, the indirect sites, and which
    callers have one, from every <file>.ci."""
    edges = collections.defaultdict(set)
    nodes = set()
    indirect = 0
    has_indirect = set()
    for path in glob.glob(os.path.join(root, "**", "*.ci"), recursive=True):
        for line in open(path, errors="replace"):
            node = re.match(r'node: \{ title: "([^"]+)"', line)
            if node:
                title = node.group(1)
                if ":" in title:
                    nodes.add(title.rsplit(":", 1)[-1])
                continue
            edge = re.match(
                r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', line)
            if edge:
                caller = edge.group(1).rsplit(":", 1)[-1]
                callee = edge.group(2).rsplit(":", 1)[-1]
                if callee == "__indirect_call":
                    indirect += 1
                    has_indirect.add(caller)
                    continue
                edges[caller].add(callee)
                nodes.add(caller)
                nodes.add(callee)
    return edges, nodes, indirect, has_indirect


def deepest(usage, edges, has_indirect, indirect_cost, roots=None):
    """The largest total frame size along any path in the graph.

    A cycle (a driver that calls back into the core) is cut rather than
    followed: it cannot nest for ever, and the alternative is not returning.

    `indirect_cost` is added to any frame that has a call the compiler could not
    resolve. It is the largest frame in the whole image, which is what a
    *pessimistic* reading of those 500-odd sites costs - so the two numbers this
    prints bracket the truth.

    `roots` restricts the walk to the chains that start at those functions -
    the report's whole-image answer is the preflight's, and this is how the
    flight loop gets its own.
    """
    memo = {}

    def walk(name, seen):
        if name in memo:
            return memo[name]
        if name in seen:
            return 0, []
        here = seen | {name}
        best_total, best_path = 0, []
        own = usage.get(name, 0)
        if name in has_indirect:
            own += indirect_cost
        for child in edges.get(name, ()):
            total, path = walk(child, here)
            if total > best_total:
                best_total, best_path = total, path
        result = (own + best_total, [name] + best_path)
        memo[name] = result
        return result

    best = (0, [])
    starts = sorted(set(edges) | set(usage)) if roots is None else sorted(roots)
    for node in starts:
        total, path = walk(node, frozenset())
        if total > best[0]:
            best = (total, path)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("build_dir")
    parser.add_argument("--top", type=int, default=8)
    parser.add_argument("--limit", type=int, default=0,
                        help="fail (exit 1) when the deepest path exceeds this")
    parser.add_argument("--from", dest="roots", action="append", default=None,
                        metavar="NAME",
                        help="only walk chains that start at this function "
                             "(repeatable)")
    args = parser.parse_args()

    usage, _where = read_stack_usage(args.build_dir)
    edges, nodes, indirect, has_indirect = read_call_graph(args.build_dir)
    if not usage:
        print("no .su files under %s - build with -fstack-usage" % args.build_dir,
              file=sys.stderr)
        return 1

    roots = None
    if args.roots:
        known = set(edges) | set(usage)
        for name in args.roots:
            if name not in known:
                print("no function named %s in this build - names are the "
                      "compiler's, without arguments" % name, file=sys.stderr)
                return 2
        roots = set(args.roots)

    floor, path = deepest(usage, edges, set(), 0)
    ceiling, _cpath = deepest(usage, edges, has_indirect, max(usage.values()))
    if roots is not None:
        rfloor, rpath = deepest(usage, edges, set(), 0, roots)
        rceiling, _ = deepest(usage, edges, has_indirect, max(usage.values()),
                              roots)
    covered = len(nodes & set(usage))

    print("stack usage, from the compiler's own records")
    print("  functions with a frame size: %d" % len(usage))
    print("  functions in the call graph: %d (%d of them also have a frame size)"
          % (len(nodes), covered))
    print("  indirect calls (edges this analysis cannot follow): %d" % indirect)
    print()
    print("  largest single frames:")
    for name, size in sorted(usage.items(), key=lambda item: -item[1])[:args.top]:
        print("    %6d  %s" % (size, name))
    print()
    print("  deepest chain of resolved calls: %d bytes over %d frames"
          % (floor, len(path)))
    print("  and with one largest-frame call at every unresolved site: %d bytes"
          % ceiling)
    for index, name in enumerate(path):
        if index < 8 or index >= len(path) - 3:
            print("    %6d  %s" % (usage.get(name, 0), name))
        elif index == 8:
            print("           ...")

    if roots is not None:
        print()
        print("  from %s: %d bytes over %d frames (%d pessimistically)"
              % (", ".join(sorted(roots)), rfloor, len(rpath), rceiling))
        for index, name in enumerate(rpath):
            print("    %6d  %s" % (usage.get(name, 0), name))

    if args.limit and ceiling > args.limit:
        print("\nOVER LIMIT: %d > %d (the pessimistic reading)" % (ceiling, args.limit))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
