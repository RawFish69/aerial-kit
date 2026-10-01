#!/usr/bin/env bash
# Host test for the wing mixer. No board, no PlatformIO, no network.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v g++ >/dev/null 2>&1; then
  echo "g++ not found; this test needs a host C++ compiler" >&2
  exit 1
fi

out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$out/mix_math_test" "$here/mix_math_test.cpp"
"$out/mix_math_test"
