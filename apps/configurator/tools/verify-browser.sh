#!/usr/bin/env bash
#
# Run the built configurator in real browsers.
#
# W4 asks for the site to be verified "on ... supported browsers" and for
# "connect/disconnect/reconnect and permission workflows" to be exercised.
# jsdom cannot answer either question, so this serves the real subpath build
# over HTTPS and drives it with Chrome.
#
#   ./tools/verify-browser.sh [base]
#
# Modes run:
#   chrome           the full workflow: connect, disconnect, reconnect, and no
#                    console errors anywhere in it
#   chrome-noserial  Chrome with Navigator.prototype.serial deleted before the
#                    app loads — the branch a Firefox or Safari user lands in
#   firefox          attempted, and reported as NOT RUN if it will not launch
#   mavlink          Chrome against the local bridge with an ArduPilot stand-in
#                    behind it. The stand-in is
#                    aerialkit/tools/mavlink_fake_vehicle.py, which encodes and
#                    decodes every byte with pymavlink — the reference
#                    implementation — so the page under test is reading frames
#                    this repository did not produce. **It is not ArduPilot
#                    SITL**, which is not installed here; that substitution is
#                    stated wherever this run is cited.
#
# Needs `puppeteer-core`. It is deliberately NOT a dependency in package.json:
# version 25 requires Node >= 20 and this package targets Node 18, so adding it
# would make `npm install` warn or fail for everyone. Install it unsaved:
#
#   npm install --no-save puppeteer-core
#
# Exits 0 if every browser that ran passed, 1 if a check failed, 3 if nothing
# could be run. **A mode that could not run does not change the exit code** —
# Firefox does not launch on this machine and that is not the app's fault — so
# a 0 does not mean every mode ran. The last line names the ones that did not.

set -uo pipefail

BASE="${1:-/aerialkit/configurator}"
BASE="/${BASE#/}"; BASE="${BASE%/}/"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8445}"
WORK="$(mktemp -d)"
SRV=""
cleanup() { [ -n "$SRV" ] && kill "$SRV" 2>/dev/null; rm -rf "$WORK"; }
trap cleanup EXIT

# Resolved against the package root, not the caller's cwd: running this from
# anywhere else would otherwise report a missing module that is right there.
if ! node -e "require.resolve('puppeteer-core', { paths: ['$HERE'] })" 2>/dev/null; then
  echo "puppeteer-core is not installed. Run this first:"
  echo
  echo "    cd $HERE && npm install --no-save puppeteer-core"
  echo
  exit 3
fi

echo "== building for the subpath: AK_BASE=$BASE"
( cd "$HERE" && AK_BASE="$BASE" npm run build >/dev/null ) || exit 1
mkdir -p "$WORK/site$BASE"
cp -r "$HERE/dist/." "$WORK/site$BASE/"

openssl req -x509 -newkey rsa:2048 -keyout "$WORK/key.pem" -out "$WORK/cert.pem" \
  -days 1 -nodes -subj "/CN=127.0.0.1" >/dev/null 2>&1

cat > "$WORK/serve.py" <<'PY'
import http.server, ssl, sys
root, port, cert, key = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER); ctx.load_cert_chain(cert, key)
srv = http.server.HTTPServer(("127.0.0.1", port),
      lambda *a, **k: http.server.SimpleHTTPRequestHandler(*a, directory=root, **k))
srv.socket = ctx.wrap_socket(srv.socket, server_side=True); srv.serve_forever()
PY
python3 "$WORK/serve.py" "$WORK/site" "$PORT" "$WORK/cert.pem" "$WORK/key.pem" >"$WORK/serve.log" 2>&1 &
SRV=$!
sleep 2
kill -0 "$SRV" 2>/dev/null || { echo "the local HTTPS server did not start"; cat "$WORK/serve.log"; exit 3; }

URL="https://127.0.0.1:$PORT$BASE"
worst=0
skipped=""
for mode in chrome chrome-noserial firefox mavlink; do
  ( cd "$HERE" && node tools/browser-check.mjs "$URL" "$mode" )
  rc=$?
  # 0 pass, 1 a check failed, 3 the browser (or, in mavlink mode, the bridge
  # behind it) would not start.
  #
  # 3 is not a failure of the app: Firefox does not launch on this machine and
  # pymavlink may not be installed, and neither is something this script can
  # fix. But it is not a pass either, and the exit code alone cannot carry that
  # — `chrome` passing beside a `mavlink` that never started exits 0, the same
  # as a run where all four ran. So the modes that did not run are *named in the
  # verdict* rather than dropped, which is what the loop below used to do: it
  # recorded nothing, and the last line said "every browser that ran passed"
  # about a run in which one browser had tested nothing at all.
  if [ "$rc" -eq 3 ]; then
    skipped="$skipped $mode"
  elif [ "$rc" -ne 0 ]; then
    worst=1
  fi
done

echo
if [ "$worst" -ne 0 ]; then
  echo "== FAILURES above"
  exit 1
fi
if [ -n "$skipped" ]; then
  echo "== every browser that ran passed, and these did not run:$skipped"
  echo "   not a failure of the app, and not a pass either — each one's own"
  echo "   '!!' line above says what was missing."
  exit 0
fi
echo "== every browser that ran passed"
exit 0
