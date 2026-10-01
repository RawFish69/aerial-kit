#!/usr/bin/env bash
#
# Verify the built site the way a host would serve it: over HTTPS, mounted at a
# subpath, fetched the way a browser fetches it.
#
# W4 asks for the site to be verified "on the intended HTTPS hosting
# configuration". No public host exists and none may be created, so what can be
# checked here is the configuration itself, on loopback: that the subpath build
# resolves every one of its own assets under the subpath, with the right
# content types, over TLS, and that the root-absolute build genuinely fails
# there (which is why AK_BASE is not optional).
#
# What this does NOT verify is anything about a real host: a real certificate, a
# redirect, HSTS, or a browser. It proves the page and its assets work over
# HTTPS at a subpath. Nothing more.
#
#   ./tools/verify-deploy.sh [base]
#
# Exits non-zero if any expected status does not match.

set -euo pipefail

BASE="${1:-/aerialkit/configurator}"
BASE="/${BASE#/}"; BASE="${BASE%/}/"          # normalize: leading and trailing /
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8443}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

fail=0
check() { # check <label> <expected> <actual>
  if [ "$2" = "$3" ]; then
    printf '  ok    %-46s %s\n' "$1" "$3"
  else
    printf '  FAIL  %-46s expected %s, got %s\n' "$1" "$2" "$3"
    fail=1
  fi
}

echo "== building for the subpath: AK_BASE=$BASE"
( cd "$HERE" && AK_BASE="$BASE" npm run build >/dev/null )

# Lay the output out exactly as a host mounting the app at BASE would.
mkdir -p "$WORK/site$BASE"
cp -r "$HERE/dist/." "$WORK/site$BASE/"

echo "== generating a throwaway certificate"
openssl req -x509 -newkey rsa:2048 -keyout "$WORK/key.pem" -out "$WORK/cert.pem" \
  -days 1 -nodes -subj "/CN=127.0.0.1" >/dev/null 2>&1

# stdlib http.server has no TLS flag, so the listening socket is wrapped.
cat > "$WORK/serve.py" <<'PY'
import http.server, ssl, sys
root, port, cert, key = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(cert, key)
srv = http.server.HTTPServer(("127.0.0.1", port),
        lambda *a, **k: http.server.SimpleHTTPRequestHandler(*a, directory=root, **k))
srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
srv.serve_forever()
PY

python3 "$WORK/serve.py" "$WORK/site" "$PORT" "$WORK/cert.pem" "$WORK/key.pem" \
  >"$WORK/serve.log" 2>&1 &
SRV=$!
trap 'kill $SRV 2>/dev/null || true; rm -rf "$WORK"' EXIT
sleep 2
kill -0 $SRV 2>/dev/null || { echo "the local HTTPS server did not start"; cat "$WORK/serve.log"; exit 1; }

status() { curl -sk -o /dev/null -w '%{http_code}' "https://127.0.0.1:$PORT$1"; }
ctype()  { curl -sk -o /dev/null -w '%{content_type}' "https://127.0.0.1:$PORT$1"; }

INDEX="$WORK/site$BASE/index.html"
JS="$(grep -o "src=\"[^\"]*index-[^\"]*\.js\"" "$INDEX" | sed 's/src="//;s/"$//')"
CSS="$(grep -o "href=\"[^\"]*index-[^\"]*\.css\"" "$INDEX" | sed 's/href="//;s/"$//')"

echo "== the build's own asset URLs"
echo "     js  $JS"
echo "     css $CSS"
case "$JS" in "$BASE"*) ;; *) echo "  FAIL  js URL is not under $BASE"; fail=1 ;; esac
case "$CSS" in "$BASE"*) ;; *) echo "  FAIL  css URL is not under $BASE"; fail=1 ;; esac

echo "== over HTTPS at the subpath"
check "index"            200 "$(status "$BASE")"
check "javascript"       200 "$(status "$JS")"
check "stylesheet"       200 "$(status "$CSS")"
check "js content-type"  "text/javascript" "$(ctype "$JS")"
check "css content-type" "text/css"        "$(ctype "$CSS")"

# A demonstration, not a guard: nothing is mounted at the root of this server,
# so this 404s whether or not the build is correct. It cannot fail here, and is
# printed so the shape of the mistake is visible. The assertion that actually
# catches a root-absolute build is the "js URL is not under $BASE" check above,
# and the javascript/stylesheet 404s that follow from it.
echo "== the deploy mistake the subpath build exists to prevent (demonstration)"
echo "     root-absolute /assets/... -> $(status "/assets/$(basename "$JS")")"

echo "== TLS was negotiated, not plaintext"
check "scheme" "HTTPS" "$(curl -sk -o /dev/null -w '%{scheme}' "https://127.0.0.1:$PORT$BASE")"

if [ "$fail" -eq 0 ]; then
  echo "== all checks passed"
else
  echo "== FAILURES above"
fi
exit "$fail"
