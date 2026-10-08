#!/usr/bin/env bash
# Mint the XR TLS material for televuer (:8012) and teleimager (:60001): a root
# CA plus a server certificate naming every address the headset or a browser
# may use for PC2. The original CA's key was never kept, so any new address
# means a new CA; the headset installs it once from http://<pc2>:8088/rootCA.crt
# and trusts it (Settings > General > About > Certificate Trust Settings).
# Compare the SHA-256 fingerprint printed below with the one the headset shows.
# Re-run after the Wi-Fi address changes; then restart teleimager and xr-teleop.
set -euo pipefail
CFG="${XR_CONFIG_DIR:-$HOME/.config/xr_teleoperate}"
ADDRS="${XR_CERT_ADDRS:-127.0.0.1 192.168.123.164 10.2.100.186 10.2.100.240}"
STAMP="$(date +%Y-%m-%d)"
CA="$CFG/ca-$STAMP"
PUB="$CFG/public"
mkdir -p "$CA" "$PUB"
for f in cert.pem key.pem; do
  [ -f "$CFG/$f" ] && cp -n "$CFG/$f" "$CFG/$f.bak-$STAMP"
done
cd "$CA"
openssl genrsa -out rootCA.key 2048 2>/dev/null
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 1095 -out rootCA.pem \
  -subj "/CN=xr-teleoperate-h1-2-$STAMP" \
  -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=critical,keyCertSign,cRLSign"
openssl genrsa -out key.pem 2048 2>/dev/null
openssl req -new -key key.pem -out server.csr -subj "/CN=h1-2-pc2"
{
  echo "authorityKeyIdentifier=keyid,issuer"
  echo "basicConstraints=CA:FALSE"
  echo "keyUsage = digitalSignature, keyEncipherment"
  echo "extendedKeyUsage = serverAuth"
  echo "subjectAltName = @alt_names"
  echo "[alt_names]"
  echo "DNS.1 = localhost"
  i=1; for a in $ADDRS; do echo "IP.$i = $a"; i=$((i+1)); done
} > v3.ext
openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key -CAcreateserial \
  -out cert.pem -days 398 -sha256 -extfile v3.ext 2>/dev/null
cp cert.pem "$CFG/cert.pem"; cp key.pem "$CFG/key.pem"; chmod 600 "$CFG/key.pem" rootCA.key
cp rootCA.pem "$PUB/rootCA.crt"; cp rootCA.pem "$PUB/rootCA.pem"
openssl verify -CAfile rootCA.pem cert.pem
openssl x509 -in "$CFG/cert.pem" -noout -issuer -ext subjectAltName -dates
echo "rootCA SHA-256 fingerprint (compare on the headset before trusting):"
openssl x509 -in rootCA.pem -noout -fingerprint -sha256
echo "installed to $CFG; serve to the headset as http://<pc2>:8088/rootCA.crt; restart teleimager.service and xr-teleop.service"
