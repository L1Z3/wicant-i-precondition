#!/bin/bash
set -euo pipefail

if ! command -v nmcli >/dev/null 2>&1; then
  echo "ERROR: nmcli not found" >&2
  exit 1
fi

BIN=$(ls -t build.custom/*.bin build.v300/*.bin 2>/dev/null | head -1 || true)
OTA_URL="http://192.168.80.1/upload/ota.bin"
PAGE_URL="http://192.168.80.1/"
WIFI_PASS="@meatpi#"
RESUME_SSID=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes:' | head -1 | cut -d: -f2- || true)
STATIC_IP="192.168.80.100/24"
VERIFY_STR='section-title">Buttons'
DEVICE_HOST="192.168.80.1"
FLASH_WAIT_MAX=120
BOOT_WAIT_MAX=180
WICAN_SEARCH_STRING="^WiCAN_"

if [[ ! -f "$BIN" ]]; then
  echo "ERROR: Firmware binary not found at $BIN" >&2
  exit 1
fi

WIFI_IF=$(nmcli -t -f DEVICE,TYPE dev status | grep ':wifi$' | head -1 | cut -d: -f1)
if [[ -z "$WIFI_IF" ]]; then
  echo "ERROR: No WiFi interface found" >&2
  exit 1
fi
echo "Using WiFi interface: $WIFI_IF"

cleanup_conn() {
  local name="$1"
  nmcli connection down "$name" 2>/dev/null || true
  nmcli connection delete "$name" 2>/dev/null || true
}

connect_ssid() {
  local ssid="$1" conn_name="$2"
  cleanup_conn "$conn_name" 2>/dev/null || true
  nmcli connection add type wifi con-name "$conn_name" ssid "$ssid" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_PASS" wifi-sec.psk-flags 0 \
    autoconnect no 2>/dev/null
  nmcli connection modify "$conn_name" \
    ipv4.method manual ipv4.addresses "$STATIC_IP" ipv4.never-default true
  timeout 45 nmcli connection up "$conn_name" >/dev/null 2>&1 || return 1
  sleep 3
}

wait_for_http() {
  local max_secs="$1"
  for i in $(seq 1 $max_secs); do
    CODE=$(curl -s -o /dev/null -w "%{http_code}" --connect-timeout 2 --max-time 4 "$PAGE_URL" 2>/dev/null || true)
    if [[ "$CODE" != "000" ]]; then
      echo "$CODE"
      return 0
    fi
    sleep 1
  done
  echo "000"
  return 1
}

get_ssids() {
  nmcli dev wifi rescan >/dev/null 2>&1 || true
  sleep 2
  nmcli -t -f SSID dev wifi list | grep -E '^${WICAN_SEARCH_STRING}' | sort -u
}

flash_one() {
  local ssid="$1"
  local CONN_NAME="wican_flash"

  echo ""
  echo "=== Flashing '$ssid' ==="

  if ! connect_ssid "$ssid" "$CONN_NAME"; then
    echo "FAILED to connect to '$ssid', skipping."
    cleanup_conn "$CONN_NAME"
    return 1
  fi
  echo "Connected to '$ssid'"

  CODE=$(wait_for_http 30)
  if [[ "$CODE" != "200" ]]; then
    echo "Device at $DEVICE_HOST not serving HTTP (got $CODE), skipping."
    cleanup_conn "$CONN_NAME"
    return 1
  fi
  echo "Device web interface is up (HTTP 200)"

  echo "Flashing firmware..."
  HTTP_CODE=$(curl -s -o /tmp/ota_resp.txt -w "%{http_code}" \
    --connect-timeout 10 --max-time 120 \
    -H "Expect:" \
    -F "ota_file=@${BIN}" \
    "$OTA_URL" || true)

  if [[ "$HTTP_CODE" != "303" ]]; then
    echo "Flash FAILED (HTTP $HTTP_CODE) for '$ssid'"
    rm -f /tmp/ota_resp.txt
    cleanup_conn "$CONN_NAME"
    return 1
  fi
  echo "Flash SUCCESS (HTTP 303) for '$ssid'"
  echo "  Response: $(cat /tmp/ota_resp.txt 2>/dev/null | head -c 200)"
  rm -f /tmp/ota_resp.txt

  cleanup_conn "$CONN_NAME"
  echo "Disconnected from '$ssid'"
  return 0
}

verify_one() {
  local ssid="$1"
  local CONN_NAME="wican_verify"

  echo ""
  echo "=== Verifying '$ssid' ==="

  if ! connect_ssid "$ssid" "$CONN_NAME"; then
    echo "FAILED to connect to '$ssid' during verify, skipping."
    cleanup_conn "$CONN_NAME"
    return 1
  fi
  echo "Connected to '$ssid'"

  CODE=$(wait_for_http $BOOT_WAIT_MAX)
  if [[ "$CODE" != "200" ]]; then
    echo "FAILED: device did not come back within ${BOOT_WAIT_MAX}s (HTTP $CODE)."
    echo "        It may need a power cycle."
    cleanup_conn "$CONN_NAME"
    return 1
  fi

  PAGE=$(curl -s --connect-timeout 5 "$PAGE_URL" || true)
  if grep -q "$VERIFY_STR" <<< "$PAGE"; then
    echo "VERIFIED: page contains '$VERIFY_STR'"
    cleanup_conn "$CONN_NAME"
    echo "Disconnected from '$ssid'"
    return 0
  fi

  echo "FAILED: page loaded but '$VERIFY_STR' NOT found - flash may not have taken effect"
  cleanup_conn "$CONN_NAME"
  echo "Disconnected from '$ssid'"
  return 1
}

echo "Scanning for WiCAN_4* networks..."
SSIDS=$(get_ssids)

if [[ -z "$SSIDS" ]]; then
  echo "No WiCAN_4* networks found."
  exit 0
fi

echo "Found networks:"
echo "$SSIDS"
echo "---"
echo "=== PASS 1: FLASH ALL ==="
flash_count=0
flash_fail=0

while IFS= read -r ssid; do
  if flash_one "$ssid"; then
    ((flash_count += 1))
  else
    ((flash_fail += 1))
  fi
  sleep 2
done <<< "$SSIDS"

echo ""
echo "=== Flash results: $flash_count flashed, $flash_fail failed ==="

echo ""
echo "=== PASS 2: VERIFY ALL ==="
verify_count=0
verify_fail=0

while IFS= read -r ssid; do
  if verify_one "$ssid"; then
    ((verify_count += 1))
  else
    ((verify_fail += 1))
  fi
  sleep 2
done <<< "$SSIDS"

echo ""
echo "=== Verify results: $verify_count verified, $verify_fail failed ==="
echo ""
echo "=== Overall: $flash_count flashed, $verify_count verified, $flash_fail flash-failures, $verify_fail verify-failures ==="
echo ""
echo "Reconnecting to $RESUME_SSID..."
if ! timeout 60 nmcli dev wifi connect "$RESUME_SSID" 2>&1; then
  echo "WARNING: Failed to reconnect to $RESUME_SSID, retrying..."
  sleep 5
  timeout 60 nmcli dev wifi connect "$RESUME_SSID" 2>&1 || echo "WARNING: Still failed to reconnect to $RESUME_SSID"
fi
echo "Done."
