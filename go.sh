#!/usr/bin/env bash
# rc car launcher
# usage: sudo bash go.sh
set -euo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
CYN='\033[0;36m'
RST='\033[0m'

log()  { echo -e "${CYN}[$(date +%H:%M:%S)]${RST} $1"; }
ok()   { echo -e "${GRN}[✓]${RST} $1"; }
warn() { echo -e "${YLW}[!]${RST} $1"; }
die()  { echo -e "${RED}[✗] $1${RST}"; exit 1; }

# must be root
[[ $EUID -eq 0 ]] || die "Run with sudo: sudo bash go.sh"

# kill stale
log "Cleaning up stale processes..."
pkill -9 -f servo_joy   2>/dev/null || true
pkill -9 -f telem_relay 2>/dev/null || true
pkill -9 -f "dashboard/app" 2>/dev/null || true
sleep 1

# load modules
log "Loading kernel modules..."
for mod in usbserial cp210x ch341 ftdi_sio; do
    modprobe "$mod" 2>/dev/null || true
done
ok "Kernel modules loaded"

# wait serial
SERIAL=""
log "Waiting for ESP32 serial port..."
for i in $(seq 1 30); do
    for dev in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
        if [ -e "$dev" ]; then
            SERIAL="$dev"
            break 2
        fi
    done
    sleep 1
    printf "."
done
echo ""
[ -n "$SERIAL" ] || die "no serial after 30s"
chmod 666 "$SERIAL"
ok "Serial port: $SERIAL"

# sanity check
python3 -c "
import serial, sys
try:
    s = serial.Serial('$SERIAL', 115200, timeout=1)
    s.close()
except Exception as e:
    print(f'Serial test failed: {e}', file=sys.stderr)
    sys.exit(1)
" || die "cannot open $SERIAL"
ok "Serial port verified"

# wait xbox
log "Waiting for Xbox controller..."
XBOX_FOUND=0
for i in $(seq 1 20); do
    if python3 -c "
import usb.core
d = usb.core.find(idVendor=0x045E, idProduct=0x028E)
exit(0 if d else 1)
" 2>/dev/null; then
        XBOX_FOUND=1
        break
    fi
    sleep 1
    printf "."
done
echo ""
[ "$XBOX_FOUND" -eq 1 ] || die "no xbox after 20s"
ok "Xbox controller detected"

# launch
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo -e "${GRN}═══════════════════════════════════════${RST}"
echo -e "${GRN}  ALL SYSTEMS GO — LAUNCHING CONTROLLER${RST}"
echo -e "${GRN}═══════════════════════════════════════${RST}"
echo -e "  Serial:     ${CYN}$SERIAL${RST}"
echo -e "  Controller: ${CYN}Xbox 360${RST}"
echo -e "  Controls:   ${YLW}LStick X${RST}=steer  ${YLW}RStick Y${RST}=motor"
echo -e "              ${YLW}A${RST}=sus(manual)  ${YLW}B${RST}=active PID"
echo -e "  ${RED}Ctrl+C to stop${RST}"
echo ""

exec python3 "$SCRIPT_DIR/tools/servo_joy.py" --port "$SERIAL"
