#!/usr/bin/env bash
# OpenManipulator-X teleop installer for Raspberry Pi OS (Bookworm, 64-bit).
#
# Idempotent: safe to re-run after a git pull. Everything it changes is either
# inside the repo directory or a clearly-named file in /etc.
#
# Usage:   ./deploy/install_rpi.sh            # install + enable the service
#          ./deploy/install_rpi.sh --no-service  # set up only, don't autostart
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="${SUDO_USER:-$USER}"
VENV="$REPO/opmx"
ENV_FILE="/etc/omx.env"
UNIT="/etc/systemd/system/omx@.service"
WANT_SERVICE=1
[[ "${1:-}" == "--no-service" ]] && WANT_SERVICE=0

say(){ printf "\n\033[1;36m==> %s\033[0m\n" "$*"; }
warn(){ printf "\033[1;33m!!  %s\033[0m\n" "$*"; }

[[ $EUID -eq 0 ]] && { echo "Run as your normal user, not root - it will sudo where needed."; exit 1; }

say "Repo:  $REPO"
say "User:  $USER_NAME"

# ---------------------------------------------------------------- packages
say "Installing system packages"
sudo apt-get update -qq
# python3-tk: the control panel. xvfb: lets it run with no monitor attached.
# libgl1/libglib2.0-0: MuJoCo's shared-library deps even when not rendering.
# Debian 13 (trixie) renamed libglib2.0-0 to libglib2.0-0t64 in the 64-bit
# time_t transition; on Bookworm only the old name exists. Pick whichever the
# running release actually offers, so this works on both.
GLIB_PKG=libglib2.0-0
if [[ -z "$(apt-cache policy libglib2.0-0 2>/dev/null | awk '/Candidate:/{print $2}' | grep -v '(none)')" ]]; then
    GLIB_PKG=libglib2.0-0t64
fi
echo "  glib package: $GLIB_PKG"
sudo apt-get install -y --no-install-recommends \
    python3-venv python3-dev python3-tk xvfb git curl \
    libgl1 "$GLIB_PKG"

# ---------------------------------------------------------------- venv
say "Creating virtualenv at $VENV"
[[ -d "$VENV" ]] || python3 -m venv "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip -q
"$VENV/bin/python" -m pip install -r "$REPO/requirements.txt"

say "Verifying imports"
"$VENV/bin/python" - <<'PY'
import mujoco, numpy, pygame, dynamixel_sdk, tkinter
print("  mujoco %s | numpy %s | pygame %s | tk %s"
      % (mujoco.__version__, numpy.__version__, pygame.__version__, tkinter.TkVersion))
PY

say "Checking the MuJoCo model loads"
"$VENV/bin/python" -c "
import mujoco, os
m = mujoco.MjModel.from_xml_path(os.path.join('$REPO','open_manipulator_x.xml'))
print('  model OK: nq=%d nu=%d' % (m.nq, m.nu))"

# ---------------------------------------------------------------- serial
say "Serial port access"
if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx dialout; then
    echo "  $USER_NAME is already in the dialout group"
else
    sudo usermod -aG dialout "$USER_NAME"
    warn "Added $USER_NAME to dialout - log out and back in for interactive use."
    warn "(The systemd service gets the group directly, so it works without that.)"
fi

# The FTDI default latency timer is 16 ms, which alone eats the entire budget
# for a 60 Hz loop. 1 ms takes a sync-read from ~16 ms to low single digits.
say "Installing FTDI low-latency udev rule"
echo 'SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
    | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger || true
for dev in /sys/bus/usb-serial/devices/ttyUSB*/latency_timer; do
    [[ -e "$dev" ]] && echo "  $(dirname "$dev" | xargs basename) latency_timer = $(cat "$dev")"
done

# ---------------------------------------------------------------- config
say "Configuring $ENV_FILE"
if sudo test -f "$ENV_FILE" && sudo grep -q OMX_WEB_TOKEN "$ENV_FILE"; then
    echo "  keeping the existing token in $ENV_FILE"
else
    TOKEN="$(head -c 24 /dev/urandom | base64 | tr -d '/+=' | head -c 32)"
    sudo tee "$ENV_FILE" > /dev/null <<EOF
# OpenManipulator-X teleop service configuration.

# Shared secret for the web panel. Anyone with this can move the arm.
OMX_WEB_TOKEN=$TOKEN

# Arguments passed to digital_twin_gui.py.
#   --headless    no 3D viewer, no desktop panel (right for a carried Pi)
#   --no-viewer   keep the desktop panel, drop the expensive 3D window
#   --bind        127.0.0.1 accepts only local + tunnelled traffic
OMX_ARGS=--headless --bind 0.0.0.0
EOF
    sudo chmod 600 "$ENV_FILE"
    echo "  generated a new token"
fi

# ---------------------------------------------------------------- service
if [[ $WANT_SERVICE -eq 1 ]]; then
    say "Installing systemd service"
    sudo cp "$REPO/deploy/omx@.service" "$UNIT"
    sudo systemctl daemon-reload
    sudo systemctl enable "omx@$USER_NAME.service"
    sudo systemctl restart "omx@$USER_NAME.service"
    sleep 4
    if systemctl is-active --quiet "omx@$USER_NAME.service"; then
        echo "  service is running"
    else
        warn "Service failed to start. Logs:"
        sudo journalctl -u "omx@$USER_NAME.service" -n 30 --no-pager || true
        exit 1
    fi
fi

# ---------------------------------------------------------------- summary
TOKEN_VAL="$(sudo grep -oP 'OMX_WEB_TOKEN=\K.*' "$ENV_FILE")"
IP="$(hostname -I | awk '{print $1}')"
say "Done"
cat <<EOF

  Web panel : http://$IP:8080/?token=$TOKEN_VAL
  Token     : $TOKEN_VAL   (stored in $ENV_FILE, mode 600)

  Open that full URL once on the phone; it stores a cookie so later visits
  need no token in the address bar.

  systemctl status  omx@$USER_NAME
  journalctl -u omx@$USER_NAME -f
  sudo systemctl restart omx@$USER_NAME

  Remote access beyond this LAN still needs a tunnel - see deploy/RPI_SETUP_PROMPT.md.

EOF
