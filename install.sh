#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# OAK-Drive-Sync — Raspberry Pi 5 (32-bit) One-Click Installer
# ═══════════════════════════════════════════════════════════════
#
# Usage:  chmod +x install.sh && ./install.sh
#
# What this script does:
#   1. Installs system packages (Python, Mosquitto, GPIO libs)
#   2. Installs uv (Python package manager)
#   3. Installs Tinkerforge Brick Daemon
#   4. Sets up the Python virtual environment + dependencies
#   5. Configures Mosquitto MQTT broker
#   6. Adds user to gpio group
#   7. Installs systemd services (auto-start on boot)
#   8. Verifies everything works
#
set -euo pipefail

# ── Colors for output ────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Pre-flight checks ────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
echo "  OAK-Drive-Sync — Raspberry Pi 5 Installer"
echo "═══════════════════════════════════════════════════════"
echo ""

# Must run as normal user (not root), will use sudo when needed
if [ "$(id -u)" -eq 0 ]; then
    error "Do not run as root. Run as your normal user — the script uses sudo when needed."
fi

# Detect project directory (where this script lives)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
USER_NAME="$(whoami)"
USER_HOME="$HOME"

info "Project directory: $PROJECT_DIR"
info "User: $USER_NAME"
info "Home: $USER_HOME"
echo ""

# ── Step 1: System packages ──────────────────────────────────
info "Step 1/8: Installing system packages..."

sudo apt update -qq
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    mosquitto \
    mosquitto-clients \
    git \
    curl \
    libgpiod2 \
    python3-lgpio \
    python3-gpiozero \
    build-essential \
    libffi-dev \
    libssl-dev

ok "System packages installed"

# ── Step 2: Install uv ───────────────────────────────────────
info "Step 2/8: Installing uv package manager..."

if command -v uv &>/dev/null; then
    ok "uv already installed: $(uv --version)"
else
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # Add to PATH for this session
    export PATH="$USER_HOME/.local/bin:$PATH"
    ok "uv installed: $(uv --version)"
fi

# Ensure uv is on PATH for future sessions
if ! grep -q 'astral' "$USER_HOME/.bashrc" 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$USER_HOME/.bashrc"
    info "Added uv to .bashrc PATH"
fi

# ── Step 3: Install Tinkerforge Brick Daemon ──────────────────
info "Step 3/8: Installing Tinkerforge Brick Daemon..."

if command -v brickd &>/dev/null || systemctl is-active --quiet brickd 2>/dev/null; then
    ok "Brick Daemon already installed"
else
    # Tinkerforge APT repository for 32-bit armhf
    if [ ! -f /etc/apt/sources.list.d/tinkerforge.list ]; then
        sudo bash -c 'echo "deb https://download.tinkerforge.com/apt/raspbian bullseye main" > /etc/apt/sources.list.d/tinkerforge.list'
        wget -q https://download.tinkerforge.com/apt/raspbian/tinkerforge.gpg -O - | sudo apt-key add - 2>/dev/null || true
        sudo apt update -qq
    fi
    sudo apt install -y brickd || warn "Brick Daemon install failed — Tinkerforge drives will run in simulation mode. Install manually from tinkerforge.com if needed."
    ok "Brick Daemon installed"
fi

# Start brickd if available
if systemctl list-unit-files | grep -q brickd; then
    sudo systemctl enable brickd
    sudo systemctl start brickd || true
    ok "Brick Daemon enabled and started"
fi

# ── Step 4: Python virtual environment + dependencies ────────
info "Step 4/8: Setting up Python environment..."

cd "$PROJECT_DIR"

# Create venv and install with Pi extras
uv venv --python python3
uv pip install -e ".[pi]"

ok "Python dependencies installed"

# Verify key imports
info "Verifying Python packages..."
uv run python3 -c "import aiomqtt; print('  aiomqtt OK')"
uv run python3 -c "import pydantic; print('  pydantic OK')"
uv run python3 -c "import structlog; print('  structlog OK')"
uv run python3 -c "import nicegui; print('  nicegui OK')"
uv run python3 -c "
try:
    import lgpio; print('  lgpio OK')
except ImportError:
    print('  lgpio NOT AVAILABLE (GPIO will run in simulation mode)')
"
uv run python3 -c "
try:
    from tinkerforge.ip_connection import IPConnection; print('  tinkerforge OK')
except ImportError:
    print('  tinkerforge NOT AVAILABLE (Tinkerforge drives will run in simulation mode)')
"

ok "Python environment ready"

# ── Step 5: Configure Mosquitto ───────────────────────────────
info "Step 5/8: Configuring Mosquitto MQTT broker..."

# Install our drop-in config
sudo cp "$PROJECT_DIR/config/mosquitto.conf" /etc/mosquitto/conf.d/oak-drive-sync.conf

# Enable and restart
sudo systemctl enable mosquitto
sudo systemctl restart mosquitto

# Verify broker is running
sleep 1
if mosquitto_pub -h localhost -t "test/install" -m "ok" -q 0 2>/dev/null; then
    ok "Mosquitto broker running on port 1883"
else
    warn "Mosquitto may not be running — check: sudo systemctl status mosquitto"
fi

# ── Step 6: GPIO permissions ─────────────────────────────────
info "Step 6/8: Setting up GPIO permissions..."

if id -nG "$USER_NAME" | grep -qw gpio; then
    ok "User already in gpio group"
else
    sudo usermod -aG gpio "$USER_NAME"
    warn "Added $USER_NAME to gpio group — YOU MUST LOG OUT AND BACK IN for this to take effect"
fi

# ── Step 7: Install systemd service ──────────────────────────
info "Step 7/8: Installing systemd service..."

# Generate service file with correct paths for this user/install
SERVICE_FILE="/etc/systemd/system/oak-drive-controller.service"

sudo tee "$SERVICE_FILE" > /dev/null <<SERVICEEOF
[Unit]
Description=OAK-Drive-Sync Pi Drive Controller
After=network-online.target mosquitto.service
Wants=network-online.target
Requires=mosquitto.service

[Service]
Type=exec
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$PROJECT_DIR
ExecStart=$USER_HOME/.local/bin/uv run oak-pi run --config config/drive_config.yaml
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

# GPIO access
SupplementaryGroups=gpio

# Environment
Environment=PYTHONUNBUFFERED=1
Environment=PATH=$USER_HOME/.local/bin:/usr/local/bin:/usr/bin:/bin

[Install]
WantedBy=multi-user.target
SERVICEEOF

sudo systemctl daemon-reload
sudo systemctl enable oak-drive-controller

ok "Systemd service installed (oak-drive-controller)"
info "  Start now:  sudo systemctl start oak-drive-controller"
info "  View logs:  journalctl -u oak-drive-controller -f"

# ── Step 8: Verify ───────────────────────────────────────────
info "Step 8/8: Running verification..."

echo ""
echo "───────────────────────────────────────────────────────"
echo "  Installation Summary"
echo "───────────────────────────────────────────────────────"

# Python
PYTHON_VER=$(uv run python3 --version 2>&1)
echo -e "  Python:        ${GREEN}$PYTHON_VER${NC}"

# uv
UV_VER=$(uv --version 2>&1)
echo -e "  uv:            ${GREEN}$UV_VER${NC}"

# Mosquitto
if systemctl is-active --quiet mosquitto; then
    echo -e "  Mosquitto:     ${GREEN}running${NC}"
else
    echo -e "  Mosquitto:     ${RED}not running${NC}"
fi

# Brick Daemon
if systemctl is-active --quiet brickd 2>/dev/null; then
    echo -e "  Brick Daemon:  ${GREEN}running${NC}"
else
    echo -e "  Brick Daemon:  ${YELLOW}not running (ok if no Tinkerforge hardware yet)${NC}"
fi

# GPIO group
if id -nG "$USER_NAME" | grep -qw gpio; then
    echo -e "  GPIO group:    ${GREEN}yes${NC}"
else
    echo -e "  GPIO group:    ${YELLOW}pending logout/login${NC}"
fi

# Service
if systemctl is-enabled --quiet oak-drive-controller 2>/dev/null; then
    echo -e "  Service:       ${GREEN}enabled (auto-start on boot)${NC}"
else
    echo -e "  Service:       ${RED}not enabled${NC}"
fi

echo ""
echo "───────────────────────────────────────────────────────"
echo "  Next Steps"
echo "───────────────────────────────────────────────────────"
echo ""
echo "  1. Edit config/drive_config.yaml"
echo "     - Set Tinkerforge bricklet UIDs (replace CHANGE_ME)"
echo "     - Adjust GPIO pins if needed"
echo ""
echo "  2. Start the controller:"
echo "     cd $PROJECT_DIR"
echo "     uv run oak-pi run"
echo ""
echo "  3. Open the GUI in a browser:"
echo "     http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "  4. Calibrate drives on the Calibration tab"
echo ""
echo "  5. For auto-start on boot:"
echo "     sudo systemctl start oak-drive-controller"
echo ""
echo -e "  ${GREEN}Installation complete!${NC}"
echo ""
