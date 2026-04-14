#!/usr/bin/env bash
# OAK-Drive-Sync — Install desktop shortcut on Raspberry Pi OS
#
# Run once after cloning: bash install-desktop-shortcut.sh
# Creates ~/Desktop/OAK-Drive-Controller.desktop so you can double-click to launch.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
SCRIPT="$PROJECT_DIR/start.sh"
DESKTOP_FILE="$HOME/Desktop/OAK-Drive-Controller.desktop"

# Make start.sh executable
chmod +x "$SCRIPT"

# Write the desktop entry with the real project path
cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=OAK Drive Controller
Comment=Start the OAK-D drive controller GUI (http://localhost:8080)
Icon=utilities-terminal
Exec=lxterminal --title="OAK Drive Controller" -e $SCRIPT
Path=$PROJECT_DIR
Terminal=false
StartupNotify=false
Categories=Utility;
EOF

chmod +x "$DESKTOP_FILE"

# Trust the desktop file (Raspberry Pi OS Bookworm requires this)
if command -v gio &>/dev/null; then
    gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null || true
fi

echo "Shortcut installed to: $DESKTOP_FILE"
echo "Double-click 'OAK Drive Controller' on your desktop to launch."
