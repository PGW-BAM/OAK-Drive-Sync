#!/usr/bin/env bash
# OAK-Drive-Sync — Start the Pi drive controller with uv
# Run this from anywhere; it always resolves to the project directory.

cd "$(dirname "$(realpath "$0")")"

echo "Starting OAK Drive Controller..."
echo "GUI will be available at http://localhost:8080"
echo ""

uv run oak-pi run

echo ""
echo "Controller stopped. Press Enter to close."
read -r
