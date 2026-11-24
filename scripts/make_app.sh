#!/bin/sh
# Builds a release VehicleSim.app (double-clickable) and the vdsim CLI into ./dist.
set -eu
cd "$(dirname "$0")/.."
swift build -c release --product VehicleSimApp
swift build -c release --product vdsim
BIN=$(swift build -c release --show-bin-path)
APP=dist/VehicleSim.app
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp "$BIN/VehicleSimApp" "$APP/Contents/MacOS/VehicleSim"
cp "$BIN/vdsim" dist/vdsim
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>VehicleSim</string>
  <key>CFBundleDisplayName</key><string>Vehicle Dynamics</string>
  <key>CFBundleIdentifier</key><string>local.vehiclesim</string>
  <key>CFBundleExecutable</key><string>VehicleSim</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>CFBundleVersion</key><string>1</string>
  <key>LSMinimumSystemVersion</key><string>14.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSPrincipalClass</key><string>NSApplication</string>
</dict></plist>
PLIST
codesign --force --sign - "$APP" >/dev/null
echo "Built $APP and dist/vdsim"
