#!/bin/bash

echo "Resetting RealSense camera..."

# Step 1: Find the device bus and device number
BUS_DEVICE=$(lsusb | grep -i "8086:0b3a" | awk '{printf("/dev/bus/usb/%03d/%03d", $2, $4)}' | sed 's/://')

if [ -z "$BUS_DEVICE" ] || [ ! -e "$BUS_DEVICE" ]; then
    echo "❌ RealSense device not found or path invalid!"
    exit 1
fi

echo "➡️  Found device at $BUS_DEVICE"
echo "➡️  Performing USB reset..."
sudo usbreset "$BUS_DEVICE"

if [ $? -eq 0 ]; then
    echo "✅ Reset complete."
else
    echo "❌ Reset failed."
fi
