#!/bin/bash
set -e

echo "=== Smart Retainer Build & Flash Script ==="

# Build application core
echo "[1/6] Building application core..."
cd /Users/zhaohaichao/Desktop/blinky/
rm -rf build/
west build -b nrf7002dk/nrf5340/cpuapp -p 2>&1 | tee build_output_app.log
APP_HEX="/Users/zhaohaichao/Desktop/blinky/build/zephyr/zephyr.hex"
echo "  Application hex: $APP_HEX"
ls -lh "$APP_HEX"

# Build network core
echo "[2/6] Building network core..."
cd /opt/nordic/ncs/v3.0.2/zephyr/samples/bluetooth/hci_ipc
rm -rf build/
west build -b nrf5340dk/nrf5340/cpunet -p 2>&1 | tee build_output_net.log

# Find the network core hex file
echo "  Looking for network core hex..."
NET_HEX=$(find build/zephyr -name "*.hex" | head -1)
if [ -z "$NET_HEX" ]; then
    echo "ERROR: Network core hex not found!"
    echo "Build directory contents:"
    ls -la build/zephyr/
    exit 1
fi
echo "  Network hex: $NET_HEX"
ls -lh "$NET_HEX"

# Erase device
echo "[3/6] Erasing device..."
nrfjprog --recover --family NRF53

# Flash network core
echo "[4/6] Flashing network core..."
nrfjprog --program "$NET_HEX" --sectorerase --verify --family NRF53

# Flash application core  
echo "[5/6] Flashing application core..."
nrfjprog --program "$APP_HEX" --sectorerase --verify --family NRF53

# Reset device
echo "[6/6] Resetting device..."
nrfjprog --reset --family NRF53

echo ""
echo "=== ✅ Flash Complete! ==="
echo "Run: cd /Users/zhaohaichao/Desktop/blinky && west attach"
