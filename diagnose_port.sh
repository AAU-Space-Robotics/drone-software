#!/bin/bash

echo "=== RTK Base Station Diagnostics ==="
echo ""

# Check if port exists
echo "1. Checking if port exists..."
if [ -e /dev/ttyACM0 ]; then
    echo "✓ /dev/ttyACM0 exists"
else
    echo "✗ /dev/ttyACM0 not found"
    echo "Available ports:"
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No USB serial ports found"
    exit 1
fi

# Check permissions
echo ""
echo "2. Checking permissions..."
ls -la /dev/ttyACM0

# Check who has the port open
echo ""
echo "3. Checking if port is in use..."
if command -v lsof &> /dev/null; then
    PROCESS=$(lsof /dev/ttyACM0 2>/dev/null)
    if [ -n "$PROCESS" ]; then
        echo "⚠ Port is IN USE by:"
        echo "$PROCESS"
    else
        echo "✓ Port is free"
    fi
else
    echo "⚠ lsof not installed, cannot check"
fi

# Check dmesg for USB info
echo ""
echo "4. Recent USB messages..."
dmesg | grep -i "ttyACM0\|cdc_acm\|u-blox" | tail -5

# Try to read a few bytes
echo ""
echo "5. Testing port read (2 seconds)..."
timeout 2 cat /dev/ttyACM0 > /tmp/gps_test.bin 2>&1
FILESIZE=$(stat -c%s /tmp/gps_test.bin 2>/dev/null || echo 0)

if [ "$FILESIZE" -gt 0 ]; then
    echo "✓ Received $FILESIZE bytes"
    echo ""
    echo "First 64 bytes in hex:"
    head -c 64 /tmp/gps_test.bin | hexdump -C
else
    echo "✗ No data received"
    echo "Possible causes:"
    echo "  - Port is open by another process"
    echo "  - Receiver is not sending data"
    echo "  - Bad configuration"
fi

# Check for running ROS nodes
echo ""
echo "6. Checking for ROS nodes using serial ports..."
if command -v ros2 &> /dev/null; then
    ros2 node list 2>/dev/null | grep -i "radio\|gps\|rtk" || echo "  No ROS nodes with radio/gps/rtk in name"
else
    echo "  ROS2 not in path"
fi

echo ""
echo "=== Diagnostics Complete ==="