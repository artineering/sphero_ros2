#!/bin/bash
# Diagnostic script to check if everything is set up correctly

echo "=========================================="
echo "  Sphero Web Interface Diagnostics"
echo "=========================================="
echo ""

# Source workspace
source /home/svaghela/ros2_ws_2/install/setup.bash

echo "1. Checking ROS2 packages..."
echo "   Sphero packages installed:"
ros2 pkg list | grep sphero | sed 's/^/     - /'
echo ""

echo "2. Checking Python dependencies..."
python3 << 'EOF'
import sys
deps = {
    'flask': 'Flask',
    'flask_socketio': 'Flask-SocketIO',
    'rclpy': 'ROS2 Python',
    'spherov2.scanner': 'Spherov2',
    'ament_index_python': 'ament_index_python'
}

all_ok = True
for module, name in deps.items():
    try:
        __import__(module)
        print(f"   ✓ {name}")
    except ImportError:
        print(f"   ✗ {name} - MISSING!")
        all_ok = False

if not all_ok:
    print("\n   Install missing dependencies:")
    print("   pip3 install flask flask-socketio python-socketio spherov2")
EOF
echo ""

echo "3. Checking sphero_controller_node executable..."
which ros2 > /dev/null
if [ $? -eq 0 ]; then
    if ros2 pkg executables sphero_package | grep -q sphero_controller_node; then
        echo "   ✓ sphero_controller_node found"
    else
        echo "   ✗ sphero_controller_node NOT found"
        echo "   Available executables:"
        ros2 pkg executables sphero_package | sed 's/^/     /'
    fi
else
    echo "   ✗ ros2 command not found"
fi
echo ""

echo "4. Checking web interface files..."
python3 << 'EOF'
from ament_index_python.packages import get_package_share_directory
import os

try:
    pkg_dir = get_package_share_directory('sphero_web_interface')
    print(f"   Package directory: {pkg_dir}")

    files_to_check = [
        ('templates/index.html', 'HTML template'),
        ('static/css/style.css', 'CSS stylesheet'),
        ('static/js/app.js', 'JavaScript app')
    ]

    for file_path, description in files_to_check:
        full_path = os.path.join(pkg_dir, file_path)
        if os.path.exists(full_path):
            print(f"   ✓ {description}")
        else:
            print(f"   ✗ {description} - MISSING at {full_path}")
except Exception as e:
    print(f"   ✗ Error: {e}")
EOF
echo ""

echo "5. Testing parameter passing to controller node..."
echo "   This will test if the toy_name parameter works correctly"
echo ""
timeout 5 ros2 run sphero_package sphero_controller_node --ros-args -p toy_name:=TEST-SPHERO 2>&1 | head -n 10
echo ""
echo "   (Timeout after 5 seconds - this is normal)"
echo ""

echo "6. Checking Bluetooth..."
if command -v hciconfig &> /dev/null; then
    hciconfig | grep -q "UP RUNNING"
    if [ $? -eq 0 ]; then
        echo "   ✓ Bluetooth is enabled"
    else
        echo "   ⚠ Bluetooth might not be running"
        echo "   Run: sudo hciconfig hci0 up"
    fi
else
    echo "   ⚠ hciconfig not found (might be using different Bluetooth stack)"
fi
echo ""

echo "=========================================="
echo "Diagnostics complete!"
echo ""
echo "To test the controller directly:"
echo "  ./test_controller.sh"
echo ""
echo "To start the web interface:"
echo "  ros2 run sphero_web_interface web_server"
echo "=========================================="
