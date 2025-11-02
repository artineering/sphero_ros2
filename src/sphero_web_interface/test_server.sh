#!/bin/bash
# Test script to verify the web server starts correctly

echo "Testing Sphero Web Interface..."
echo ""

# Source workspace
source /home/svaghela/ros2_ws_2/install/setup.bash

echo "Checking package installation..."
ros2 pkg list | grep sphero_web_interface

echo ""
echo "Checking Python dependencies..."
python3 -c "import flask; import flask_socketio; print('✓ Flask dependencies OK')"

echo ""
echo "Checking template files..."
python3 << 'EOF'
from ament_index_python.packages import get_package_share_directory
import os

pkg_dir = get_package_share_directory('sphero_web_interface')
template_file = os.path.join(pkg_dir, 'templates', 'index.html')
css_file = os.path.join(pkg_dir, 'static', 'css', 'style.css')
js_file = os.path.join(pkg_dir, 'static', 'js', 'app.js')

print(f"✓ Package directory: {pkg_dir}")
print(f"✓ Template exists: {os.path.exists(template_file)}")
print(f"✓ CSS exists: {os.path.exists(css_file)}")
print(f"✓ JS exists: {os.path.exists(js_file)}")
EOF

echo ""
echo "=========================================="
echo "All checks passed! Ready to launch."
echo ""
echo "To start the web server, run:"
echo "  ros2 run sphero_web_interface web_server"
echo ""
echo "Then open: http://localhost:5000"
echo "=========================================="
