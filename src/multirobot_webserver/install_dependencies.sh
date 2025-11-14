#!/bin/bash
# Multi-Robot Webserver - Dependency Installation Script

echo "=============================================="
echo "Installing Multi-Robot Webserver Dependencies"
echo "=============================================="
echo

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "⚠️  Please do not run this script as root"
    exit 1
fi

echo "📦 Installing Python packages..."
echo

# List of required packages
PACKAGES=(
    "flask"
    "flask-socketio"
    "flask-cors"
    "eventlet"
    "python-socketio[client]"
)

# Install each package
for package in "${PACKAGES[@]}"; do
    echo "Installing $package..."
    pip install "$package" --break-system-packages
    if [ $? -eq 0 ]; then
        echo "✅ $package installed successfully"
    else
        echo "❌ Failed to install $package"
        exit 1
    fi
    echo
done

echo "=============================================="
echo "✅ All dependencies installed successfully!"
echo "=============================================="
echo
echo "Next steps:"
echo "1. Build the packages:"
echo "   cd ~/ros2_ws_2"
echo "   colcon build --packages-select sphero_instance_controller multirobot_webserver"
echo "   source install/setup.bash"
echo
echo "2. Start the multi-robot web server:"
echo "   ros2 run multirobot_webserver multirobot_webapp"
echo
echo "3. Open in browser:"
echo "   http://localhost:5000"
echo
