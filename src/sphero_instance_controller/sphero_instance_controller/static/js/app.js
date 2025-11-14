// WebSocket connection
let socket;

// XY Plot variables
let xyChart = null;
let plotData = [];
let maxPlotPoints = 100;
let autoScale = true;

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    initializeWebSocket();
    updateLedColorPreview();
    updateMatrixColorPreview();
    initializeXYPlot();
    checkStatus();

    // Poll status every 2 seconds
    setInterval(checkStatus, 2000);
});

// Initialize WebSocket connection
function initializeWebSocket() {
    socket = io();

    socket.on('connect', function() {
        console.log('WebSocket connected');
    });

    socket.on('disconnect', function() {
        console.log('WebSocket disconnected');
    });

    socket.on('state_update', function(data) {
        updateStateDisplay(data);
    });

    socket.on('sensor_update', function(data) {
        updateSensorDisplay(data);
    });

    // Battery updates come from status_update (heartbeat) only
    // Removed battery_update listener to prevent high-frequency updates

    socket.on('status_update', function(data) {
        updateStatusDisplay(data);
    });

    socket.on('error', function(data) {
        handleError(data);
    });

    socket.on('force_disconnect', function(data) {
        handleForceDisconnect(data);
    });

    socket.on('task_status_update', function(data) {
        updateTaskStatusDisplay(data);
    });
}

// Tab switching
function openTab(evt, tabName) {
    // Hide all tab contents
    const tabContents = document.getElementsByClassName('tab-content');
    for (let i = 0; i < tabContents.length; i++) {
        tabContents[i].classList.remove('active');
    }

    // Remove active class from all buttons
    const tabButtons = document.getElementsByClassName('tab-button');
    for (let i = 0; i < tabButtons.length; i++) {
        tabButtons[i].classList.remove('active');
    }

    // Show current tab and mark button as active
    document.getElementById(tabName).classList.add('active');
    evt.currentTarget.classList.add('active');
}

// Connection functions
async function connectSphero() {
    const spheroName = document.getElementById('sphero-name').value.trim();

    if (!spheroName) {
        showMessage('Please enter a Sphero name', 'error');
        return;
    }

    try {
        const response = await fetch('/api/connect', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ sphero_name: spheroName })
        });

        const data = await response.json();

        if (data.success) {
            showMessage(data.message, 'success');
            document.getElementById('connect-btn').disabled = true;
            document.getElementById('disconnect-btn').disabled = false;
            document.getElementById('sphero-name').disabled = true;
            enableTabs();

            // Update status
            document.getElementById('connection-status').textContent = 'Connecting...';
            document.getElementById('connection-status').className = 'status-connected';
        } else {
            showMessage(data.message, 'error');
        }
    } catch (error) {
        showMessage('Failed to connect: ' + error.message, 'error');
    }
}

async function disconnectSphero() {
    try {
        const response = await fetch('/api/disconnect', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });

        const data = await response.json();

        if (data.success) {
            showMessage(data.message, 'success');
            document.getElementById('connect-btn').disabled = false;
            document.getElementById('disconnect-btn').disabled = true;
            document.getElementById('sphero-name').disabled = false;
            disableTabs();

            // Update status
            document.getElementById('connection-status').textContent = 'Disconnected';
            document.getElementById('connection-status').className = 'status-disconnected';
            document.getElementById('battery-status').textContent = 'Battery: --';
        } else {
            showMessage(data.message, 'error');
        }
    } catch (error) {
        showMessage('Failed to disconnect: ' + error.message, 'error');
    }
}

async function checkStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();

        if (data.connected) {
            if (data.ready) {
                document.getElementById('connection-status').textContent = 'Connected';
                document.getElementById('connection-status').className = 'status-connected';
            } else {
                document.getElementById('connection-status').textContent = 'Connecting...';
                document.getElementById('connection-status').className = 'status-connected';
            }

            if (data.latest_battery && data.latest_battery.percentage !== undefined) {
                // Use unified battery display function
                setBatteryDisplay(data.latest_battery.percentage);
            }
        } else {
            document.getElementById('connection-status').textContent = 'Disconnected';
            document.getElementById('connection-status').className = 'status-disconnected';
        }
    } catch (error) {
        console.error('Failed to check status:', error);
    }
}

// UI Helper functions
function showMessage(message, type) {
    const messageDiv = document.getElementById('connection-message');
    messageDiv.textContent = message;
    messageDiv.className = `message ${type}`;
    messageDiv.style.display = 'block';

    setTimeout(() => {
        messageDiv.style.display = 'none';
    }, 5000);
}

function enableTabs() {
    document.getElementById('state-tab').disabled = false;
    document.getElementById('sensors-tab').disabled = false;
    document.getElementById('plot-tab').disabled = false;
    document.getElementById('tasks-tab').disabled = false;
    document.getElementById('matrix-tab').disabled = false;
    document.getElementById('motion-tab').disabled = false;
}

function disableTabs() {
    document.getElementById('state-tab').disabled = true;
    document.getElementById('sensors-tab').disabled = true;
    document.getElementById('plot-tab').disabled = true;
    document.getElementById('tasks-tab').disabled = true;
    document.getElementById('matrix-tab').disabled = true;
    document.getElementById('motion-tab').disabled = true;

    // Switch back to connection tab
    document.querySelector('.tab-button').click();
}

// State display update
function updateStateDisplay(state) {
    // Connection info
    document.getElementById('state-name').textContent = state.toy_name || '--';
    document.getElementById('state-connection').textContent = state.connection_state || '--';

    // Calculate is_healthy from state
    const is_healthy = state.connection_state === 'connected' && state.battery && state.battery.percentage > 10;
    document.getElementById('state-healthy').textContent = is_healthy ? 'Yes' : 'No';

    // Update main connection status when we get state updates (lowercase 'connected')
    if (state.connection_state === 'connected') {
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').className = 'status-connected';
    }

    // Battery info (display in state tab, but NOT in header - that's updated by heartbeat only)
    if (state.battery) {
        const batteryPct = state.battery.percentage !== undefined ? state.battery.percentage : null;
        const batteryVolt = state.battery.voltage !== undefined ? state.battery.voltage : null;

        document.getElementById('state-battery').textContent =
            batteryPct !== null ? (typeof batteryPct === 'number' ? batteryPct.toFixed(0) : batteryPct) : '--';
        document.getElementById('state-voltage').textContent =
            batteryVolt !== null ? (typeof batteryVolt === 'number' ? batteryVolt.toFixed(2) : batteryVolt) : '--';
        document.getElementById('state-bat-health').textContent =
            batteryPct && batteryPct > 20 ? 'Good' : (batteryPct !== null ? 'Low' : '--');

        // DO NOT update battery header here - it's updated only by status_update (heartbeat)
        // This prevents flickering from high-frequency state updates
    }

    // Motion info
    if (state.motion) {
        document.getElementById('state-heading').textContent =
            state.motion.heading !== undefined ? state.motion.heading : '--';
        document.getElementById('state-speed').textContent =
            state.motion.speed !== undefined ? state.motion.speed : '--';
        document.getElementById('state-moving').textContent =
            state.motion.is_moving ? 'Yes' : 'No';
    }

    // LED color (state.led has red, green, blue directly, not nested under color)
    if (state.led) {
        const ledDisplay = document.getElementById('state-led');
        ledDisplay.style.background =
            `rgb(${state.led.red || 0}, ${state.led.green || 0}, ${state.led.blue || 0})`;
    }

    // Update sensor displays from state data
    // Sensors are top-level properties in state, not nested under "sensors"
    updateSensorDisplayFromState(state);

    // Raw data
    document.getElementById('raw-state-data').textContent =
        JSON.stringify(state, null, 2);
}

// Update sensor displays from state sensor data
function updateSensorDisplayFromState(sensors) {
    // Accelerometer
    if (sensors.accelerometer) {
        document.getElementById('sensor-accel-x').textContent =
            sensors.accelerometer.x ? sensors.accelerometer.x.toFixed(2) : '--';
        document.getElementById('sensor-accel-y').textContent =
            sensors.accelerometer.y ? sensors.accelerometer.y.toFixed(2) : '--';
        document.getElementById('sensor-accel-z').textContent =
            sensors.accelerometer.z ? sensors.accelerometer.z.toFixed(2) : '--';
    }

    // Gyroscope
    if (sensors.gyroscope) {
        document.getElementById('sensor-gyro-x').textContent =
            sensors.gyroscope.x ? sensors.gyroscope.x.toFixed(2) : '--';
        document.getElementById('sensor-gyro-y').textContent =
            sensors.gyroscope.y ? sensors.gyroscope.y.toFixed(2) : '--';
        document.getElementById('sensor-gyro-z').textContent =
            sensors.gyroscope.z ? sensors.gyroscope.z.toFixed(2) : '--';
    }

    // Velocity
    if (sensors.velocity) {
        document.getElementById('sensor-vel-x').textContent =
            sensors.velocity.x ? sensors.velocity.x.toFixed(2) : '--';
        document.getElementById('sensor-vel-y').textContent =
            sensors.velocity.y ? sensors.velocity.y.toFixed(2) : '--';
    }

    // Location (stored in 'position' in the state data)
    if (sensors.position) {
        const locX = sensors.position.x;
        const locY = sensors.position.y;

        document.getElementById('sensor-loc-x').textContent =
            locX !== undefined && locX !== null ? locX.toFixed(2) : '--';
        document.getElementById('sensor-loc-y').textContent =
            locY !== undefined && locY !== null ? locY.toFixed(2) : '--';

        // Update plot with location data
        if (locX !== undefined && locX !== null && locY !== undefined && locY !== null) {
            updatePlot(locX, locY);
        }
    }
    // Fallback to 'location' if that's what's being used
    else if (sensors.location) {
        const locX = sensors.location.x;
        const locY = sensors.location.y;

        document.getElementById('sensor-loc-x').textContent =
            locX !== undefined && locX !== null ? locX.toFixed(2) : '--';
        document.getElementById('sensor-loc-y').textContent =
            locY !== undefined && locY !== null ? locY.toFixed(2) : '--';

        // Update plot with location data
        if (locX !== undefined && locX !== null && locY !== undefined && locY !== null) {
            updatePlot(locX, locY);
        }
    }
}

// Sensor display update
function updateSensorDisplay(data) {
    console.log('Sensor update:', data);
}

// Battery display update
// Helper function to get battery background color based on percentage
function getBatteryColor(percentage) {
    // Use 6 discrete color levels based on battery percentage
    if (percentage >= 80) {
        return '#28a745'; // Green - Excellent
    } else if (percentage >= 60) {
        return '#5cb85c'; // Light Green - Good
    } else if (percentage >= 40) {
        return '#ffc107'; // Yellow/Amber - Fair
    } else if (percentage >= 20) {
        return '#fd7e14'; // Orange - Low
    } else if (percentage >= 10) {
        return '#dc3545'; // Red - Critical
    } else {
        return '#a71d2a'; // Dark Red - Emergency
    }
}

// Track last valid battery percentage to prevent flickering
let lastValidBatteryPct = null;

// Unified battery update function
function setBatteryDisplay(percentage) {
    const batteryElement = document.getElementById('battery-status');
    if (batteryElement && percentage !== undefined && percentage !== null) {
        const pct = typeof percentage === 'number' ? percentage : parseFloat(percentage);

        // Only update if we have a valid percentage
        if (!isNaN(pct) && pct >= 0 && pct <= 100) {
            // Skip suspicious 0% readings if we previously had a higher value
            // (unless we've seen consistently low values)
            if (pct === 0 && lastValidBatteryPct !== null && lastValidBatteryPct > 5) {
                console.warn(`Ignoring suspicious 0% battery reading (last valid: ${lastValidBatteryPct}%)`);
                return; // Don't update
            }

            // Update display
            lastValidBatteryPct = pct;
            batteryElement.textContent = `Battery: ${pct.toFixed(0)}%`;
            batteryElement.style.backgroundColor = getBatteryColor(pct);
            batteryElement.style.color = '#ffffff'; // White text for readability
            batteryElement.style.padding = '4px 8px';
            batteryElement.style.borderRadius = '4px';
            batteryElement.style.fontWeight = 'bold';
        }
    }
}

// Status display update
function updateStatusDisplay(data) {
    // Update connection status when we receive heartbeat (lowercase 'connected')
    if (data.connection_state === 'connected') {
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').className = 'status-connected';
    }

    // Update battery from status using unified function
    if (data.battery && data.battery.percentage !== undefined) {
        setBatteryDisplay(data.battery.percentage);
    }
}

// LED Control
function updateLedColorPreview() {
    const r = document.getElementById('led-red').value;
    const g = document.getElementById('led-green').value;
    const b = document.getElementById('led-blue').value;

    document.getElementById('led-red-val').textContent = r;
    document.getElementById('led-green-val').textContent = g;
    document.getElementById('led-blue-val').textContent = b;

    document.getElementById('led-color-preview').style.background =
        `rgb(${r}, ${g}, ${b})`;
}

async function sendLed() {
    const r = parseInt(document.getElementById('led-red').value);
    const g = parseInt(document.getElementById('led-green').value);
    const b = parseInt(document.getElementById('led-blue').value);
    const ledType = document.getElementById('led-type').value;

    try {
        await fetch('/api/led', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ red: r, green: g, blue: b, led: ledType })
        });
    } catch (error) {
        console.error('Failed to send LED command:', error);
    }
}

// Matrix Control
function updateMatrixColorPreview() {
    const r = document.getElementById('matrix-red').value;
    const g = document.getElementById('matrix-green').value;
    const b = document.getElementById('matrix-blue').value;

    document.getElementById('matrix-red-val').textContent = r;
    document.getElementById('matrix-green-val').textContent = g;
    document.getElementById('matrix-blue-val').textContent = b;

    document.getElementById('matrix-color-preview').style.background =
        `rgb(${r}, ${g}, ${b})`;
}

async function sendMatrix(pattern) {
    const r = parseInt(document.getElementById('matrix-red').value);
    const g = parseInt(document.getElementById('matrix-green').value);
    const b = parseInt(document.getElementById('matrix-blue').value);

    try {
        await fetch('/api/matrix', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                pattern: pattern,
                red: r,
                green: g,
                blue: b
            })
        });
    } catch (error) {
        console.error('Failed to send matrix command:', error);
    }
}

async function clearMatrix() {
    try {
        await fetch('/api/matrix/clear', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
    } catch (error) {
        console.error('Failed to clear matrix:', error);
    }
}

// Heading Control
function updateHeading() {
    const heading = document.getElementById('heading').value;
    document.getElementById('heading-val').textContent = heading;

    // Update compass arrow
    const arrow = document.getElementById('compass-arrow');
    arrow.style.transform = `rotate(${heading}deg)`;
}

async function sendHeading() {
    const heading = parseInt(document.getElementById('heading').value);

    try {
        await fetch('/api/motion/heading', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ heading: heading })
        });
    } catch (error) {
        console.error('Failed to send heading command:', error);
    }
}

// Speed Control
function updateSpeedVal() {
    const speed = document.getElementById('speed').value;
    document.getElementById('speed-val').textContent = speed;
}

async function sendSpeed() {
    const speed = parseInt(document.getElementById('speed').value);

    try {
        await fetch('/api/motion/speed', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ speed: speed })
        });
    } catch (error) {
        console.error('Failed to send speed command:', error);
    }
}

async function sendStop() {
    try {
        await fetch('/api/motion/stop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
    } catch (error) {
        console.error('Failed to send stop command:', error);
    }
}

// Reset to Origin
async function resetToOrigin() {
    try {
        const response = await fetch('/api/motion/reset', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        const result = await response.json();
        if (result.success) {
            console.log('Sphero reset to origin: heading=0°, position=(0,0)');
            // Optionally show a notification to the user
            alert('Sphero reset to origin!\nHeading: 0°\nPosition: (0, 0)');
        }
    } catch (error) {
        console.error('Failed to reset to origin:', error);
        alert('Failed to reset Sphero to origin');
    }
}

// Quick Move
async function quickMove(heading) {
    const speed = 100;

    try {
        await fetch('/api/motion/roll', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                heading: heading,
                speed: speed,
                duration: 0
            })
        });
    } catch (error) {
        console.error('Failed to send roll command:', error);
    }
}

// Error Handling
function handleError(data) {
    console.error('Error from server:', data);

    // Show error notification
    const errorDiv = document.createElement('div');
    errorDiv.className = 'error-notification';
    errorDiv.innerHTML = `
        <strong>Error #${data.error_count}:</strong> ${data.message}
        <button onclick="this.parentElement.remove()">×</button>
    `;
    document.body.appendChild(errorDiv);

    // Auto-remove after 5 seconds
    setTimeout(() => {
        if (errorDiv.parentElement) {
            errorDiv.remove();
        }
    }, 5000);

    // Update status display
    document.getElementById('connection-status').textContent = 'Error';
    document.getElementById('connection-status').className = 'status-error';
}

function handleForceDisconnect(data) {
    console.warn('Force disconnect:', data);

    // Show critical error notification
    alert(`Connection Lost!\n\nReason: ${data.reason}\n\n${data.message}`);

    // Force UI to disconnect state
    document.getElementById('connect-btn').disabled = false;
    document.getElementById('disconnect-btn').disabled = true;
    document.getElementById('sphero-name').disabled = false;
    disableTabs();

    // Update status
    document.getElementById('connection-status').textContent = 'Disconnected';
    document.getElementById('connection-status').className = 'status-disconnected';
    document.getElementById('battery-status').textContent = 'Battery: --';

    // Show error in connection tab
    showMessage(`Disconnected: ${data.message}`, 'error');

    // Switch to connection tab
    document.querySelector('.tab-button').click();
}

// XY Plot Functions
function initializeXYPlot() {
    const ctx = document.getElementById('xy-plot').getContext('2d');

    xyChart = new Chart(ctx, {
        type: 'scatter',
        data: {
            datasets: [{
                label: 'Sphero Path',
                data: [],
                backgroundColor: 'rgba(102, 126, 234, 0.6)',
                borderColor: 'rgba(102, 126, 234, 1)',
                borderWidth: 2,
                showLine: true,
                pointRadius: 3,
                pointHoverRadius: 5
            }, {
                label: 'Current Position',
                data: [],
                backgroundColor: 'rgba(239, 68, 68, 1)',
                borderColor: 'rgba(239, 68, 68, 1)',
                borderWidth: 2,
                pointRadius: 8,
                pointHoverRadius: 10,
                showLine: false
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: true,
            aspectRatio: 1,
            plugins: {
                title: {
                    display: true,
                    text: 'Sphero XY Position (cm)',
                    font: {
                        size: 16
                    }
                },
                legend: {
                    display: true,
                    position: 'top'
                },
                zoom: {
                    zoom: {
                        wheel: {
                            enabled: true
                        },
                        pinch: {
                            enabled: true
                        },
                        mode: 'xy'
                    },
                    pan: {
                        enabled: true,
                        mode: 'xy'
                    }
                }
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'X Position (cm)'
                    },
                    grid: {
                        color: 'rgba(0, 0, 0, 0.1)'
                    }
                },
                y: {
                    type: 'linear',
                    title: {
                        display: true,
                        text: 'Y Position (cm)'
                    },
                    grid: {
                        color: 'rgba(0, 0, 0, 0.1)'
                    }
                }
            }
        }
    });
}

function updatePlot(x, y) {
    if (!xyChart) return;

    // Add new point to the dataset
    plotData.push({ x: x, y: y });

    // Limit the number of points
    if (plotData.length > maxPlotPoints) {
        plotData.shift();
    }

    // Update chart data
    xyChart.data.datasets[0].data = plotData;
    xyChart.data.datasets[1].data = [{ x: x, y: y }]; // Current position marker

    // Update the chart
    xyChart.update('none'); // 'none' for no animation, faster updates

    // Update info display
    document.getElementById('plot-current-x').textContent = x.toFixed(2);
    document.getElementById('plot-current-y').textContent = y.toFixed(2);
    document.getElementById('plot-point-count').textContent = plotData.length;

    // Auto-scale if enabled
    if (autoScale) {
        updateChartScale();
    }
}

function updateChartScale() {
    if (!xyChart || plotData.length === 0) return;

    const xValues = plotData.map(p => p.x);
    const yValues = plotData.map(p => p.y);

    const xMin = Math.min(...xValues);
    const xMax = Math.max(...xValues);
    const yMin = Math.min(...yValues);
    const yMax = Math.max(...yValues);

    const xPadding = (xMax - xMin) * 0.1 || 10;
    const yPadding = (yMax - yMin) * 0.1 || 10;

    xyChart.options.scales.x.min = xMin - xPadding;
    xyChart.options.scales.x.max = xMax + xPadding;
    xyChart.options.scales.y.min = yMin - yPadding;
    xyChart.options.scales.y.max = yMax + yPadding;
}

function clearPlot() {
    plotData = [];
    if (xyChart) {
        xyChart.data.datasets[0].data = [];
        xyChart.data.datasets[1].data = [];
        xyChart.update();
    }
    document.getElementById('plot-current-x').textContent = '--';
    document.getElementById('plot-current-y').textContent = '--';
    document.getElementById('plot-point-count').textContent = '0';
}

function resetZoom() {
    if (xyChart) {
        xyChart.options.scales.x.min = undefined;
        xyChart.options.scales.x.max = undefined;
        xyChart.options.scales.y.min = undefined;
        xyChart.options.scales.y.max = undefined;
        xyChart.update();
        if (autoScale) {
            updateChartScale();
            xyChart.update();
        }
    }
}

function toggleAutoScale() {
    autoScale = document.getElementById('auto-scale').checked;
    if (autoScale) {
        updateChartScale();
        if (xyChart) {
            xyChart.update();
        }
    }
}

function updateMaxPoints() {
    const newMax = parseInt(document.getElementById('max-points').value);
    if (newMax > 0) {
        maxPlotPoints = newMax;
        // Trim existing data if necessary
        while (plotData.length > maxPlotPoints) {
            plotData.shift();
        }
        if (xyChart) {
            xyChart.data.datasets[0].data = plotData;
            xyChart.update();
        }
    }
}

// Task Controller Functions
let currentTaskType = null;
let currentTaskParams = {};

async function startTaskController() {
    try {
        const response = await fetch('/api/task_controller/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        const data = await response.json();

        if (data.success) {
            document.getElementById('start-task-controller-btn').disabled = true;
            document.getElementById('stop-task-controller-btn').disabled = false;
            showTaskMessage(data.message, 'success');
        } else {
            showTaskMessage(data.message, 'error');
        }
    } catch (error) {
        showTaskMessage('Failed to start task controller: ' + error.message, 'error');
    }
}

async function stopTaskController() {
    try {
        const response = await fetch('/api/task_controller/stop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        const data = await response.json();

        if (data.success) {
            document.getElementById('start-task-controller-btn').disabled = false;
            document.getElementById('stop-task-controller-btn').disabled = true;
            showTaskMessage(data.message, 'success');
        } else {
            showTaskMessage(data.message, 'error');
        }
    } catch (error) {
        showTaskMessage('Failed to stop task controller: ' + error.message, 'error');
    }
}

function showTaskMessage(message, type) {
    const messageDiv = document.getElementById('task-controller-status');
    messageDiv.textContent = message;
    messageDiv.className = `message ${type}`;
    messageDiv.style.display = 'block';

    setTimeout(() => {
        messageDiv.style.display = 'none';
    }, 5000);
}

function showTaskForm(taskType) {
    currentTaskType = taskType;
    currentTaskParams = {};

    document.getElementById('task-form-title').textContent = taskType.replace('_', ' ').toUpperCase();
    const formFields = document.getElementById('task-form-fields');
    formFields.innerHTML = '';

    // Generate form fields based on task type
    if (taskType === 'move_to') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Target X (cm):</label>
                <input type="number" id="task-x" value="100" step="10">
            </div>
            <div class="form-group">
                <label>Target Y (cm):</label>
                <input type="number" id="task-y" value="100" step="10">
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="100" min="0" max="255">
            </div>
        `;
    } else if (taskType === 'patrol') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Waypoints (JSON array of {x, y} objects):</label>
                <textarea id="task-waypoints" rows="4">[{"x": 50, "y": 50}, {"x": 100, "y": 50}, {"x": 100, "y": 100}]</textarea>
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="100" min="0" max="255">
            </div>
            <div class="form-group">
                <label><input type="checkbox" id="task-loop"> Loop</label>
            </div>
        `;
    } else if (taskType === 'circle') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Radius (cm):</label>
                <input type="number" id="task-radius" value="50" step="10">
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="100" min="0" max="255">
            </div>
            <div class="form-group">
                <label>Duration (seconds):</label>
                <input type="number" id="task-duration" value="10" step="1">
            </div>
        `;
    } else if (taskType === 'square') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Side Length (cm):</label>
                <input type="number" id="task-side-length" value="100" step="10">
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="100" min="0" max="255">
            </div>
        `;
    } else if (taskType === 'spin') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Rotations:</label>
                <input type="number" id="task-rotations" value="2" step="0.5">
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="100" min="0" max="255">
            </div>
        `;
    } else if (taskType === 'jumping_bean') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Duration (seconds):</label>
                <input type="number" id="task-duration" value="10" step="1" min="1" max="60">
            </div>
            <div class="form-group">
                <label>Speed (0-255):</label>
                <input type="number" id="task-speed" value="200" min="0" max="255">
            </div>
            <div class="form-group">
                <label>Flip Interval (seconds):</label>
                <input type="number" id="task-flip-interval" value="0.1" step="0.05" min="0.05" max="1.0">
            </div>
            <small>Rapid back-and-forth flipping motion like a jumping bean!</small>
        `;
    } else if (taskType === 'collision_start') {
        formFields.innerHTML = `
            <div class="form-group">
                <label>Detection Mode:</label>
                <select id="task-collision-mode">
                    <option value="obstacle" selected>Obstacle (while moving)</option>
                    <option value="tap">Tap (while stationary)</option>
                </select>
            </div>
            <div class="form-group">
                <label>Sensitivity:</label>
                <select id="task-collision-sensitivity">
                    <option value="SUPER_HIGH">Super High (most sensitive)</option>
                    <option value="VERY_HIGH">Very High</option>
                    <option value="HIGH" selected>High (default)</option>
                    <option value="MEDIUM">Medium</option>
                    <option value="LOW">Low</option>
                    <option value="VERY_LOW">Very Low (least sensitive)</option>
                </select>
                <small>Higher sensitivity detects lighter touches</small>
            </div>
        `;
    } else if (taskType === 'collision_stop') {
        formFields.innerHTML = `
            <p>This will stop collision detection.</p>
        `;
    }

    document.getElementById('task-form-container').style.display = 'block';
}

function cancelTaskForm() {
    document.getElementById('task-form-container').style.display = 'none';
    currentTaskType = null;
    currentTaskParams = {};
}

async function submitCurrentTask() {
    if (!currentTaskType) return;

    const parameters = {};

    // Collect parameters based on task type
    if (currentTaskType === 'move_to') {
        parameters.x = parseFloat(document.getElementById('task-x').value);
        parameters.y = parseFloat(document.getElementById('task-y').value);
        parameters.speed = parseInt(document.getElementById('task-speed').value);
    } else if (currentTaskType === 'patrol') {
        try {
            parameters.waypoints = JSON.parse(document.getElementById('task-waypoints').value);
            parameters.speed = parseInt(document.getElementById('task-speed').value);
            parameters.loop = document.getElementById('task-loop').checked;
        } catch (e) {
            showTaskMessage('Invalid waypoints JSON: ' + e.message, 'error');
            return;
        }
    } else if (currentTaskType === 'circle') {
        parameters.radius = parseFloat(document.getElementById('task-radius').value);
        parameters.speed = parseInt(document.getElementById('task-speed').value);
        parameters.duration = parseFloat(document.getElementById('task-duration').value);
    } else if (currentTaskType === 'square') {
        parameters.side_length = parseFloat(document.getElementById('task-side-length').value);
        parameters.speed = parseInt(document.getElementById('task-speed').value);
    } else if (currentTaskType === 'spin') {
        parameters.rotations = parseFloat(document.getElementById('task-rotations').value);
        parameters.speed = parseInt(document.getElementById('task-speed').value);
    } else if (currentTaskType === 'jumping_bean') {
        parameters.duration = parseFloat(document.getElementById('task-duration').value);
        parameters.speed = parseInt(document.getElementById('task-speed').value);
        parameters.flip_interval = parseFloat(document.getElementById('task-flip-interval').value);
    } else if (currentTaskType === 'collision_start') {
        parameters.action = 'start';
        parameters.mode = document.getElementById('task-collision-mode').value;

        // Get sensitivity - with debug logging
        const sensitivityElement = document.getElementById('task-collision-sensitivity');
        const sensitivityValue = sensitivityElement ? sensitivityElement.value : null;
        console.log('[DEBUG] Sensitivity element:', sensitivityElement);
        console.log('[DEBUG] Sensitivity value:', sensitivityValue);
        parameters.sensitivity = sensitivityValue;
    } else if (currentTaskType === 'collision_stop') {
        parameters.action = 'stop';
    }

    // Map collision task types to 'collision' for the backend
    let taskType = currentTaskType;
    if (currentTaskType === 'collision_start' || currentTaskType === 'collision_stop') {
        taskType = 'collision';
    }

    const taskData = {
        task_type: taskType,
        parameters: parameters
    };

    try {
        const response = await fetch('/api/task', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(taskData)
        });
        const data = await response.json();

        if (data.success) {
            showTaskMessage(data.message, 'success');
            cancelTaskForm();
        } else {
            showTaskMessage('Failed to submit task', 'error');
        }
    } catch (error) {
        showTaskMessage('Failed to submit task: ' + error.message, 'error');
    }
}

async function submitStopTask() {
    const taskData = {
        task_type: 'stop',
        parameters: {}
    };

    try {
        const response = await fetch('/api/task', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(taskData)
        });
        const data = await response.json();

        if (data.success) {
            showTaskMessage('Stop task submitted', 'success');
        }
    } catch (error) {
        showTaskMessage('Failed to submit stop task: ' + error.message, 'error');
    }
}

function updateTaskStatusDisplay(taskStatus) {
    document.getElementById('current-task-id').textContent = taskStatus.task_id || '--';
    document.getElementById('current-task-type').textContent = taskStatus.task_type || '--';
    document.getElementById('current-task-status').textContent = taskStatus.status || '--';

    if (taskStatus.started_at && taskStatus.status === 'running') {
        const duration = Date.now() / 1000 - taskStatus.started_at;
        document.getElementById('current-task-duration').textContent = duration.toFixed(1);
    } else if (taskStatus.started_at && taskStatus.completed_at) {
        const duration = taskStatus.completed_at - taskStatus.started_at;
        document.getElementById('current-task-duration').textContent = duration.toFixed(1);
    } else {
        document.getElementById('current-task-duration').textContent = '--';
    }

    document.getElementById('task-status-data').textContent = JSON.stringify(taskStatus, null, 2);
}
