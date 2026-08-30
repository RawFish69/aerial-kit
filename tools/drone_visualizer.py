#!/usr/bin/env python3
"""
Universal Drone 3D Visualizer - Real-time 3D visualization in your browser
Supports: CRSF, SBUS, iBus protocols with dynamic switching via web UI
Usage: python drone_visualizer.py COM3
Then open: http://localhost:5000
"""

import sys
import time
import json
import threading
import serial
from flask import Flask, render_template_string, jsonify, request
from protocols import get_protocol, list_protocols, CRSFDecoder

# Global state
drone_state = {
    'channels': [992] * 16,
    'roll': 0,
    'pitch': 0,
    'yaw': 0,
    'throttle': 0,
    'frame_count': 0,
    'error_count': 0,
    'rate': 0,
    'connected': False,
    'flight_mode': 'angle',
    'protocol': 'CRSF'
}

serial_port = None
serial_lock = threading.Lock()
current_decoder = None
port_name_global = None
should_reconnect = False

def channel_to_angle(value, max_angle=45, ch_min=172, ch_max=1811, ch_mid=992):
    """Convert channel value to angle in degrees"""
    normalized = (value - ch_mid) / (ch_max - ch_min)
    return normalized * max_angle * 2

def channel_to_normalized(value, ch_min=172, ch_max=1811):
    """Convert channel value to 0-1 range"""
    return max(0, min(1, (value - ch_min) / (ch_max - ch_min)))

def serial_reader_thread(port_name):
    """Background thread to read protocol data from serial port"""
    global serial_port, drone_state, current_decoder, should_reconnect
    
    protocol_name = drone_state['protocol']
    
    # Get protocol decoder
    protocol_class = get_protocol(protocol_name)
    if not protocol_class:
        print(f"✗ Unknown protocol: {protocol_name}")
        return
    
    current_decoder = protocol_class()
    config = current_decoder.get_config()
    
    try:
        with serial_lock:
            if serial_port:
                serial_port.close()
            serial_port = serial.Serial(port_name, config['baudrate'], timeout=0.1)
        drone_state['connected'] = True
        print(f"✓ Connected to {port_name} ({config['name']} @ {config['baudrate']} baud)")
        
        last_time = time.time()
        
        while True:
            # Check if protocol switch requested
            if should_reconnect:
                with serial_lock:
                    if serial_port:
                        serial_port.close()
                    should_reconnect = False
                print(f"↻ Switching to {drone_state['protocol']}...")
                # Restart with new protocol
                return serial_reader_thread(port_name)
            
            try:
                channels = current_decoder.decode(serial_port)
                
                if channels:
                    with serial_lock:
                        drone_state['channels'] = channels
                        
                        # Handle flight mode
                        if drone_state['flight_mode'] == 'angle':
                            # Angle mode: Direct angle control with limits
                            drone_state['roll'] = channel_to_angle(
                                channels[0], max_angle=45, 
                                ch_min=config['channel_min'],
                                ch_max=config['channel_max'],
                                ch_mid=config['channel_mid']
                            )
                            drone_state['pitch'] = channel_to_angle(
                                channels[1], max_angle=45,
                                ch_min=config['channel_min'],
                                ch_max=config['channel_max'],
                                ch_mid=config['channel_mid']
                            )
                        else:  # acro mode
                            # Acro mode: Rate control, accumulate rotation
                            roll_rate = channel_to_angle(
                                channels[0], max_angle=180,
                                ch_min=config['channel_min'],
                                ch_max=config['channel_max'],
                                ch_mid=config['channel_mid']
                            ) * 0.02
                            pitch_rate = channel_to_angle(
                                channels[1], max_angle=180,
                                ch_min=config['channel_min'],
                                ch_max=config['channel_max'],
                                ch_mid=config['channel_mid']
                            ) * 0.02
                            drone_state['roll'] += roll_rate
                            drone_state['pitch'] += pitch_rate
                        
                        # Yaw always accumulates (same in both modes)
                        drone_state['yaw'] += channel_to_angle(
                            channels[3], max_angle=180,
                            ch_min=config['channel_min'],
                            ch_max=config['channel_max'],
                            ch_mid=config['channel_mid']
                        ) * 0.02
                        
                        drone_state['throttle'] = channel_to_normalized(
                            channels[2],
                            ch_min=config['channel_min'],
                            ch_max=config['channel_max']
                        )
                        drone_state['frame_count'] += 1
                        
                        # Calculate rate
                        now = time.time()
                        if now - last_time > 0:
                            drone_state['rate'] = 1.0 / (now - last_time)
                        last_time = now
                else:
                    drone_state['error_count'] += 1
            
            except Exception as e:
                print(f"Error reading serial: {e}")
                time.sleep(0.1)
    
    except Exception as e:
        print(f"✗ Failed to connect to {port_name}: {e}")
        drone_state['connected'] = False

# Flask app
app = Flask(__name__)

@app.route('/')
def index():
    """Serve the main page with embedded HTML/CSS/JS"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/drone')
def get_drone_state():
    """API endpoint to get current drone state"""
    with serial_lock:
        return jsonify(drone_state)

@app.route('/api/set_mode/<mode>')
def set_flight_mode(mode):
    """API endpoint to set flight mode"""
    global drone_state
    if mode in ['angle', 'acro']:
        with serial_lock:
            drone_state['flight_mode'] = mode
            # Reset orientation when switching modes
            if mode == 'angle':
                drone_state['roll'] = 0
                drone_state['pitch'] = 0
            drone_state['yaw'] = 0
        return jsonify({'status': 'ok', 'mode': mode})
    return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400

@app.route('/api/set_protocol/<protocol>')
def set_protocol(protocol):
    """API endpoint to set protocol"""
    global drone_state, current_decoder, should_reconnect
    
    if protocol in list_protocols():
        with serial_lock:
            drone_state['protocol'] = protocol
            drone_state['connected'] = False
            # Reset drone state for clean reconnection
            drone_state['roll'] = 0
            drone_state['pitch'] = 0
            drone_state['yaw'] = 0
            drone_state['throttle'] = 0
            drone_state['channels'] = [992] * 16
            drone_state['frame_count'] = 0
            drone_state['error_count'] = 0
            should_reconnect = True
        return jsonify({'status': 'ok', 'protocol': protocol})
    return jsonify({'status': 'error', 'message': 'Invalid protocol'}), 400

@app.route('/api/protocols')
def get_protocols():
    """API endpoint to get available protocols"""
    return jsonify({'protocols': list_protocols()})

# Embedded HTML template with Three.js 3D visualization
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Universal Drone Visualizer</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #fff;
            overflow: hidden;
        }
        #container {
            display: grid;
            grid-template-columns: 300px 1fr;
            height: 100vh;
        }
        #sidebar {
            background: rgba(0, 0, 0, 0.5);
            padding: 20px;
            overflow-y: auto;
            backdrop-filter: blur(10px);
        }
        #canvas-container {
            position: relative;
        }
        h1 {
            font-size: 24px;
            margin-bottom: 20px;
            text-align: center;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
            animation: pulse 2s infinite;
        }
        .status.connected { background: #00ff00; }
        .status.disconnected { background: #ff0000; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        .section {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 15px;
        }
        .section h2 {
            font-size: 16px;
            margin-bottom: 10px;
            color: #ffd700;
        }
        .data-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 14px;
        }
        .label { opacity: 0.8; }
        .value { 
            font-weight: bold;
            font-family: 'Courier New', monospace;
        }
        .bar-container {
            background: rgba(0, 0, 0, 0.3);
            height: 20px;
            border-radius: 4px;
            overflow: hidden;
            margin-top: 5px;
        }
        .bar {
            height: 100%;
            transition: width 0.1s;
            border-radius: 4px;
        }
        .bar.roll { background: linear-gradient(90deg, #ff6b6b, #ee5a6f); }
        .bar.pitch { background: linear-gradient(90deg, #4ecdc4, #44a8b3); }
        .bar.throttle { background: linear-gradient(90deg, #45b7d1, #3aa1c1); }
        .bar.yaw { background: linear-gradient(90deg, #ffa07a, #ff8a65); }
        .mode-btn {
            flex: 1;
            padding: 10px;
            background: rgba(255, 255, 255, 0.1);
            border: 2px solid rgba(255, 255, 255, 0.3);
            color: #fff;
            border-radius: 6px;
            cursor: pointer;
            font-weight: bold;
            font-size: 14px;
            transition: all 0.3s;
        }
        .mode-btn:hover {
            background: rgba(255, 255, 255, 0.2);
            border-color: rgba(255, 255, 255, 0.5);
        }
        .mode-btn.active {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-color: #ffd700;
            box-shadow: 0 0 10px rgba(255, 215, 0, 0.5);
        }
        .protocol-badge {
            display: inline-block;
            background: linear-gradient(135deg, #ff6b6b 0%, #ee5a6f 100%);
            padding: 4px 12px;
            border-radius: 12px;
            font-size: 12px;
            font-weight: bold;
            letter-spacing: 1px;
        }
        canvas { display: block; }
    </style>
</head>
<body>
    <div id="container">
        <div id="sidebar">
            <h1>Drone Visualizer</h1>
            
            <div class="section">
                <h2>Protocol</h2>
                <div style="display: flex; gap: 5px; margin-bottom: 10px;">
                    <button class="mode-btn active" id="protocol-CRSF" onclick="setProtocol('CRSF')" style="flex: 1; font-size: 12px;">CRSF</button>
                    <button class="mode-btn" id="protocol-SBUS" onclick="setProtocol('SBUS')" style="flex: 1; font-size: 12px;">SBUS</button>
                    <button class="mode-btn" id="protocol-iBus" onclick="setProtocol('iBus')" style="flex: 1; font-size: 12px;">iBus</button>
                </div>
                <div style="font-size: 11px; opacity: 0.7;">
                    Select protocol to match your receiver
                </div>
            </div>
            
            <div class="section">
                <h2>Connection</h2>
                <div class="data-row">
                    <span class="label">Status:</span>
                    <span class="value">
                        <span class="status" id="status"></span>
                        <span id="status-text">Connecting...</span>
                    </span>
                </div>
                <div class="data-row">
                    <span class="label">Protocol:</span>
                    <span class="value">
                        <span class="protocol-badge" id="protocol">CRSF</span>
                    </span>
                </div>
                <div class="data-row">
                    <span class="label">Frame Rate:</span>
                    <span class="value"><span id="rate">0</span> Hz</span>
                </div>
                <div class="data-row">
                    <span class="label">Frames:</span>
                    <span class="value" id="frames">0</span>
                </div>
                <div class="data-row">
                    <span class="label">Errors:</span>
                    <span class="value" id="errors">0</span>
                </div>
            </div>
            
            <div class="section">
                <h2>Flight Mode</h2>
                <div style="display: flex; gap: 10px; margin-bottom: 10px;">
                    <button id="angle-btn" class="mode-btn active" onclick="setFlightMode('angle')">
                        ANGLE
                    </button>
                    <button id="acro-btn" class="mode-btn" onclick="setFlightMode('acro')">
                        ACRO
                    </button>
                </div>
                <div style="font-size: 12px; opacity: 0.7; line-height: 1.4;">
                    <span id="mode-description">
                        Self-leveling, max ±45°
                    </span>
                </div>
            </div>
            
            <div class="section">
                <h2>Orientation</h2>
                <div class="data-row">
                    <span class="label">Roll:</span>
                    <span class="value"><span id="roll">0.0</span>°</span>
                </div>
                <div class="bar-container">
                    <div class="bar roll" id="roll-bar" style="width: 50%"></div>
                </div>
                
                <div class="data-row" style="margin-top: 10px">
                    <span class="label">Pitch:</span>
                    <span class="value"><span id="pitch">0.0</span>°</span>
                </div>
                <div class="bar-container">
                    <div class="bar pitch" id="pitch-bar" style="width: 50%"></div>
                </div>
                
                <div class="data-row" style="margin-top: 10px">
                    <span class="label">Yaw:</span>
                    <span class="value"><span id="yaw">0.0</span>°</span>
                </div>
            </div>
            
            <div class="section">
                <h2>Throttle</h2>
                <div class="data-row">
                    <span class="label">Throttle:</span>
                    <span class="value"><span id="throttle">0</span>%</span>
                </div>
                <div class="bar-container">
                    <div class="bar throttle" id="throttle-bar" style="width: 0%"></div>
                </div>
            </div>
            
            <div class="section">
                <h2>RC Channels</h2>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 5px; font-size: 12px;">
                    <div class="data-row" style="margin: 0;"><span class="label">Ch1:</span><span class="value" id="ch1">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch2:</span><span class="value" id="ch2">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch3:</span><span class="value" id="ch3">172</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch4:</span><span class="value" id="ch4">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch5:</span><span class="value" id="ch5">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch6:</span><span class="value" id="ch6">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch7:</span><span class="value" id="ch7">992</span></div>
                    <div class="data-row" style="margin: 0;"><span class="label">Ch8:</span><span class="value" id="ch8">992</span></div>
                </div>
            </div>
        </div>
        
        <div id="canvas-container"></div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script>
        // Three.js setup
        const container = document.getElementById('canvas-container');
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);
        scene.fog = new THREE.Fog(0x1a1a2e, 10, 50);
        
        const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
        camera.position.set(4, 4, 4);
        camera.lookAt(0, 0, 0);
        
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.shadowMap.enabled = true;
        container.appendChild(renderer.domElement);
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 5);
        directionalLight.castShadow = true;
        scene.add(directionalLight);
        
        // Ground plane
        const groundGeometry = new THREE.PlaneGeometry(20, 20);
        const groundMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x2d5016,
            side: THREE.DoubleSide 
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        scene.add(ground);
        
        // Grid
        const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
        scene.add(gridHelper);
        
        // Drone (quadcopter X configuration)
        const drone = new THREE.Group();
        
        // Body (center)
        const bodyGeometry = new THREE.BoxGeometry(0.6, 0.2, 0.6);
        const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.castShadow = true;
        drone.add(body);
        
        // Arms and motors - X configuration
        const armMaterial = new THREE.MeshPhongMaterial({ color: 0x222222 });
        const motorColors = [0xff0000, 0xff0000, 0x0066ff, 0x0066ff];
        const armLength = 0.85;
        const positions = [
            [armLength, 0, armLength],   // Front-Right (RED)
            [-armLength, 0, armLength],  // Front-Left (RED)
            [armLength, 0, -armLength],  // Back-Right (BLUE)
            [-armLength, 0, -armLength]  // Back-Left (BLUE)
        ];
        
        positions.forEach((pos, i) => {
            // Arm
            const armGeometry = new THREE.CylinderGeometry(0.05, 0.05, 1.2, 8);
            const arm = new THREE.Mesh(armGeometry, armMaterial);
            arm.rotation.z = Math.PI / 2;
            arm.rotation.y = Math.atan2(pos[0], pos[2]);
            arm.position.set(pos[0] * 0.5, 0, pos[2] * 0.5);
            arm.castShadow = true;
            drone.add(arm);
            
            // Motor
            const motorGeometry = new THREE.CylinderGeometry(0.2, 0.2, 0.15, 16);
            const motorMaterial = new THREE.MeshPhongMaterial({ color: motorColors[i] });
            const motor = new THREE.Mesh(motorGeometry, motorMaterial);
            motor.position.set(pos[0], 0, pos[2]);
            motor.castShadow = true;
            drone.add(motor);
            
            // Propeller
            const propGeometry = new THREE.BoxGeometry(0.8, 0.02, 0.05);
            const propMaterial = new THREE.MeshPhongMaterial({ 
                color: 0xcccccc,
                transparent: true,
                opacity: 0.7 
            });
            const prop = new THREE.Mesh(propGeometry, propMaterial);
            prop.position.set(pos[0], 0.1, pos[2]);
            drone.add(prop);
            
            // LED
            const ledGeometry = new THREE.SphereGeometry(0.08, 8, 8);
            const ledColor = i < 2 ? 0xff0000 : 0x00ff00;
            const ledMaterial = new THREE.MeshBasicMaterial({ 
                color: ledColor,
                transparent: true,
                opacity: 0.8
            });
            const led = new THREE.Mesh(ledGeometry, ledMaterial);
            led.position.set(pos[0] * 0.7, 0, pos[2] * 0.7);
            drone.add(led);
        });
        
        // Front indicator (red cone)
        const noseGeometry = new THREE.ConeGeometry(0.2, 0.5, 8);
        const noseMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000, emissive: 0x440000 });
        const nose = new THREE.Mesh(noseGeometry, noseMaterial);
        nose.rotation.x = -Math.PI / 2;
        nose.position.set(0, 0, 1.3);
        nose.castShadow = true;
        drone.add(nose);
        
        drone.position.y = 2;
        scene.add(drone);
        
        // Mouse controls
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };
        let cameraAngle = { theta: Math.PI / 4, phi: Math.PI / 4 };
        let cameraDistance = 7;
        
        container.addEventListener('mousedown', (e) => {
            isDragging = true;
            previousMousePosition = { x: e.clientX, y: e.clientY };
        });
        
        container.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaX = e.clientX - previousMousePosition.x;
                const deltaY = e.clientY - previousMousePosition.y;
                
                cameraAngle.theta += deltaX * 0.01;
                cameraAngle.phi -= deltaY * 0.01;
                cameraAngle.phi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraAngle.phi));
                
                previousMousePosition = { x: e.clientX, y: e.clientY };
            }
        });
        
        container.addEventListener('mouseup', () => { isDragging = false; });
        container.addEventListener('mouseleave', () => { isDragging = false; });
        
        container.addEventListener('wheel', (e) => {
            e.preventDefault();
            cameraDistance += e.deltaY * 0.01;
            cameraDistance = Math.max(2, Math.min(15, cameraDistance));
        });
        
        window.addEventListener('resize', () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        });
        
        // Protocol control
        function setProtocol(protocol) {
            fetch('/api/set_protocol/' + protocol)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        // Update UI
                        ['CRSF', 'SBUS', 'iBus'].forEach(p => {
                            document.getElementById('protocol-' + p).classList.remove('active');
                        });
                        document.getElementById('protocol-' + protocol).classList.add('active');
                        
                        // Reset drone orientation for clean reconnection
                        droneState.roll = 0;
                        droneState.pitch = 0;
                        droneState.yaw = 0;
                        droneState.throttle = 0;
                        droneState.frame_count = 0;
                        droneState.error_count = 0;
                        
                        // Show reconnecting status
                        document.getElementById('status-text').textContent = 'Reconnecting...';
                    }
                })
                .catch(err => console.error('Error setting protocol:', err));
        }
        
        // Flight mode control
        function setFlightMode(mode) {
            fetch('/api/set_mode/' + mode)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        document.getElementById('angle-btn').classList.remove('active');
                        document.getElementById('acro-btn').classList.remove('active');
                        document.getElementById(mode + '-btn').classList.add('active');
                        
                        const desc = mode === 'angle' 
                            ? 'Self-leveling, max ±45°'
                            : 'Rate mode, full 360° rotation';
                        document.getElementById('mode-description').textContent = desc;
                    }
                })
                .catch(err => console.error('Error setting mode:', err));
        }
        
        // Update drone state
        let droneState = {
            roll: 0, pitch: 0, yaw: 0, throttle: 0,
            channels: [992, 992, 172, 992],
            rate: 0, frame_count: 0, error_count: 0,
            connected: false, flight_mode: 'angle', protocol: 'CRSF'
        };
        
        let lastProtocol = 'CRSF';
        let reconnectCount = 0;
        
        function updateDroneState() {
            fetch('/api/drone')
                .then(response => response.json())
                .then(data => {
                    // Check if protocol changed (reconnection happened)
                    if (data.protocol !== lastProtocol) {
                        lastProtocol = data.protocol;
                        reconnectCount++;
                        // Force state update after reconnection
                        if (data.connected) {
                            console.log('Protocol switched to ' + data.protocol);
                        }
                    }
                    
                    droneState = data;
                    
                    // Update UI
                    document.getElementById('status').className = 'status ' + (data.connected ? 'connected' : 'disconnected');
                    document.getElementById('status-text').textContent = data.connected ? 'Connected' : 'Disconnected';
                    document.getElementById('protocol').textContent = data.protocol || 'CRSF';
                    
                    // Update protocol buttons
                    ['CRSF', 'SBUS', 'iBus'].forEach(p => {
                        const btn = document.getElementById('protocol-' + p);
                        if (btn) {
                            if (p === data.protocol) {
                                btn.classList.add('active');
                            } else {
                                btn.classList.remove('active');
                            }
                        }
                    });
                    document.getElementById('rate').textContent = data.rate.toFixed(1);
                    document.getElementById('frames').textContent = data.frame_count;
                    document.getElementById('errors').textContent = data.error_count;
                    
                    document.getElementById('roll').textContent = data.roll.toFixed(1);
                    document.getElementById('pitch').textContent = data.pitch.toFixed(1);
                    document.getElementById('yaw').textContent = data.yaw.toFixed(1);
                    document.getElementById('throttle').textContent = (data.throttle * 100).toFixed(0);
                    
                    // Update channels
                    for (let i = 0; i < 8; i++) {
                        document.getElementById('ch' + (i + 1)).textContent = data.channels[i];
                    }
                    
                    // Update bars
                    const rollPct = ((data.channels[0] - 172) / (1811 - 172)) * 100;
                    const pitchPct = ((data.channels[1] - 172) / (1811 - 172)) * 100;
                    const throttlePct = data.throttle * 100;
                    
                    document.getElementById('roll-bar').style.width = rollPct + '%';
                    document.getElementById('pitch-bar').style.width = pitchPct + '%';
                    document.getElementById('throttle-bar').style.width = throttlePct + '%';
                })
                .catch(err => console.error('Error:', err));
        }
        
        setInterval(updateDroneState, 50); // 20 Hz
        
        // Animation loop
        function animate() {
            requestAnimationFrame(animate);
            
            // Update camera
            camera.position.x = cameraDistance * Math.sin(cameraAngle.phi) * Math.cos(cameraAngle.theta);
            camera.position.y = cameraDistance * Math.cos(cameraAngle.phi);
            camera.position.z = cameraDistance * Math.sin(cameraAngle.phi) * Math.sin(cameraAngle.theta);
            camera.lookAt(0, 2, 0);
            
            // Update drone orientation
            drone.rotation.x = THREE.MathUtils.degToRad(-droneState.pitch);
            drone.rotation.z = THREE.MathUtils.degToRad(-droneState.roll);
            drone.rotation.y = THREE.MathUtils.degToRad(-droneState.yaw);
            
            // Update altitude
            const targetY = 0.5 + droneState.throttle * 3;
            drone.position.y += (targetY - drone.position.y) * 0.1;
            
            renderer.render(scene, camera);
        }
        
        animate();
    </script>
</body>
</html>
'''

def main():
    if len(sys.argv) < 2:
        print()
        print("╔══════════════════════════════════════════════════════════╗")
        print("║        Universal Drone 3D Visualizer v2.0               ║")
        print("╚══════════════════════════════════════════════════════════╝")
        print()
        print("Usage: python drone_visualizer.py <PORT>")
        print()
        print("Examples:")
        print("  python drone_visualizer.py COM3")
        print("  python drone_visualizer.py /dev/ttyUSB0")
        print()
        print("Then open: http://localhost:5000")
        print("Switch protocols in the web UI!")
        print()
        sys.exit(1)
    
    port = sys.argv[1]
    global port_name_global
    port_name_global = port
    
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║        Universal Drone 3D Visualizer v2.0               ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print(f"⚡ Serial Port: {port}")
    print(f"📡 Protocol: {drone_state['protocol']} (switchable in web UI)")
    
    # Start serial reader thread
    reader_thread = threading.Thread(
        target=serial_reader_thread, 
        args=(port,), 
        daemon=True
    )
    reader_thread.start()
    
    time.sleep(1)
    
    print(f"🌐 Web Server: http://localhost:5000")
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║  → Open: http://localhost:5000                           ║")
    print("║  → Switch protocols in web UI (CRSF/SBUS/iBus)          ║")
    print("║  → Switch ANGLE/ACRO flight modes                        ║")
    print("║  → Press Ctrl+C to stop                                  ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n✓ Stopped")

if __name__ == "__main__":
    main()
