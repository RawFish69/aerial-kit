// Custom TX/RX Main
// Use BUILD_TX or BUILD_RX build flag to select mode

#include <Arduino.h>
#include "protocol.h"
#include "config.h"

#ifdef BUILD_TX
#include <Adafruit_NeoPixel.h>
#if TX_CONTROL_MODE == 1
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#endif
#if TX_CONTROL_MODE == 0
#include "imu_handler.h"
#endif
#endif

// Build mode check
#if !defined(BUILD_TX) && !defined(BUILD_RX)
  #error "Please define either BUILD_TX or BUILD_RX in platformio.ini"
#endif

#ifdef BUILD_TX
#if TX_CONTROL_MODE == 1
// ============================================================
// TRANSMITTER (TX) CODE - Web-Based Control
// ============================================================

// TX Configuration
#define RC_SEND_FREQUENCY_HZ 100  // Options: 50, 100, 150, 250, 500 Hz

// WiFi AP Configuration
const char* AP_SSID = "Quad Controller";
const char* AP_PASSWORD = "";  // No password

// Web server
AsyncWebServer server(80);

// NeoPixel LED Configuration
// M5Stack ESP32-C3 (Stamp C3) - Common pins: GPIO 2 or GPIO 8
// If GPIO 2 doesn't work, try changing to GPIO 8
#define LED_PIN 2              // M5Stack ESP32-C3 LED pin (try GPIO 2 or GPIO 8)
#define LED_COUNT 1            // Single LED
#define LED_BRIGHTNESS 255     // Brightness (0-255) - max brightness for testing

// LED Status Colors
#define COLOR_RED 0xFF0000      // System errors
#define COLOR_ORANGE 0xFF6600   // WiFi OK, TX/RX sending failures
#define COLOR_YELLOW 0xFFFF00   // WiFi OK, TX/RX OK, no devices connected
#define COLOR_GREEN 0x00FF00    // WiFi OK, TX/RX OK, device(s) connected
#define COLOR_CYAN 0x00FFFF     // User actively controlling

Adafruit_NeoPixel led = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// LED state tracking
uint32_t currentColor = COLOR_RED;
uint32_t targetColor = COLOR_RED;
unsigned long lastColorUpdateMs = 0;
unsigned long lastUserActivityMs = 0;
const unsigned long COLOR_TRANSITION_TIME = 500;  // 500ms for smooth transition
const unsigned long USER_ACTIVITY_TIMEOUT = 200;  // 200ms to detect user activity

// Broadcast to all receivers (no pairing needed!)
uint8_t receiverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// RC channel values (CRSF format: 172-1811, center: 992)
uint16_t rcChannels[16] = {
  992, 992, 172, 992, 992, 992, 992, 992,  // Channels 1-8
  992, 992, 992, 992, 992, 992, 992, 992   // Channels 9-16
};

// Web joystick values (normalized -1.0 to 1.0, throttle 0.0 to 1.0)
float webThrottle = 0.0;   // 0.0 to 1.0 (non-spring)
float webYaw = 0.0;        // -1.0 to 1.0 (spring)
float webPitch = 0.0;      // -1.0 to 1.0 (spring)
float webRoll = 0.0;       // -1.0 to 1.0 (spring)

// AUX channel states (from web toggles)
bool aux1State = false;    // Channel 4
bool aux2State = false;    // Channel 5
bool aux3State = false;    // Channel 6

// Timing
unsigned long lastRcSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastTxErrorMs = 0;
unsigned long lastLedUpdateMs = 0;

// System status tracking
bool systemError = false;
bool wifiOk = false;
bool txRxOk = false;
bool deviceConnected = false;
bool userActive = false;
unsigned long lastJoystickDataMs = 0;  // Track last joystick data received

// TX/RX failure tracking (for orange LED - only show on continuous failures)
unsigned long lastSuccessfulSendMs = 0;
unsigned long consecutiveFailures = 0;
const unsigned long TX_RX_FAILURE_THRESHOLD_MS = 2000;  // Show orange only after 2 seconds of continuous failures
const unsigned long TX_RX_FAILURE_COUNT_THRESHOLD = 10;  // Or after 10 consecutive failures

const unsigned long RC_SEND_INTERVAL = 1000 / RC_SEND_FREQUENCY_HZ;
const unsigned long ERROR_PRINT_INTERVAL = 1000;

// HTML page with virtual joysticks
const char* htmlPage = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Quad Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            touch-action: none;
            -webkit-user-select: none;
            user-select: none;
        }
        body {
            font-family: Arial, sans-serif;
            background: #1a1a1a;
            color: #fff;
            overflow: hidden;
            height: 100vh;
        }
        .header {
            background: #2a2a2a;
            padding: 10px;
            text-align: center;
            border-bottom: 2px solid #444;
        }
        .status {
            display: flex;
            justify-content: space-around;
            padding: 10px;
            background: #2a2a2a;
            margin: 10px;
            border-radius: 5px;
            font-size: 12px;
        }
        .status-item {
            text-align: center;
        }
        .status-label {
            color: #aaa;
            font-size: 10px;
        }
        .status-value {
            font-weight: bold;
            margin-top: 5px;
        }
        .status-value.active {
            color: #4CAF50;
        }
        .status-value.inactive {
            color: #f44336;
        }
        .controls {
            display: flex;
            justify-content: space-around;
            padding: 20px 10px;
            height: calc(100vh - 200px);
        }
        .joystick-container {
            width: 45%;
            height: 100%;
            position: relative;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .joystick {
            width: 200px;
            height: 200px;
            border-radius: 50%;
            background: #2a2a2a;
            border: 3px solid #444;
            position: relative;
            touch-action: none;
        }
        .joystick-stick {
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background: #4CAF50;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            transition: none;
            box-shadow: 0 2px 10px rgba(0,0,0,0.5);
        }
        .joystick-label {
            position: absolute;
            bottom: -30px;
            left: 50%;
            transform: translateX(-50%);
            font-size: 14px;
            color: #aaa;
            text-align: center;
        }
        .aux-section {
            background: #2a2a2a;
            padding: 15px;
            margin: 10px;
            border-radius: 5px;
        }
        .aux-title {
            text-align: center;
            margin-bottom: 10px;
            color: #aaa;
            font-size: 14px;
        }
        .aux-buttons {
            display: flex;
            justify-content: space-around;
            gap: 10px;
        }
        .aux-button {
            flex: 1;
            padding: 15px;
            background: #333;
            border: 2px solid #555;
            border-radius: 5px;
            color: #fff;
            font-size: 14px;
            cursor: pointer;
            transition: all 0.2s;
        }
        .aux-button.active {
            background: #4CAF50;
            border-color: #4CAF50;
        }
        .aux-button:active {
            transform: scale(0.95);
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Quad Controller</h1>
    </div>
    
    <div class="status">
        <div class="status-item">
            <div class="status-label">Link Status</div>
            <div class="status-value" id="linkStatus">DOWN</div>
        </div>
        <div class="status-item">
            <div class="status-label">Packets Sent</div>
            <div class="status-value" id="packetsSent">0</div>
        </div>
        <div class="status-item">
            <div class="status-label">Packets Received</div>
            <div class="status-value" id="packetsReceived">0</div>
        </div>
        <div class="status-item">
            <div class="status-label">Packet Loss</div>
            <div class="status-value" id="packetLoss">0%</div>
        </div>
        <div class="status-item">
            <div class="status-label">Last RX</div>
            <div class="status-value" id="lastRX">N/A</div>
        </div>
    </div>
    
    <div class="aux-section">
        <div class="aux-title">AUX Channels</div>
        <div class="aux-buttons">
            <button class="aux-button" id="aux1" onclick="toggleAux(1)">AUX1</button>
            <button class="aux-button" id="aux2" onclick="toggleAux(2)">AUX2</button>
            <button class="aux-button" id="aux3" onclick="toggleAux(3)">AUX3</button>
        </div>
    </div>
    
    <div class="controls">
        <div class="joystick-container">
            <div class="joystick" id="leftJoystick">
                <div class="joystick-stick" id="leftStick"></div>
                <div class="joystick-label">Throttle / Yaw</div>
            </div>
        </div>
        <div class="joystick-container">
            <div class="joystick" id="rightJoystick">
                <div class="joystick-stick" id="rightStick"></div>
                <div class="joystick-label">Pitch / Roll</div>
            </div>
        </div>
    </div>
    
    <script>
        let leftJoystick = document.getElementById('leftJoystick');
        let rightJoystick = document.getElementById('rightJoystick');
        let leftStick = document.getElementById('leftStick');
        let rightStick = document.getElementById('rightStick');
        
        let leftActive = false;
        let rightActive = false;
        let leftTouchId = null;
        let rightTouchId = null;
        
        let throttle = 0.0;  // 0.0 to 1.0 (non-spring)
        let yaw = 0.0;        // -1.0 to 1.0 (spring)
        let pitch = 0.0;     // -1.0 to 1.0 (spring)
        let roll = 0.0;      // -1.0 to 1.0 (spring)
        
        // Track last throttle stick position for non-spring behavior
        let lastThrottleStickY = 0.0;  // Last Y position of left stick (in pixels)
        
        // Keyboard state tracking
        let keysPressed = {
            w: false, s: false, a: false, d: false,  // Left stick (WASD)
            arrowUp: false, arrowDown: false, arrowLeft: false, arrowRight: false,  // Right stick (arrows)
            key1: false, key2: false, key3: false  // AUX channels (1, 2, 3)
        };
        
        // Current keyboard-controlled joystick positions (in pixels, relative to center)
        let keyboardLeftStickX = 0.0;   // Yaw position
        let keyboardLeftStickY = 0.0;   // Throttle position (stays when key released)
        let keyboardRightStickX = 0.0;   // Roll position
        let keyboardRightStickY = 0.0;   // Pitch position
        
        // Keyboard control speed (how fast values change per update)
        const KEYBOARD_SPEED = 2.0;  // Pixels per update (adjust for sensitivity)
        const KEYBOARD_UPDATE_INTERVAL = 20;  // Update every 20ms for smooth control
        
        let auxStates = {1: false, 2: false, 3: false};
        
        function updateJoystick(joystick, stick, x, y, isLeft) {
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;
            const radius = rect.width / 2 - 30;
            
            const dx = x - centerX;
            const dy = y - centerY;
            const distance = Math.min(Math.sqrt(dx*dx + dy*dy), radius);
            const angle = Math.atan2(dy, dx);
            
            const stickX = Math.cos(angle) * distance;
            const stickY = Math.sin(angle) * distance;
            
            stick.style.transform = 'translate(calc(-50% + ' + stickX + 'px), calc(-50% + ' + stickY + 'px))';
            
            // Normalize to -1.0 to 1.0
            const normalizedX = stickX / radius;
            const normalizedY = -stickY / radius;  // Invert Y (up is positive)
            
            if (isLeft) {
                // Left joystick: Y = Throttle (0.0 to 1.0, non-spring), X = Yaw (-1.0 to 1.0, spring)
                throttle = Math.max(0.0, Math.min(1.0, (normalizedY + 1.0) / 2.0));
                yaw = normalizedX;
                // Store the stick Y position for non-spring behavior
                lastThrottleStickY = stickY;
    } else {
                // Right joystick: Y = Pitch (-1.0 to 1.0, spring), X = Roll (-1.0 to 1.0, spring)
                pitch = normalizedY;
                roll = normalizedX;
            }
            
            sendJoystickData();
        }
        
        function resetJoystick(joystick, stick, isLeft) {
            if (isLeft) {
                // Left stick: Throttle (Y) stays where it is (non-spring), Yaw (X) returns to center (spring)
                // Reset X to center (yaw), but keep Y (throttle) at last position
                stick.style.transform = 'translate(calc(-50% + 0px), calc(-50% + ' + lastThrottleStickY + 'px))';
                stick.style.transition = 'transform 0.2s';
                setTimeout(() => {
                    stick.style.transition = 'none';
                }, 200);
                
                // Only reset yaw, keep throttle
                yaw = 0.0;
                // throttle value stays the same (already set from last update)
            } else {
                // Right stick: Both pitch and roll return to center (spring)
                stick.style.transform = 'translate(-50%, -50%)';
                stick.style.transition = 'transform 0.2s';
                setTimeout(() => {
                    stick.style.transition = 'none';
                }, 200);
                
                pitch = 0.0;
                roll = 0.0;
            }
            
            sendJoystickData();
        }
        
        function handleTouchStart(e, joystick, stick, isLeft) {
            e.preventDefault();
            const touch = e.touches[0];
            const touchId = touch.identifier;
            
            if (isLeft) {
                leftActive = true;
                leftTouchId = touchId;
    } else {
                rightActive = true;
                rightTouchId = touchId;
            }
            
            updateJoystick(joystick, stick, touch.clientX, touch.clientY, isLeft);
        }
        
        function handleTouchMove(e, joystick, stick, isLeft) {
            e.preventDefault();
            let touch = null;
            
            if (isLeft && leftActive) {
                for (let t of e.touches) {
                    if (t.identifier === leftTouchId) {
                        touch = t;
                        break;
                    }
                }
            } else if (!isLeft && rightActive) {
                for (let t of e.touches) {
                    if (t.identifier === rightTouchId) {
                        touch = t;
                        break;
                    }
                }
            }
            
            if (touch) {
                updateJoystick(joystick, stick, touch.clientX, touch.clientY, isLeft);
            }
        }
        
        function handleTouchEnd(e, joystick, stick, isLeft) {
            e.preventDefault();
            let touchFound = false;
            
            if (isLeft && leftActive) {
                for (let t of e.changedTouches) {
                    if (t.identifier === leftTouchId) {
                        touchFound = true;
                        break;
                    }
                }
                if (touchFound) {
                    leftActive = false;
                    leftTouchId = null;
                    resetJoystick(joystick, stick, isLeft);
                }
            } else if (!isLeft && rightActive) {
                for (let t of e.changedTouches) {
                    if (t.identifier === rightTouchId) {
                        touchFound = true;
                        break;
                    }
                }
                if (touchFound) {
                    rightActive = false;
                    rightTouchId = null;
                    resetJoystick(joystick, stick, isLeft);
                }
            }
        }
        
        // Mouse events for desktop
        function handleMouseDown(e, joystick, stick, isLeft) {
            e.preventDefault();
            if (isLeft) leftActive = true;
            else rightActive = true;
            updateJoystick(joystick, stick, e.clientX, e.clientY, isLeft);
        }
        
        function handleMouseMove(e, joystick, stick, isLeft) {
            e.preventDefault();
            if ((isLeft && leftActive) || (!isLeft && rightActive)) {
                updateJoystick(joystick, stick, e.clientX, e.clientY, isLeft);
            }
        }
        
        function handleMouseUp(e, joystick, stick, isLeft) {
            e.preventDefault();
            if (isLeft) leftActive = false;
            else rightActive = false;
            resetJoystick(joystick, stick, isLeft);
        }
        
        // Attach event listeners
        leftJoystick.addEventListener('touchstart', (e) => handleTouchStart(e, leftJoystick, leftStick, true));
        leftJoystick.addEventListener('touchmove', (e) => handleTouchMove(e, leftJoystick, leftStick, true));
        leftJoystick.addEventListener('touchend', (e) => handleTouchEnd(e, leftJoystick, leftStick, true));
        leftJoystick.addEventListener('mousedown', (e) => handleMouseDown(e, leftJoystick, leftStick, true));
        document.addEventListener('mousemove', (e) => handleMouseMove(e, leftJoystick, leftStick, true));
        document.addEventListener('mouseup', (e) => handleMouseUp(e, leftJoystick, leftStick, true));
        
        rightJoystick.addEventListener('touchstart', (e) => handleTouchStart(e, rightJoystick, rightStick, false));
        rightJoystick.addEventListener('touchmove', (e) => handleTouchMove(e, rightJoystick, rightStick, false));
        rightJoystick.addEventListener('touchend', (e) => handleTouchEnd(e, rightJoystick, rightStick, false));
        rightJoystick.addEventListener('mousedown', (e) => handleMouseDown(e, rightJoystick, rightStick, false));
        document.addEventListener('mousemove', (e) => handleMouseMove(e, rightJoystick, rightStick, false));
        document.addEventListener('mouseup', (e) => handleMouseUp(e, rightJoystick, rightStick, false));
        
        function toggleAux(num) {
            auxStates[num] = !auxStates[num];
            const btn = document.getElementById('aux' + num);
            if (auxStates[num]) {
                btn.classList.add('active');
  } else {
                btn.classList.remove('active');
            }
            sendJoystickData();
        }
        
        function sendJoystickData() {
            fetch('/joystick', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    throttle: throttle,
                    yaw: yaw,
                    pitch: pitch,
                    roll: roll,
                    aux1: auxStates[1],
                    aux2: auxStates[2],
                    aux3: auxStates[3]
                })
            }).catch(err => console.error('Send error:', err));
        }
        
        // Update joystick from keyboard input (rate-based, continuous movement)
        function updateJoystickFromKeyboard() {
            // Only update if not actively touching/clicking
            if (leftActive || rightActive) return;
            
            const rect = leftJoystick.getBoundingClientRect();
            const radius = rect.width / 2 - 30;
            let changed = false;
            
            // Left stick: WASD - continuous rate-based movement
            if (keysPressed.w) {
                keyboardLeftStickY -= KEYBOARD_SPEED;  // Throttle up (continuous)
                changed = true;
            }
            if (keysPressed.s) {
                keyboardLeftStickY += KEYBOARD_SPEED;  // Throttle down (continuous)
                changed = true;
            }
            if (keysPressed.a) {
                keyboardLeftStickX -= KEYBOARD_SPEED;  // Yaw left (continuous)
                changed = true;
            } else if (!keysPressed.d && keyboardLeftStickX < 0) {
                // Return yaw to center when A is released (spring behavior)
                keyboardLeftStickX = Math.min(0, keyboardLeftStickX + KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardLeftStickX) < 0.5) keyboardLeftStickX = 0;
                changed = true;
            }
            if (keysPressed.d) {
                keyboardLeftStickX += KEYBOARD_SPEED;  // Yaw right (continuous)
                changed = true;
            } else if (!keysPressed.a && keyboardLeftStickX > 0) {
                // Return yaw to center when D is released (spring behavior)
                keyboardLeftStickX = Math.max(0, keyboardLeftStickX - KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardLeftStickX) < 0.5) keyboardLeftStickX = 0;
                changed = true;
            }
            // Note: Throttle (Y) doesn't return to center - stays where it is (non-spring)
            
            // Right stick: Arrow keys - continuous rate-based movement
            if (keysPressed.arrowUp) {
                keyboardRightStickY -= KEYBOARD_SPEED;  // Pitch up (continuous)
                changed = true;
            } else if (keyboardRightStickY < 0) {
                // Return pitch to center when up arrow released (spring behavior)
                keyboardRightStickY = Math.min(0, keyboardRightStickY + KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardRightStickY) < 0.5) keyboardRightStickY = 0;
                changed = true;
            }
            if (keysPressed.arrowDown) {
                keyboardRightStickY += KEYBOARD_SPEED;  // Pitch down (continuous)
                changed = true;
            } else if (keyboardRightStickY > 0) {
                // Return pitch to center when down arrow released (spring behavior)
                keyboardRightStickY = Math.max(0, keyboardRightStickY - KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardRightStickY) < 0.5) keyboardRightStickY = 0;
                changed = true;
            }
            if (keysPressed.arrowLeft) {
                keyboardRightStickX -= KEYBOARD_SPEED;  // Roll left (continuous)
                changed = true;
            } else if (keyboardRightStickX < 0) {
                // Return roll to center when left arrow released (spring behavior)
                keyboardRightStickX = Math.min(0, keyboardRightStickX + KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardRightStickX) < 0.5) keyboardRightStickX = 0;
                changed = true;
            }
            if (keysPressed.arrowRight) {
                keyboardRightStickX += KEYBOARD_SPEED;  // Roll right (continuous)
                changed = true;
            } else if (keyboardRightStickX > 0) {
                // Return roll to center when right arrow released (spring behavior)
                keyboardRightStickX = Math.max(0, keyboardRightStickX - KEYBOARD_SPEED * 2);
                if (Math.abs(keyboardRightStickX) < 0.5) keyboardRightStickX = 0;
                changed = true;
            }
            
            // Clamp to radius
            const leftDistance = Math.sqrt(keyboardLeftStickX * keyboardLeftStickX + keyboardLeftStickY * keyboardLeftStickY);
            if (leftDistance > radius) {
                keyboardLeftStickX = (keyboardLeftStickX / leftDistance) * radius;
                keyboardLeftStickY = (keyboardLeftStickY / leftDistance) * radius;
            }
            
            const rightDistance = Math.sqrt(keyboardRightStickX * keyboardRightStickX + keyboardRightStickY * keyboardRightStickY);
            if (rightDistance > radius) {
                keyboardRightStickX = (keyboardRightStickX / rightDistance) * radius;
                keyboardRightStickY = (keyboardRightStickY / rightDistance) * radius;
            }
            
            // Update left stick visual and values
            leftStick.style.transform = 'translate(calc(-50% + ' + keyboardLeftStickX + 'px), calc(-50% + ' + keyboardLeftStickY + 'px))';
            const normalizedX = keyboardLeftStickX / radius;
            const normalizedY = -keyboardLeftStickY / radius;
            throttle = Math.max(0.0, Math.min(1.0, (normalizedY + 1.0) / 2.0));
            yaw = normalizedX;
            lastThrottleStickY = keyboardLeftStickY;
            
            // Update right stick visual and values
            rightStick.style.transform = 'translate(calc(-50% + ' + keyboardRightStickX + 'px), calc(-50% + ' + keyboardRightStickY + 'px))';
            const normalizedRightX = keyboardRightStickX / radius;
            const normalizedRightY = -keyboardRightStickY / radius;
            pitch = normalizedRightY;
            roll = normalizedRightX;
            
            // Send data if anything changed
            if (changed) {
                sendJoystickData();
            }
        }
        
        // Keyboard event handlers
        function handleKeyDown(e) {
            // Prevent default behavior for game keys
            const key = e.key.toLowerCase();
            if (key === 'w' || key === 's' || key === 'a' || key === 'd' || 
                e.key === 'ArrowUp' || e.key === 'ArrowDown' || 
                e.key === 'ArrowLeft' || e.key === 'ArrowRight' ||
                key === '1' || key === '2' || key === '3') {
                e.preventDefault();
            }
            
            // Update key states
            if (key === 'w') keysPressed.w = true;
            else if (key === 's') keysPressed.s = true;
            else if (key === 'a') keysPressed.a = true;
            else if (key === 'd') keysPressed.d = true;
            else if (e.key === 'ArrowUp') keysPressed.arrowUp = true;
            else if (e.key === 'ArrowDown') keysPressed.arrowDown = true;
            else if (e.key === 'ArrowLeft') keysPressed.arrowLeft = true;
            else if (e.key === 'ArrowRight') keysPressed.arrowRight = true;
            else if (key === '1') {
                keysPressed.key1 = true;
                toggleAux(1);
            }
            else if (key === '2') {
                keysPressed.key2 = true;
                toggleAux(2);
            }
            else if (key === '3') {
                keysPressed.key3 = true;
                toggleAux(3);
            }
        }
        
        function handleKeyUp(e) {
            const key = e.key.toLowerCase();
            
            // Update key states
            if (key === 'w') keysPressed.w = false;
            else if (key === 's') keysPressed.s = false;
            else if (key === 'a') keysPressed.a = false;
            else if (key === 'd') keysPressed.d = false;
            else if (e.key === 'ArrowUp') keysPressed.arrowUp = false;
            else if (e.key === 'ArrowDown') keysPressed.arrowDown = false;
            else if (e.key === 'ArrowLeft') keysPressed.arrowLeft = false;
            else if (e.key === 'ArrowRight') keysPressed.arrowRight = false;
            else if (key === '1') keysPressed.key1 = false;
            else if (key === '2') keysPressed.key2 = false;
            else if (key === '3') keysPressed.key3 = false;
        }
        
        // Attach keyboard event listeners
        document.addEventListener('keydown', handleKeyDown);
        document.addEventListener('keyup', handleKeyUp);
        
        // Continuous keyboard update loop
        setInterval(updateJoystickFromKeyboard, KEYBOARD_UPDATE_INTERVAL);
        
        // Update status periodically
        setInterval(() => {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('linkStatus').textContent = data.linkActive ? 'ACTIVE' : 'DOWN';
                    document.getElementById('linkStatus').className = 'status-value ' + (data.linkActive ? 'active' : 'inactive');
                    document.getElementById('packetsSent').textContent = data.packetsSent;
                    document.getElementById('packetsReceived').textContent = data.packetsReceived;
                    document.getElementById('packetLoss').textContent = data.packetLoss.toFixed(1) + '%';
                    document.getElementById('lastRX').textContent = data.lastPacketMs + 'ms';
                })
                .catch(err => console.error('Status error:', err));
        }, 500);
    </script>
</body>
</html>
)HTML";

// Helper function to blend colors smoothly
uint32_t blendColor(uint32_t color1, uint32_t color2, float ratio) {
  ratio = constrain(ratio, 0.0, 1.0);
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;
  
  uint8_t r = (uint8_t)(r1 + (r2 - r1) * ratio);
  uint8_t g = (uint8_t)(g1 + (g2 - g1) * ratio);
  uint8_t b = (uint8_t)(b1 + (b2 - b1) * ratio);
  
  return (r << 16) | (g << 8) | b;
}

// Update LED based on system status
void updateLED() {
  unsigned long now = millis();
  
  // Check if TX/RX is continuously failing (for orange LED)
  bool txRxContinuouslyFailing = false;
  if (txRxOk) {
    // TX/RX is working, reset failure tracking
    consecutiveFailures = 0;
    lastSuccessfulSendMs = now;
  } else {
    // Check if we have sustained failures
    unsigned long timeSinceLastSuccess = now - lastSuccessfulSendMs;
    if (timeSinceLastSuccess > TX_RX_FAILURE_THRESHOLD_MS || 
        consecutiveFailures > TX_RX_FAILURE_COUNT_THRESHOLD) {
      txRxContinuouslyFailing = true;
    }
  }
  
  // Determine target color based on status (priority order matters!)
  // Priority: RED (errors) > ORANGE (continuous TX/RX failure) > YELLOW (no device) > CYAN (user active) > GREEN (all good)
  if (systemError || !wifiOk) {
    // Highest priority: System errors or WiFi down = RED
    targetColor = COLOR_RED;
  } else if (txRxContinuouslyFailing) {
    // Second priority: Only show orange if TX/RX is continuously failing (not just occasional failures)
    targetColor = COLOR_ORANGE;
  } else if (!deviceConnected) {
    // Third priority: WiFi OK, TX/RX OK, but no device connected = YELLOW
    targetColor = COLOR_YELLOW;
  } else if (userActive) {
    // Fourth priority: User actively controlling = CYAN
    targetColor = COLOR_CYAN;
  } else {
    // Lowest priority (best state): Everything working, device connected, user idle = GREEN
    targetColor = COLOR_GREEN;
  }
  
  // Smooth color transition
  if (targetColor != currentColor) {
    float elapsed = (float)(now - lastColorUpdateMs);
    float ratio = elapsed / COLOR_TRANSITION_TIME;
    
    if (ratio >= 1.0) {
      currentColor = targetColor;
      } else {
      currentColor = blendColor(currentColor, targetColor, ratio);
    }
    
    if (ratio >= 1.0) {
      lastColorUpdateMs = now;
    }
  } else {
    lastColorUpdateMs = now;
  }
  
  // Update LED
  led.setPixelColor(0, currentColor);
  led.setBrightness(LED_BRIGHTNESS);
  led.show();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Transmitter (TX)");
  Serial.println("Web-Based Control");
  Serial.println("=================================\n");
  
  // Initialize NeoPixel LED
  Serial.println("[LED] Initializing NeoPixel on GPIO 2...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure pin is ready
  delay(50);
  
  led.begin();
  led.clear();  // Clear all pixels
  led.setBrightness(LED_BRIGHTNESS);
  led.show();
  delay(100);
  
  // Test LED - flash red a few times to confirm it's working
  Serial.println("[LED] Testing LED...");
  for (int i = 0; i < 3; i++) {
    led.setPixelColor(0, COLOR_RED);
    led.show();
    delay(300);
    led.setPixelColor(0, 0);  // Off
    led.show();
    delay(300);
  }
  led.setPixelColor(0, COLOR_RED);
  led.show();
  Serial.println("[LED] NeoPixel initialized (RED = Starting)");
  
  // Initialize WiFi AP_STA mode (both AP and STA for ESP-NOW compatibility)
  Serial.println("[WiFi] Setting up Access Point...");
  
  // Make sure WiFi is off first
  WiFi.mode(WIFI_OFF);
  delay(200);
  
  // Set to AP_STA mode
  WiFi.mode(WIFI_AP_STA);
  delay(300);
  
  // Start the AP on channel 1 (same as ESP-NOW will use)
  Serial.println("[WiFi] Starting AP...");
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);  // channel 1, hidden=false, max_connection=4
  if (!apStarted) {
    Serial.println("[WiFi] ERROR: Failed to start AP! Retrying...");
    delay(500);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_AP_STA);
    delay(300);
    apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);
    if (!apStarted) {
      Serial.println("[WiFi] CRITICAL: AP failed to start after retry!");
    }
  }
  
  delay(1500);  // Give AP plenty of time to fully start
  
  IPAddress IP = WiFi.softAPIP();
  wifi_mode_t currentMode = WiFi.getMode();
  Serial.printf("[WiFi] AP Started: %s (Status: %s)\n", AP_SSID, apStarted ? "OK" : "FAILED");
  Serial.printf("[WiFi] IP Address: %s\n", IP.toString().c_str());
  Serial.printf("[WiFi] WiFi Mode: %d (3=AP_STA)\n", (int)currentMode);
  Serial.printf("[WiFi] AP SSID: %s\n", WiFi.softAPSSID().c_str());
  Serial.printf("[WiFi] AP Clients: %d\n", WiFi.softAPgetStationNum());
  Serial.println("[WiFi] Connect to this network and open http://192.168.4.1");
  Serial.println();
  
  if (!apStarted || IP.toString() == "0.0.0.0") {
    Serial.println("[WiFi] CRITICAL: AP failed to start properly!");
    Serial.println("[WiFi] Please check Serial Monitor for errors");
    systemError = true;
    wifiOk = false;
    updateLED();
    // Don't continue if AP failed
    while (1) {
      delay(1000);
      Serial.println("[WiFi] Waiting for AP to start...");
      WiFi.mode(WIFI_OFF);
      delay(200);
      WiFi.mode(WIFI_AP_STA);
      delay(300);
      if (WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4)) {
        IP = WiFi.softAPIP();
        if (IP.toString() != "0.0.0.0") {
          Serial.printf("[WiFi] AP finally started! IP: %s\n", IP.toString().c_str());
          systemError = false;
          wifiOk = true;
          updateLED();
          break;
        }
      }
      updateLED();
    }
      } else {
    wifiOk = true;
    systemError = false;
    updateLED();
  }
  
  // Initialize protocol (TX mode, broadcast)
  // ESP-NOW init will be called, but it should NOT touch WiFi if in AP_STA mode
  Serial.println("[TX] Initializing ESP-NOW (AP should remain active)...");
  if (!CustomProtocol_Init(true, receiverMac)) {
    Serial.println("[TX] Protocol init failed!");
    systemError = true;
    txRxOk = false;
    updateLED();
      while (1) {
        delay(1000);
      updateLED();
    }
  } else {
    txRxOk = true;
    updateLED();
  }
  
  // CRITICAL: Verify AP is still running after ESP-NOW init
  delay(1000);
  wifi_mode_t modeAfter = WiFi.getMode();
  IPAddress ipAfter = WiFi.softAPIP();
  Serial.printf("[WiFi] After ESP-NOW init - Mode: %d, IP: %s\n", (int)modeAfter, ipAfter.toString().c_str());
  
  if (modeAfter != WIFI_AP_STA || ipAfter.toString() == "0.0.0.0") {
    Serial.println("[WiFi] CRITICAL: AP was disrupted! Restarting...");
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_AP_STA);
    delay(300);
    WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);
    delay(1000);
    ipAfter = WiFi.softAPIP();
    Serial.printf("[WiFi] AP Restarted. IP: %s\n", ipAfter.toString().c_str());
  }
  
  // Final verification
  if (WiFi.softAPIP().toString() == "0.0.0.0") {
    Serial.println("[WiFi] FATAL: AP still not working after all attempts!");
    systemError = true;
    wifiOk = false;
      } else {
    Serial.println("[WiFi] AP is running and ready!");
    wifiOk = true;
    systemError = false;
  }
  updateLED();
  
  Serial.println("[TX] Ready to transmit!");
  Serial.println("[TX] Broadcasting to all receivers");
  Serial.printf("[TX] RC commands: %d Hz | Heartbeat: 2 Hz | Stats: 1 Hz\n", RC_SEND_FREQUENCY_HZ);
  Serial.println();
  
  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", htmlPage);
  });
  
  // Receive joystick data
  server.on("/joystick", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
      StaticJsonDocument<200> doc;
      deserializeJson(doc, (char*)data);
      
      float newThrottle = doc["throttle"] | 0.0;
      float newYaw = doc["yaw"] | 0.0;
      float newPitch = doc["pitch"] | 0.0;
      float newRoll = doc["roll"] | 0.0;
      bool newAux1 = doc["aux1"] | false;
      bool newAux2 = doc["aux2"] | false;
      bool newAux3 = doc["aux3"] | false;
      
      // Detect user activity (joystick movement or channel changes)
      if (abs(newThrottle - webThrottle) > 0.01 || abs(newYaw - webYaw) > 0.01 ||
          abs(newPitch - webPitch) > 0.01 || abs(newRoll - webRoll) > 0.01 ||
          newAux1 != aux1State || newAux2 != aux2State || newAux3 != aux3State) {
        lastUserActivityMs = millis();
        userActive = true;
      }
      
      webThrottle = newThrottle;
      webYaw = newYaw;
      webPitch = newPitch;
      webRoll = newRoll;
      aux1State = newAux1;
      aux2State = newAux2;
      aux3State = newAux3;
      
      // Mark device as connected and update timestamp
      deviceConnected = true;
      lastJoystickDataMs = millis();
      
      request->send(200, "text/plain", "OK");
    });
  
  // Send status
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    StaticJsonDocument<200> doc;
    doc["linkActive"] = stats.linkActive;
    doc["packetsSent"] = stats.packetsSent;
    doc["packetsReceived"] = stats.packetsReceived;
    doc["packetLoss"] = stats.packetLossPercent;
    doc["lastPacketMs"] = stats.lastPacketMs;
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });
  
  server.begin();
  Serial.println("[Web] Server started!");
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol (handles timeouts, etc.)
  CustomProtocol_Update();
  
  // Check user activity timeout
  if (userActive && (now - lastUserActivityMs > USER_ACTIVITY_TIMEOUT)) {
    userActive = false;
  }
  
  // Check device connection - ONLY if WiFi is OK
  // If WiFi is down, device connection is irrelevant (will show RED anyway)
  if (wifiOk) {
    // Check device connection (if no joystick data received in 5 seconds, consider disconnected)
    if (deviceConnected && (now - lastJoystickDataMs > 5000)) {
      deviceConnected = false;
    }
    
    // Also check WiFi client count
    int connectedClients = WiFi.softAPgetStationNum();
    if (connectedClients > 0) {
      deviceConnected = true;
      lastJoystickDataMs = now;  // Reset timeout if clients are connected
    }
    } else {
    // WiFi is down - don't check device connection (will show RED)
    deviceConnected = false;
  }
  
  // Update LED continuously for smooth transitions
  if (now - lastLedUpdateMs >= 20) {  // Update LED at 50Hz for smooth transitions
    lastLedUpdateMs = now;
    updateLED();
  }
  
  // Send RC commands at configured frequency
  if (now - lastRcSendMs >= RC_SEND_INTERVAL) {
    lastRcSendMs = now;
    
    // Map web joystick values to CRSF RC channel values (172-1811, center: 992)
    // Roll: -1.0 to 1.0 -> 172 to 1811 (center: 992)
    rcChannels[0] = (uint16_t)(webRoll * 819.0 + 992.0);
    
    // Pitch: -1.0 to 1.0 -> 172 to 1811 (center: 992)
    rcChannels[1] = (uint16_t)(webPitch * 819.0 + 992.0);
    
    // Throttle: 0.0 to 1.0 -> 172 to 1811
    rcChannels[2] = (uint16_t)(172.0 + webThrottle * 1639.0);
    
    // Yaw: -1.0 to 1.0 -> 172 to 1811 (center: 992)
    // Invert yaw so Betaflight sees left rotation as negative.
    rcChannels[3] = (uint16_t)((-webYaw) * 819.0 + 992.0);
    
    // AUX channels
    rcChannels[4] = aux1State ? 1811 : 172;  // AUX1
    rcChannels[5] = aux2State ? 1811 : 172;  // AUX2
    rcChannels[6] = aux3State ? 1811 : 172;  // AUX3
    
    // Clamp values to valid range
    for (int i = 0; i < 16; i++) {
      if (rcChannels[i] < 172) rcChannels[i] = 172;
      if (rcChannels[i] > 1811) rcChannels[i] = 1811;
    }
    
    // Send RC command
    if (!CustomProtocol_SendRcCommand(rcChannels)) {
      txRxOk = false;  // Mark TX/RX as having issues
      consecutiveFailures++;
      if (now - lastTxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[TX] Failed to send RC command (rate-limited errors)");
        lastTxErrorMs = now;
      }
    } else {
      txRxOk = true;  // TX/RX is working
      consecutiveFailures = 0;  // Reset failure counter on success
      lastSuccessfulSendMs = now;
    }
  }
  
  // Send heartbeat at 2 Hz
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    CustomProtocol_SendHeartbeat();
  }
  
  // Print statistics every second
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    // CRITICAL: Check if AP is still running every second
    wifi_mode_t currentMode = WiFi.getMode();
    IPAddress apIP = WiFi.softAPIP();
    if (currentMode != WIFI_AP_STA || apIP.toString() == "0.0.0.0") {
      Serial.println("\n[WiFi] CRITICAL: AP DOWN! Restarting immediately...");
      wifiOk = false;
      systemError = true;
      WiFi.mode(WIFI_OFF);
      delay(200);
      WiFi.mode(WIFI_AP_STA);
      delay(300);
      WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);
      delay(1000);
      apIP = WiFi.softAPIP();
      Serial.printf("[WiFi] AP Restarted. IP: %s, Mode: %d\n", apIP.toString().c_str(), (int)WiFi.getMode());
      if (apIP.toString() == "0.0.0.0") {
        Serial.println("[WiFi] ERROR: AP restart failed! Will retry next cycle.");
        wifiOk = false;
        systemError = true;
      } else {
        wifiOk = true;
        systemError = false;
      }
    } else {
      wifiOk = true;
      systemError = false;
    }
    
    // Update device connection status based on WiFi clients
    // Only check if WiFi is OK
    if (wifiOk) {
      int connectedClients = WiFi.softAPgetStationNum();
      if (connectedClients > 0) {
        deviceConnected = true;
        lastJoystickDataMs = now;  // Reset timeout if clients are connected
      }
    } else {
      // WiFi is down, so no devices can be connected
      deviceConnected = false;
    }
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- TX Statistics ---");
    Serial.printf("Packets Sent: %lu | Received: %lu | Loss: %.1f%%\n", 
                  stats.packetsSent, stats.packetsReceived, stats.packetLossPercent);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "DOWN", stats.lastPacketMs);
    
    // Display telemetry if received
    if (stats.packetsReceived > 0) {
      TelemetryPayload telemetry;
      CustomProtocol_GetTelemetry(&telemetry);
      
      if (telemetry.voltage > 0.1) {
        Serial.printf("Battery: %.2fV ", telemetry.voltage);
      } else {
        Serial.print("Battery: N/A ");
      }
      
      if (telemetry.current > 0.1) {
        Serial.printf("%.2fA | ", telemetry.current);
      } else {
        Serial.print("N/A | ");
      }
      
      Serial.printf("RSSI: %ddBm | LQ: %d%%\n", 
                    telemetry.rssi, telemetry.linkQuality);
    }
    
    Serial.printf("Web Joystick: T:%.2f Y:%.2f P:%.2f R:%.2f | AUX1:%d AUX2:%d AUX3:%d\n",
                  webThrottle, webYaw, webPitch, webRoll,
                  aux1State ? 1 : 0, aux2State ? 1 : 0, aux3State ? 1 : 0);
    Serial.printf("RC Channels: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d AUX3:%d\n",
                  rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3],
                  rcChannels[4], rcChannels[5], rcChannels[6]);
    Serial.println();
  }
  
  delay(1);
}

#elif TX_CONTROL_MODE == 0
// ============================================================
// TRANSMITTER (TX) CODE - IMU + Potentiometer Control
// ============================================================

// NeoPixel LED Configuration
#define LED_PIN 2
#define LED_COUNT 1
#define LED_BRIGHTNESS 255

// LED Status Colors
#define COLOR_RED 0xFF0000
#define COLOR_ORANGE 0xFF6600
#define COLOR_YELLOW 0xFFFF00
#define COLOR_GREEN 0x00FF00
#define COLOR_CYAN 0x00FFFF

Adafruit_NeoPixel led = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t currentColor = COLOR_RED;
uint32_t targetColor = COLOR_RED;
unsigned long lastColorUpdateMs = 0;
unsigned long lastLedUpdateMs = 0;
const unsigned long COLOR_TRANSITION_TIME = 500;

bool systemError = false;
bool wifiOk = true;
bool txRxOk = false;
bool deviceConnected = true;
bool userActive = true;

unsigned long lastRcSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastTxErrorMs = 0;

unsigned long lastSuccessfulSendMs = 0;
unsigned long consecutiveFailures = 0;
const unsigned long TX_RX_FAILURE_THRESHOLD_MS = 2000;
const unsigned long TX_RX_FAILURE_COUNT_THRESHOLD = 10;

uint8_t receiverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint16_t rcChannels[16] = {
  992, 992, 172, 992, 992, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

const unsigned long RC_SEND_INTERVAL = 1000 / RC_SEND_FREQUENCY_HZ;
const unsigned long ERROR_PRINT_INTERVAL = 1000;

IMUHandler imu;
int potMinValue = 0;
int potMaxValue = 4095;
bool potCalibrated = false;

uint32_t blendColor(uint32_t color1, uint32_t color2, float ratio) {
  ratio = constrain(ratio, 0.0f, 1.0f);
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;

  uint8_t r = (uint8_t)(r1 + (r2 - r1) * ratio);
  uint8_t g = (uint8_t)(g1 + (g2 - g1) * ratio);
  uint8_t b = (uint8_t)(b1 + (b2 - b1) * ratio);

  return (r << 16) | (g << 8) | b;
}

void updateLED() {
  unsigned long now = millis();
  bool txRxContinuouslyFailing = false;

  if (txRxOk) {
    consecutiveFailures = 0;
    lastSuccessfulSendMs = now;
  } else {
    unsigned long timeSinceLastSuccess = now - lastSuccessfulSendMs;
    if (timeSinceLastSuccess > TX_RX_FAILURE_THRESHOLD_MS ||
        consecutiveFailures > TX_RX_FAILURE_COUNT_THRESHOLD) {
      txRxContinuouslyFailing = true;
    }
  }

  if (systemError || !wifiOk) {
    targetColor = COLOR_RED;
  } else if (txRxContinuouslyFailing) {
    targetColor = COLOR_ORANGE;
  } else if (!deviceConnected) {
    targetColor = COLOR_YELLOW;
  } else if (userActive) {
    targetColor = COLOR_CYAN;
  } else {
    targetColor = COLOR_GREEN;
  }

  if (targetColor != currentColor) {
    float elapsed = (float)(now - lastColorUpdateMs);
    float ratio = elapsed / (float)COLOR_TRANSITION_TIME;
    if (ratio >= 1.0f) {
      currentColor = targetColor;
    } else {
      currentColor = blendColor(currentColor, targetColor, ratio);
    }
    if (ratio >= 1.0f) {
      lastColorUpdateMs = now;
    }
  } else {
    lastColorUpdateMs = now;
  }

  led.setPixelColor(0, currentColor);
  led.setBrightness(LED_BRIGHTNESS);
  led.show();
}

void calibratePotentiometer() {
  Serial.println("[TX] Calibrating potentiometer...");
  Serial.println("     Please move the potentiometer to MIN position...");
  int minVal = 4095;
  int maxVal = 0;
  delay(2000);
  for (int i = 0; i < POT_CALIBRATION_SAMPLES; i++) {
    int val = analogRead(THROTTLE_POT_PIN);
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
    delay(20);
  }

  Serial.println("     Now move the potentiometer to MAX position...");
  delay(2000);
  for (int i = 0; i < POT_CALIBRATION_SAMPLES; i++) {
    int val = analogRead(THROTTLE_POT_PIN);
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
    delay(20);
  }

  potMinValue = minVal;
  potMaxValue = maxVal;
  potCalibrated = (potMaxValue > potMinValue);

  Serial.printf("[TX] Pot calibration complete: MIN=%d MAX=%d\n", potMinValue, potMaxValue);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Transmitter (TX)");
  Serial.println("=================================\n");
  Serial.println("[TX] IMU + Potentiometer Control Mode");
  Serial.printf("[TX] RC commands: %d Hz | Heartbeat: 2 Hz | Stats: 1 Hz\n", RC_SEND_FREQUENCY_HZ);
  Serial.println();

  led.begin();
  led.setPixelColor(0, 0);
  led.show();
  delay(100);
  led.setPixelColor(0, COLOR_RED);
  led.show();

  if (!CustomProtocol_Init(true, receiverMac)) {
    Serial.println("[TX] Protocol init failed!");
    systemError = true;
    txRxOk = false;
    updateLED();
    while (1) {
      delay(1000);
    }
  } else {
    txRxOk = true;
    updateLED();
  }

  Serial.println("[TX] Initializing IMU...");
  if (!imu.begin(IMU_SDA_PIN, IMU_SCL_PIN, static_cast<IMUType>(IMU_TYPE))) {
    Serial.println("[TX] IMU initialization failed!");
    systemError = true;
    while (1) {
      delay(1000);
    }
  }

  if (IMU_TYPE == IMU_TYPE_BNO085) {
    Serial.println("[TX] IMU Type: BNO085 (sensor fusion)");
  } else {
    Serial.println("[TX] IMU Type: MPU6050 (gyro + accel)");
  }

  imu.setRollSensitivity(ROLL_SENSITIVITY);
  imu.setPitchSensitivity(PITCH_SENSITIVITY);
  imu.setYawSensitivity(YAW_SENSITIVITY);

  pinMode(THROTTLE_POT_PIN, INPUT);
  calibratePotentiometer();

  wifiOk = true;
  deviceConnected = true;
  userActive = true;

  Serial.println("[TX] All controls initialized!");
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  CustomProtocol_Update();

  if (now - lastLedUpdateMs >= 20) {
    lastLedUpdateMs = now;
    updateLED();
  }

  if (now - lastRcSendMs >= RC_SEND_INTERVAL) {
    lastRcSendMs = now;

    imu.update();
    float rollNormalized = imu.getRoll();
    float pitchNormalized = imu.getPitch();
    float yawNormalized = imu.getYaw();

    int potValue = analogRead(THROTTLE_POT_PIN);
    float throttleNormalized = 0.0f;
    if (potCalibrated && (potMaxValue > potMinValue)) {
      throttleNormalized = (float)(potValue - potMinValue) / (float)(potMaxValue - potMinValue);
    } else {
      throttleNormalized = (float)potValue / 4095.0f;
    }
    throttleNormalized = constrain(throttleNormalized, 0.0f, 1.0f);

    rcChannels[0] = (uint16_t)(rollNormalized * 819.0f + 992.0f);
    rcChannels[1] = (uint16_t)(pitchNormalized * 819.0f + 992.0f);
    rcChannels[2] = (uint16_t)(172.0f + throttleNormalized * 1639.0f);
    // Invert yaw so Betaflight sees left rotation as negative.
    rcChannels[3] = (uint16_t)((-yawNormalized) * 819.0f + 992.0f);
    rcChannels[4] = 992;
    rcChannels[5] = 992;
    rcChannels[6] = 992;

    for (int i = 0; i < 16; i++) {
      if (rcChannels[i] < 172) rcChannels[i] = 172;
      if (rcChannels[i] > 1811) rcChannels[i] = 1811;
    }

    if (!CustomProtocol_SendRcCommand(rcChannels)) {
      txRxOk = false;
      consecutiveFailures++;
      if (now - lastTxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[TX] Failed to send RC command (rate-limited errors)");
        lastTxErrorMs = now;
      }
    } else {
      txRxOk = true;
      consecutiveFailures = 0;
      lastSuccessfulSendMs = now;
    }

    userActive = true;
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    CustomProtocol_SendHeartbeat();
  }

  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;

    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);

    Serial.println("\n--- TX Statistics ---");
    Serial.printf("Packets Sent: %lu | Received: %lu | Loss: %.1f%%\n",
                  stats.packetsSent, stats.packetsReceived, stats.packetLossPercent);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n",
                  stats.linkActive ? "ACTIVE" : "DOWN", stats.lastPacketMs);

    if (stats.packetsReceived > 0) {
      TelemetryPayload telemetry;
      CustomProtocol_GetTelemetry(&telemetry);
      if (telemetry.voltage > 0.1) {
        Serial.printf("Battery: %.2fV ", telemetry.voltage);
      } else {
        Serial.print("Battery: N/A ");
      }
      if (telemetry.current > 0.1) {
        Serial.printf("%.2fA | ", telemetry.current);
      } else {
        Serial.print("N/A | ");
      }
      Serial.printf("RSSI: %ddBm | LQ: %d%%\n",
                    telemetry.rssi, telemetry.linkQuality);
    }

    float rollAngle, pitchAngle, yawAngle;
    imu.getAngles(rollAngle, pitchAngle, yawAngle);

    int potValue = analogRead(THROTTLE_POT_PIN);
    float throttleNormalized = 0.0f;
    if (potCalibrated && (potMaxValue > potMinValue)) {
      throttleNormalized = (float)(potValue - potMinValue) / (float)(potMaxValue - potMinValue);
    } else {
      throttleNormalized = (float)potValue / 4095.0f;
    }
    throttleNormalized = constrain(throttleNormalized, 0.0f, 1.0f);

    Serial.printf("IMU Angles (°): Roll:%.1f Pitch:%.1f Yaw:%.1f | Normalized: R:%.2f P:%.2f Y:%.2f\n",
                  rollAngle, pitchAngle, yawAngle, imu.getRoll(), imu.getPitch(), imu.getYaw());
    Serial.printf("Pot ADC: %d (min:%d max:%d) | Throttle: %.2f (%.1f%%)\n",
                  potValue, potMinValue, potMaxValue, throttleNormalized, throttleNormalized * 100.0f);
    Serial.printf("RC Channels: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d AUX3:%d\n",
                  rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3],
                  rcChannels[4], rcChannels[5], rcChannels[6]);
    Serial.println();
  }

  delay(1);
}


#else
#error "TX_CONTROL_MODE must be 0 (IMU) or 1 (Web)."
#endif // TX control mode

#endif // BUILD_TX

#ifdef BUILD_RX
// ============================================================
// RECEIVER (RX) CODE
// ============================================================

#include "crsf_bridge.h"
#include <Adafruit_NeoPixel.h>

// CRSF Bridge instance
CRSFBridge crsfBridge;

// RX Configuration
#define CRSF_TX_PIN 21 
#define CRSF_RX_PIN 20   

// NeoPixel LED Configuration
// LED_PIN is defined in config.h
#define LED_COUNT 1            // Single LED
#define LED_BRIGHTNESS 255     // Brightness (0-255)

// LED Status Colors
#define COLOR_YELLOW 0xFFFF00   // Yellow
#define COLOR_ORANGE 0xFF6600   // Orange
#define COLOR_RED 0xFF0000      // Red
#define COLOR_GREEN 0x00FF00    // Green
#define COLOR_BLUE 0x0000FF     // Blue

Adafruit_NeoPixel led = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// LED state tracking
uint32_t currentColor = COLOR_RED;
unsigned long lastLedUpdateMs = 0;
const unsigned long LED_UPDATE_INTERVAL = 20;  // Update LED at 50Hz for smooth transitions

// Breathing effect parameters
const unsigned long BREATHING_CYCLE_TIME = 2000;  // 2 seconds for full breathing cycle
unsigned long breathingStartMs = 0;

#define ENABLE_CRSF_OUTPUT true
#define CRSF_OUTPUT_FREQUENCY_HZ 100

// Timing
unsigned long lastTelemetrySendMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastRxErrorMs = 0;
unsigned long lastCrsfUpdateMs = 0;

const unsigned long CRSF_OUTPUT_INTERVAL = 1000 / CRSF_OUTPUT_FREQUENCY_HZ;
const unsigned long TELEMETRY_SEND_INTERVAL = RX_TELEMETRY_SEND_INTERVAL_MS;
const unsigned long STATS_INTERVAL = RX_STATS_INTERVAL_MS;
const unsigned long ERROR_PRINT_INTERVAL = 1000;

float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
int16_t rssi = -45;
uint8_t linkQuality = 100;

// Helper function to blend colors smoothly
uint32_t blendColor(uint32_t color1, uint32_t color2, float ratio) {
  ratio = constrain(ratio, 0.0, 1.0);
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;
  
  uint8_t r = (uint8_t)(r1 + (r2 - r1) * ratio);
  uint8_t g = (uint8_t)(g1 + (g2 - g1) * ratio);
  uint8_t b = (uint8_t)(b1 + (b2 - b1) * ratio);
  
  return (r << 16) | (g << 8) | b;
}

// Update LED based on connection status
void updateLED() {
  unsigned long now = millis();
  bool linkActive = CustomProtocol_IsLinkActive();
  
  if (linkActive) {
    // Active connection: RGB breathing effect (red -> green -> blue -> red)
    unsigned long cycleTime = now % (BREATHING_CYCLE_TIME * 3);  // 3 colors = 3 cycles
    float colorPhase = (float)cycleTime / BREATHING_CYCLE_TIME;
    
    uint32_t color1, color2;
    float ratio;
    
    if (colorPhase < 1.0) {
      // Red to Green
      color1 = COLOR_RED;
      color2 = COLOR_GREEN;
      ratio = colorPhase;
    } else if (colorPhase < 2.0) {
      // Green to Blue
      color1 = COLOR_GREEN;
      color2 = COLOR_BLUE;
      ratio = colorPhase - 1.0;
    } else {
      // Blue to Red
      color1 = COLOR_BLUE;
      color2 = COLOR_RED;
      ratio = colorPhase - 2.0;
    }
    
    // Blend colors
    currentColor = blendColor(color1, color2, ratio);
    
    // Apply independent breathing effect (sine wave for smooth fade)
    float breathingPhase = (float)(now % BREATHING_CYCLE_TIME) / BREATHING_CYCLE_TIME;
    float breathing = (sin(breathingPhase * 2.0 * PI - PI/2.0) + 1.0) / 2.0;  // 0.0 to 1.0
    breathing = breathing * 0.4 + 0.6;  // Keep brightness between 60% and 100%
    
    // Apply breathing brightness
    uint8_t r = (uint8_t)(((currentColor >> 16) & 0xFF) * breathing);
    uint8_t g = (uint8_t)(((currentColor >> 8) & 0xFF) * breathing);
    uint8_t b = (uint8_t)((currentColor & 0xFF) * breathing);
    currentColor = (r << 16) | (g << 8) | b;
  } else {
    // No connection: Yellow -> Orange -> Red -> Orange -> Yellow breathing
    unsigned long cycleTime = now % (BREATHING_CYCLE_TIME * 2);  // Forward and back = 2 cycles
    float colorPhase = (float)cycleTime / BREATHING_CYCLE_TIME;
    
    uint32_t color1, color2;
    float ratio;
    
    if (colorPhase < 1.0) {
      // Yellow to Orange to Red (forward)
      if (colorPhase < 0.5) {
        // Yellow to Orange
        color1 = COLOR_YELLOW;
        color2 = COLOR_ORANGE;
        ratio = colorPhase * 2.0;
      } else {
        // Orange to Red
        color1 = COLOR_ORANGE;
        color2 = COLOR_RED;
        ratio = (colorPhase - 0.5) * 2.0;
      }
    } else {
      // Red to Orange to Yellow (backward)
      float backPhase = colorPhase - 1.0;
      if (backPhase < 0.5) {
        // Red to Orange
        color1 = COLOR_RED;
        color2 = COLOR_ORANGE;
        ratio = backPhase * 2.0;
      } else {
        // Orange to Yellow
        color1 = COLOR_ORANGE;
        color2 = COLOR_YELLOW;
        ratio = (backPhase - 0.5) * 2.0;
      }
    }
    
    // Blend colors
    currentColor = blendColor(color1, color2, ratio);
    
    // Apply independent breathing effect (sine wave for smooth fade)
    float breathingPhase = (float)(now % BREATHING_CYCLE_TIME) / BREATHING_CYCLE_TIME;
    float breathing = (sin(breathingPhase * 2.0 * PI - PI/2.0) + 1.0) / 2.0;  // 0.0 to 1.0
    breathing = breathing * 0.3 + 0.4;  // Keep brightness between 40% and 70%
    
    // Apply breathing brightness
    uint8_t r = (uint8_t)(((currentColor >> 16) & 0xFF) * breathing);
    uint8_t g = (uint8_t)(((currentColor >> 8) & 0xFF) * breathing);
    uint8_t b = (uint8_t)((currentColor & 0xFF) * breathing);
    currentColor = (r << 16) | (g << 8) | b;
  }
  
  // Update LED
  led.setPixelColor(0, currentColor);
  led.setBrightness(LED_BRIGHTNESS);
  led.show();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize NeoPixel LED
#if !ENABLE_CRSF_OUTPUT
  Serial.println("[LED] Initializing NeoPixel on GPIO 2...");
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure pin is ready
  delay(50);
  
  led.begin();
  led.clear();  // Clear all pixels
  led.setBrightness(LED_BRIGHTNESS);
  led.show();
  delay(100);
  
  // Test LED - flash red a few times to confirm it's working
#if !ENABLE_CRSF_OUTPUT
  Serial.println("[LED] Testing LED...");
#endif
  for (int i = 0; i < 3; i++) {
    led.setPixelColor(0, COLOR_RED);
    led.show();
    delay(300);
    led.setPixelColor(0, 0);  // Off
    led.show();
    delay(300);
  }
  led.setPixelColor(0, COLOR_RED);
  led.show();
#if !ENABLE_CRSF_OUTPUT
  Serial.println("[LED] NeoPixel initialized (RED = Starting)");
#endif
  
#if !ENABLE_CRSF_OUTPUT
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Receiver (RX)");
  Serial.println("=================================");
  Serial.println("DEBUG MODE - CRSF Output Disabled\n");
#endif
  
  if (!CustomProtocol_Init(false, nullptr)) {
#if !ENABLE_CRSF_OUTPUT
    Serial.println("[RX] Protocol init failed!");
#endif
    while (1) {
      delay(1000);
    }
  }
  
#if !ENABLE_CRSF_OUTPUT
  Serial.println("[RX] Protocol initialized successfully");
  Serial.flush();
#endif
  delay(200);
  
#if ENABLE_CRSF_OUTPUT
  crsfBridge.init(&Serial1, CRSF_TX_PIN, CRSF_RX_PIN);
  delay(100);
#else
  Serial.println("[RX] CRSF output DISABLED - Debug mode active");
  Serial.println("[RX] Ready to receive!");
  Serial.println("[RX] Listening for any transmitter (broadcast mode)");
  Serial.printf("[RX] Test frequency: %d Hz\n", CRSF_OUTPUT_FREQUENCY_HZ);
  Serial.println();
  Serial.flush();
#endif
  
  breathingStartMs = millis();
  delay(100);
}

void loop() {
  unsigned long now = millis();
  
  CustomProtocol_Update();
  
  // Update LED continuously for smooth transitions
  if (now - lastLedUpdateMs >= LED_UPDATE_INTERVAL) {
    lastLedUpdateMs = now;
    updateLED();
  }
  
#if ENABLE_CRSF_OUTPUT
  crsfBridge.update();
  
  if (now - lastCrsfUpdateMs >= CRSF_OUTPUT_INTERVAL) {
    lastCrsfUpdateMs = now;
    
    if (crsfBridge.isActive()) {
      if (CustomProtocol_IsLinkActive()) {
        uint16_t channels[16];
        CustomProtocol_GetRcChannels(channels);
        crsfBridge.sendRcChannels(channels);
      } else {
        uint16_t failsafeChannels[16] = {
          992, 992, 172, 992, 172, 172, 172, 172,
          172, 172, 172, 172, 172, 172, 172, 172
        };
        crsfBridge.sendRcChannels(failsafeChannels);
      }
    }
  }
#endif
  
  if (CustomProtocol_IsLinkActive() && (now - lastTelemetrySendMs >= TELEMETRY_SEND_INTERVAL)) {
    lastTelemetrySendMs = now;
    
    batteryVoltage = 0.0;
    batteryCurrent = 0.0;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    linkQuality = (stats.linkActive) ? 100 : 0;
    
    if (!CustomProtocol_SendTelemetry(batteryVoltage, batteryCurrent, rssi, linkQuality)) {
#if !ENABLE_CRSF_OUTPUT
      if (now - lastRxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[RX] Failed to send telemetry");
        lastRxErrorMs = now;
      }
#endif
    }
  }
  
#if !ENABLE_CRSF_OUTPUT
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- RX Statistics ---");
    Serial.printf("Packets RX: %lu | TX: %lu | Loss: %.1f%% | CRC Err: %lu\n", 
                  stats.packetsReceived, stats.packetsSent, 
                  stats.packetLossPercent, stats.crcErrors);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "FAILSAFE", stats.lastPacketMs);
    
    if (stats.linkActive) {
      uint16_t channels[16];
      CustomProtocol_GetRcChannels(channels);
      
      Serial.printf("RX Received RC: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d\n",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5]);
      Serial.printf("All 16 channels: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5], channels[6], channels[7],
                    channels[8], channels[9], channels[10], channels[11],
                    channels[12], channels[13], channels[14], channels[15]);
    } else {
      Serial.println("*** FAILSAFE MODE - No signal ***");
    }
    
    Serial.println();
  }
#endif
  
  delay(1);
}

#endif // BUILD_RX
