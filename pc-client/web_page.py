import socket
import threading
import time
import json
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit

ESP32_IP = "192.168.85.148"
ESP32_PORT = 42424

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables for connection management
esp32_socket = None
esp32_connected = False
latest_esp32_data = {'connected': False}
connection_lock = threading.Lock()

class ESP32Connection:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.receive_thread = None
        self.should_stop = False
        
    def connect(self):
        """Establish connection to ESP32"""
        try:
            if self.socket:
                self.socket.close()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)  # Increased timeout
            self.socket.connect((ESP32_IP, ESP32_PORT))
            self.connected = True
            self.should_stop = False
            
            # Start receiving thread
            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()
            
            print(f"[✓] Connected to ESP32 at {ESP32_IP}:{ESP32_PORT}")
            
            # Notify all clients about connection status
            socketio.emit('connection_status', {'connected': True})
            
            return True
        except Exception as e:
            print(f"[✗] Connection failed: {e}")
            self.connected = False
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            
            # Notify all clients about connection status
            socketio.emit('connection_status', {'connected': False})
            return False
    
    def disconnect(self):
        """Disconnect from ESP32"""
        self.should_stop = True
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        print("[!] Disconnected from ESP32")
        
        # Notify all clients about disconnection
        socketio.emit('connection_status', {'connected': False})
    
    def send_data(self, data):
        """Send data to ESP32"""
        if not self.connected or not self.socket:
            print("[!] Not connected to ESP32")
            return False
        
        try:
            self.socket.sendall(data.encode())
            print(f"[✓] Sent to ESP32: {data.strip()}")
            return True
        except Exception as e:
            print(f"[✗] Send error: {e}")
            self.disconnect()
            return False
    
    def receive_data(self):
        """Continuously receive data from ESP32"""
        global latest_esp32_data
        buffer = ""
        
        while not self.should_stop and self.connected:
            try:
                if not self.socket:
                    break
                
                # Set socket timeout for receiving
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024).decode()
                
                if not data:
                    print("[!] ESP32 disconnected - no data received")
                    break
                
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line:
                        continue
                    
                    print(f"[DEBUG] Received raw line: {line}")
                    
                    # Parse CSV data
                    values = line.split(',')
                    if len(values) >= 17:
                        try:
                            parsed_data = {
                                'horizontal_load': float(values[0]),
                                'vertical_load': float(values[1]),
                                'encoder0': float(values[2]),
                                'encoder1': float(values[3]),
                                'encoder2': float(values[4]),
                                'encoder3': float(values[5]),
                                'switch0': int(values[6]),
                                'switch1': int(values[7]),
                                'x': float(values[8]),
                                'y': float(values[9]),
                                'z': float(values[10]),
                                'w': float(values[11]),
                                'd1': float(values[12]),
                                'theta2': float(values[13]),
                                'theta3': float(values[14]),
                                'theta4': float(values[15]),
                                'theta5': float(values[16]),
                                'timestamp': time.time(),
                                'connected': True,
                                # Format data for your existing HTML template
                                'floats': [
                                    float(values[0]),  # horizontal_load
                                    float(values[1]),  # vertical_load
                                    float(values[2]),  # encoder0
                                    float(values[3]),  # encoder1
                                    float(values[4]),  # encoder2
                                    float(values[5]),  # encoder3
                                    float(values[8]),  # x
                                    float(values[9]),  # y
                                    float(values[10]) # z (9th float)
                                ],
                                'booleans': [
                                    bool(int(values[6])),  # switch0
                                    bool(int(values[7]))   # switch1
                                ]
                            }
                            
                            with connection_lock:
                                latest_esp32_data = parsed_data
                            
                            # Emit to all connected web clients
                            print(f"[DEBUG] Emitting monitor_update to clients")
                            socketio.emit('monitor_update', parsed_data)
                            
                            print(f"[✓] Received from ESP32: Load H={parsed_data['horizontal_load']:.2f}, V={parsed_data['vertical_load']:.2f}")
                        except (ValueError, IndexError) as e:
                            print(f"[!] Error parsing values: {e}")
                            print(f"[!] Raw data: {line}")
                    else:
                        print(f"[!] Invalid data format (expected 17 values, got {len(values)}): {line}")
                        
            except socket.timeout:
                # This is normal - just continue
                continue
            except Exception as e:
                print(f"[✗] Receive error: {e}")
                break
        
        print("[!] Receive thread ending")
        self.disconnect()

# Global ESP32 connection instance
esp32_conn = ESP32Connection()

def ensure_connection():
    """Ensure ESP32 connection is active"""
    if not esp32_conn.connected:
        print("[!] Attempting to reconnect to ESP32...")
        return esp32_conn.connect()
    return True

@app.route("/", methods=["GET"])
def serve_form():
    return render_template("index.html")

@app.route("/", methods=["POST"])
def receive_data():
    try:
        if not ensure_connection():
            return "Failed to connect to ESP32", 500
        
        # Get boolean values
        bool1 = request.form.get("bool1") == "true"
        bool2 = request.form.get("bool2") == "true"
        
        # Get float values - FIXED: Now includes all 9 floats
        floats = []
        for i in range(1, 10):  # 1 to 9 inclusive
            float_val = request.form.get(f"float{i}", "").strip()
            if not float_val:
                return f"Missing value for float{i}", 400
            
            # Better validation for float values
            try:
                float_value = float(float_val)
                floats.append(str(float_value))
            except ValueError:
                return f"Invalid input for float{i}: '{float_val}' is not a valid number", 400
        
        # Format: float1,float2,...,float9,bool1,bool2
        data = ",".join(floats) + f",{int(bool1)},{int(bool2)}\n"
        print(f"[DEBUG] Sending data to ESP32: {data.strip()}")
        
        success = esp32_conn.send_data(data)
        
        if success:
            return "Values sent successfully!"
        else:
            return "Failed to send data to ESP32.", 500
    except Exception as e:
        print(f"[ERROR] Server error in receive_data: {e}")
        return f"Server error: {e}", 500

@app.route("/api/esp32_data", methods=["GET"])
def get_esp32_data():
    """API endpoint to get latest ESP32 data"""
    with connection_lock:
        return jsonify(latest_esp32_data)

@app.route("/api/connection_status", methods=["GET"])
def get_connection_status():
    """API endpoint to check ESP32 connection status"""
    return jsonify({
        'connected': esp32_conn.connected,
        'ip': ESP32_IP,
        'port': ESP32_PORT
    })

@app.route("/api/connect", methods=["POST"])
def connect_esp32():
    """API endpoint to manually connect to ESP32"""
    success = esp32_conn.connect()
    return jsonify({'success': success, 'connected': esp32_conn.connected})

@app.route("/api/disconnect", methods=["POST"])
def disconnect_esp32():
    """API endpoint to manually disconnect from ESP32"""
    esp32_conn.disconnect()
    return jsonify({'success': True, 'connected': esp32_conn.connected})

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection from client"""
    print('[✓] Client connected to WebSocket')
    emit('connection_status', {'connected': esp32_conn.connected})
    
    # Send latest data if available
    with connection_lock:
        if latest_esp32_data and 'connected' in latest_esp32_data:
            print(f"[DEBUG] Sending latest data to new client: {latest_esp32_data.get('connected', False)}")
            emit('monitor_update', latest_esp32_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print('[!] Client disconnected from WebSocket')

@socketio.on('request_data')
def handle_request_data():
    """Handle request for current ESP32 data"""
    print("[DEBUG] Client requested current data")
    with connection_lock:
        if latest_esp32_data:
            emit('monitor_update', latest_esp32_data)

# Auto-connect on startup
def startup_connection():
    """Attempt to connect to ESP32 on startup"""
    time.sleep(2)  # Give Flask time to start
    if ensure_connection():
        print("[✓] Initial connection to ESP32 established")
    else:
        print("[!] Initial connection to ESP32 failed - will retry on first request")

# Periodic connection check
def connection_monitor():
    """Monitor connection status and attempt reconnection"""
    while True:
        time.sleep(30)  # Check every 30 seconds
        if not esp32_conn.connected:
            print("[!] ESP32 disconnected, attempting reconnection...")
            esp32_conn.connect()

if __name__ == "__main__":
    # Start connection in background
    startup_thread = threading.Thread(target=startup_connection, daemon=True)
    startup_thread.start()
    
    # Start connection monitor
    monitor_thread = threading.Thread(target=connection_monitor, daemon=True)
    monitor_thread.start()
    
    # Run Flask app with SocketIO
    socketio.run(app, host="0.0.0.0", port=5000, debug=True, allow_unsafe_werkzeug=True)
