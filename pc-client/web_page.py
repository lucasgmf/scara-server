import socket
import threading
import time
import json
import sys
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit

# ESP32 Configuration - Get from command line arguments
if len(sys.argv) > 1:
    ESP32_IP = sys.argv[1]
else:
    print("Error: No IP address provided")
    print("This script should be called with an IP address argument")
    sys.exit(1)

ESP32_PORT = 42424

# Flask App Setup
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key-change-this'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
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
            # Close existing connection
            if self.socket:
                self.socket.close()
            
            # Create new socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((ESP32_IP, ESP32_PORT))
            
            self.connected = True
            self.should_stop = False
            
            # Start receiving thread
            if self.receive_thread and self.receive_thread.is_alive():
                self.should_stop = True
                self.receive_thread.join(timeout=2)
            
            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()
            
            print(f"[✓] Connected to ESP32 at {ESP32_IP}:{ESP32_PORT}")
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
                    
                    print(f"[DEBUG] Received: {line}")
                    
                    # Parse CSV data (expecting 17 values)
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
                                'switch0': bool(int(values[6])),
                                'switch1': bool(int(values[7])),
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
                                'connected': True
                            }
                            
                            with connection_lock:
                                latest_esp32_data = parsed_data
                            
                            # Emit to all connected web clients
                            socketio.emit('monitor_update', parsed_data)
                            
                        except (ValueError, IndexError) as e:
                            print(f"[!] Error parsing data: {e}")
                    else:
                        print(f"[!] Invalid data format: expected 17 values, got {len(values)}")
                        
            except socket.timeout:
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

@app.route("/")
def serve_form():
    return render_template("index.html")

@app.route("/send_data", methods=["POST"])
def send_data():
    """Send data to ESP32"""
    try:
        if not ensure_connection():
            return jsonify({"success": False, "error": "Failed to connect to ESP32"}), 500
        
        data = request.get_json()
        if not data:
            return jsonify({"success": False, "error": "No data provided"}), 400
        
        # Get boolean values
        bool1 = data.get("bool1", False)
        bool2 = data.get("bool2", False)
        
        # Get float values
        floats = []
        for i in range(1, 10):  # float1 to float9
            float_val = data.get(f"float{i}")
            if float_val is None:
                return jsonify({"success": False, "error": f"Missing float{i}"}), 400
            
            try:
                floats.append(str(float(float_val)))
            except (ValueError, TypeError):
                return jsonify({"success": False, "error": f"Invalid float{i}: {float_val}"}), 400
        
        # Format: float1,float2,...,float9,bool1,bool2
        message = ",".join(floats) + f",{int(bool1)},{int(bool2)}\n"
        
        success = esp32_conn.send_data(message)
        
        if success:
            return jsonify({"success": True, "message": "Data sent successfully"})
        else:
            return jsonify({"success": False, "error": "Failed to send data"}), 500
            
    except Exception as e:
        print(f"[ERROR] Send data error: {e}")
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/api/esp32_data")
def get_esp32_data():
    """Get latest ESP32 data"""
    with connection_lock:
        return jsonify(latest_esp32_data)

@app.route("/api/connection_status")
def get_connection_status():
    """Get ESP32 connection status"""
    return jsonify({
        'connected': esp32_conn.connected,
        'ip': ESP32_IP,
        'port': ESP32_PORT
    })

@app.route("/api/connect", methods=["POST"])
def connect_esp32():
    """Manually connect to ESP32"""
    success = esp32_conn.connect()
    return jsonify({'success': success, 'connected': esp32_conn.connected})

@app.route("/api/disconnect", methods=["POST"])
def disconnect_esp32():
    """Manually disconnect from ESP32"""
    esp32_conn.disconnect()
    return jsonify({'success': True, 'connected': esp32_conn.connected})

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    print('[✓] Client connected to WebSocket')
    emit('connection_status', {'connected': esp32_conn.connected})
    
    # Send latest data if available
    with connection_lock:
        if latest_esp32_data and latest_esp32_data.get('connected', False):
            emit('monitor_update', latest_esp32_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print('[!] Client disconnected from WebSocket')

@socketio.on('request_data')
def handle_request_data():
    """Handle request for current ESP32 data"""
    with connection_lock:
        if latest_esp32_data:
            emit('monitor_update', latest_esp32_data)

def startup_connection():
    """Attempt to connect to ESP32 on startup"""
    time.sleep(2)  # Give Flask time to start
    if ensure_connection():
        print("[✓] Initial connection to ESP32 established")
    else:
        print("[!] Initial connection failed - will retry on first request")

def connection_monitor():
    """Monitor connection and attempt reconnection"""
    while True:
        time.sleep(30)  # Check every 30 seconds
        if not esp32_conn.connected:
            print("[!] ESP32 disconnected, attempting reconnection...")
            esp32_conn.connect()

if __name__ == "__main__":
    # Start background threads
    startup_thread = threading.Thread(target=startup_connection, daemon=True)
    startup_thread.start()
    
    monitor_thread = threading.Thread(target=connection_monitor, daemon=True)
    monitor_thread.start()
    
    # Run the app
    print(f"[✓] Starting Flask server...")
    print(f"[✓] Will connect to ESP32 at {ESP32_IP}:{ESP32_PORT}")
    socketio.run(app, host="0.0.0.0", port=5000, debug=True, allow_unsafe_werkzeug=True)
