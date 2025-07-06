from flask import Flask, request, jsonify, render_template
import socket

ESP32_IP = "192.168.85.192"
ESP32_PORT = 42424
app = Flask(__name__)

def send_to_esp32(data):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.settimeout(5)
            client_socket.connect((ESP32_IP, ESP32_PORT))
            client_socket.sendall(data.encode())
            print(f"[✓] Sent to ESP32: {data}")
            return True
    except Exception as e:
        print(f"[✗] Error: {e}")
        return False

@app.route("/", methods=["POST"])
def receive_data():
    try:
        # Get boolean values
        bool1 = request.form.get("bool1") == "true"
        bool2 = request.form.get("bool2") == "true"
        
        # Get float values
        floats = []
        for i in range(1, 9):  # 8 floats
            float_val = request.form.get(f"float{i}", "").strip()
            if not float_val.replace("-", "").replace(".", "", 1).isdigit():
                return f"Invalid input for float{i}", 400
            floats.append(float_val)
        
        # Format: float1,float2,...,float8,bool1,bool2 (floats first, then booleans)
        data = ",".join(floats) + f",{int(bool1)},{int(bool2)}\n"
        success = send_to_esp32(data)
        
        if success:
            return "Values sent successfully!"
        else:
            return "Failed to send data to ESP32.", 500
    except Exception as e:
        return f"Server error: {e}", 500

@app.route("/", methods=["GET"])
def serve_form():
    return render_template("index.html")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
