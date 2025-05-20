from flask import Flask, request, jsonify, render_template
import socket

ESP32_IP = "192.168.225.67"
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
        numbers = [request.form.get(f"num{i}", "").strip() for i in range(1, 6)]
        
        if not all(num.replace("-", "").replace(".", "", 1).isdigit() for num in numbers):
            return "Invalid input", 400
        
        data = ",".join(numbers) + "\n"
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
