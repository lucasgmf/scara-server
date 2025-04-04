from flask import Flask, render_template, request
import socket

ESP32_IP = "192.168.101.148"  # Replace with your ESP32's IP
ESP32_PORT = 42424

app = Flask(__name__)

def send_to_esp32(data):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((ESP32_IP, ESP32_PORT))
            client_socket.sendall(data.encode())
            print(f"Sent: {data}")
    except Exception as e:
        print(f"Error: {e}")

@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        numbers = [request.form[f"num{i}"] for i in range(1, 6)]
        data = ",".join(numbers) + "\n"  # Send numbers as CSV
        send_to_esp32(data)
        return "Numbers sent!"
    
    return render_template("index.html")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)

