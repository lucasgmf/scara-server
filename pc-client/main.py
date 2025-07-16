import subprocess
import sys

if __name__ == "__main__":
    # Check if IP address is provided as command line argument
    if len(sys.argv) > 1:
        esp32_ip = sys.argv[1]
        print(f"Using ESP32 IP: {esp32_ip}")
    else:
        print("Usage: python main.py <ESP32_IP>")
        print("Example: python main.py 192.168.0.1")
        sys.exit(1)
    
    try:
        print("Starting Flask server...")
        # Pass the IP address as an environment variable or command line argument
        subprocess.run(["python", "web_page.py", esp32_ip], check=True)
    except KeyboardInterrupt:
        print("\nServer stopped manually.")
    except Exception as e:
        print(f"Error: {e}")
