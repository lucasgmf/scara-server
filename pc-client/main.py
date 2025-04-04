import socket
import time
import random

ESP32_IP = "192.168.101.148"  # Replace with your ESP32's IP
ESP32_PORT = 42424

def send_random_numbers():
    try:
        # Create a TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((ESP32_IP, ESP32_PORT))
        print(f"Connected to ESP32 at {ESP32_IP}:{ESP32_PORT}")

        while True:
            # Generate a random number
            number = random.randint(0, 100)
            message = f"{number}\n"

            # Send the number to the ESP32
            client_socket.sendall(message.encode())
            print(f"Sent: {number}")

            # Wait for 1 second
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nClient stopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    send_random_numbers()
