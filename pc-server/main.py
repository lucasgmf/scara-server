import socket

def start_server(host='0.0.0.0', port=42424):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")

    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")
        
        data = client_socket.recv(1024).decode('utf-8')
        if data:
            print(f"Data received: {data}")
        
        client_socket.close()

if __name__ == "__main__":
    start_server()

