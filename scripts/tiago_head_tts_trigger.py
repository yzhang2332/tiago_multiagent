import socket
import threading

HEAD_IP = "10.68.0.207"
PORT_HEAD = 9091          # Port to send commands to tiago head
LAPTOP_LISTEN_PORT = 9092  # Port to receive messages like "finished" from tiago head

def send_to_robot_b(message: str):
    try:
        with socket.create_connection((HEAD_IP, PORT_HEAD), timeout=5) as sock:
            sock.sendall(message.encode())
            response = sock.recv(1024).decode()
            print(f"[Robot B responded] → {response}")
    except Exception as e:
        print(f"[Error] Could not send message to Robot B: {e}")

def listen_for_response():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', LAPTOP_LISTEN_PORT))
    server.listen(1)
    print(f"[Listening] Laptop waiting for response on port {LAPTOP_LISTEN_PORT}...")

    while True:
        conn, addr = server.accept()
        data = conn.recv(1024).decode().strip()
        if data:
            print(f"[Robot B → Laptop] Received from {addr}: {data}")
        conn.close()

if __name__ == '__main__':
    # Start the listener in a background thread
    listener_thread = threading.Thread(target=listen_for_response, daemon=True)
    listener_thread.start()

    # Send a predefined message to Robot B
    send_to_robot_b("high_explicit_correct_1_1")

    # Keep the main thread alive to receive responses
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\n[Stopped] Listener shut down.")
