import socket
import threading
from typing import Callable


class TCPClient:
    def __init__(self, target_ip: str, target_port: int):
        self.target_ip = target_ip
        self.target_port = target_port

    def send(self, message: str, timeout=5) -> str:
        try:
            with socket.create_connection((self.target_ip, self.target_port), timeout=timeout) as sock:
                sock.sendall(message.encode('utf-8'))
                return sock.recv(1024).decode('utf-8').strip()
        except Exception as e:
            return f"[Error] {e}"


class TCPServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 9000, handler: Callable[[str, tuple], str] = None):
        self.host = host
        self.port = port
        self.handler = handler
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)

    def start(self):
        self.server_thread.start()

    def _run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(5)
            print(f"[TCPServer] Listening on {self.host}:{self.port}")
            while True:
                conn, addr = server.accept()
                with conn:
                    try:
                        data = conn.recv(1024).decode('utf-8').strip()
                        print(f"[TCPServer] Received from {addr}: {data}")
                        if self.handler:
                            response = self.handler(data, addr)
                            conn.sendall(response.encode('utf-8'))
                        else:
                            conn.sendall(b'ok')
                    except Exception as e:
                        print(f"[TCPServer] Error: {e}")
