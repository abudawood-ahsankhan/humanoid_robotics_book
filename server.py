import http.server
import socketserver
import os
from pathlib import Path

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory="E:\\Quarter_4\\Hackathon_1\\humanoid-robotics-book", **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        super().end_headers()

def start_server(port=3000):
    """Start a simple HTTP server to serve the documentation"""
    handler = CustomHTTPRequestHandler

    with socketserver.TCPServer(("", port), handler) as httpd:
        print(f"Server started at http://localhost:{port}")
        print("Serving documentation for Humanoid Robotics Book")
        print("Press Ctrl+C to stop the server")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    start_server(3000)