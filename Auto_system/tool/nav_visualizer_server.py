from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PORT = 8765


def main() -> None:
    tool_dir = Path(__file__).resolve().parent

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(tool_dir), **kwargs)

    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Navigation visualizer: http://127.0.0.1:{PORT}/nav_visualizer.html")
    server.serve_forever()


if __name__ == "__main__":
    main()
