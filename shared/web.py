"""Serve a browser window's page on the same port as its WebSocket.

Both programs have one window each (simulator window, control window). Each runs one
`websockets` server; this module answers a plain HTTP GET on that server with the page
file, so no second web server is needed. Besides the page it serves the two files both
windows share: the 3D models (`models.js`) and the style sheet (`style.css`).
"""

from http import HTTPStatus
from pathlib import Path

from websockets.asyncio.server import ServerConnection
from websockets.datastructures import Headers
from websockets.http11 import Request, Response

SHARED_DIR = Path(__file__).parent / "web"
# URL path -> (file, content type) of the files both windows load.
SHARED_FILES = {
    "/models.js": (SHARED_DIR / "models.js", "text/javascript"),
    "/style.css": (SHARED_DIR / "style.css", "text/css"),
}


def page_response(
    connection: ServerConnection, request: Request, page_path: Path
) -> Response | None:
    """Answer a plain HTTP GET with the page or a shared file; let WebSocket upgrades through.

    Use it from the `process_request` hook of `websockets.asyncio.server.serve`.

    Args:
        connection: The connection (used to build the response).
        request: The HTTP request.
        page_path: The HTML file of the window.

    Returns:
        A response for a plain GET (404 if the file is missing), None for a WebSocket
        upgrade. The paths in `SHARED_FILES` give those files, every other path the page.
    """
    if "Upgrade" in request.headers:
        return None
    file_path, content_type = SHARED_FILES.get(request.path, (page_path, "text/html"))
    if not file_path.is_file():
        return connection.respond(HTTPStatus.NOT_FOUND, "Page not found\n")
    body = file_path.read_bytes()
    headers = Headers(
        [("Content-Type", f"{content_type}; charset=utf-8"), ("Content-Length", str(len(body)))]
    )
    return Response(HTTPStatus.OK.value, HTTPStatus.OK.phrase, headers, body)
