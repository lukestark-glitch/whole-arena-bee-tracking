"""Serve a browser window's page on the same port as its WebSocket.

Both programs have one window each (simulator window, control window). Each runs one
`websockets` server; this module answers a plain HTTP GET on that server with the page
file, so no second web server is needed.
"""

from http import HTTPStatus
from pathlib import Path

from websockets.asyncio.server import ServerConnection
from websockets.datastructures import Headers
from websockets.http11 import Request, Response


def page_response(
    connection: ServerConnection, request: Request, page_path: Path
) -> Response | None:
    """Answer a plain HTTP GET with the page; let WebSocket upgrades through.

    Use it from the `process_request` hook of `websockets.asyncio.server.serve`.

    Args:
        connection: The connection (used to build the response).
        request: The HTTP request.
        page_path: The HTML file of the window.

    Returns:
        A response for a plain GET (404 if the file is missing), None for a WebSocket
        upgrade.
    """
    if "Upgrade" in request.headers:
        return None
    if not page_path.is_file():
        return connection.respond(HTTPStatus.NOT_FOUND, "Page not found\n")
    body = page_path.read_bytes()
    headers = Headers(
        [("Content-Type", "text/html; charset=utf-8"), ("Content-Length", str(len(body)))]
    )
    return Response(HTTPStatus.OK.value, HTTPStatus.OK.phrase, headers, body)
