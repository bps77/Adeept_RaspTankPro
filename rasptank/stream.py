"""Opt-in loopback HTTP video presentation; capture is always borrowed."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import secrets
import threading

from .hardware._lifecycle import ManagedController, number


class CameraStream(ManagedController):
    """Serve a token-scoped video page on 127.0.0.1, never a motor-control endpoint.

    The page polls fresh JPEGs through an SSH tunnel. No camera, browser, widget,
    or public network listener is opened implicitly. Closing the page does not
    stop motors or close the borrowed camera; close this handle to release HTTP.
    """

    def __init__(self, camera, *, port=8765, fps=10, transform=None,
                 on_disconnect=None):
        """Bind loopback and start serving; port=0 chooses an available port.

        camera must already be started and implement capture(timeout=...).
        transform optionally maps a Frame to nonempty JPEG bytes. Capture or
        transform failures return HTTP 503 and invoke on_disconnect if supplied.
        No callback runs merely because a browser disconnects.
        """
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port must be an integer in 0..65535")
        number(fps, 1, 30, "fps")
        if transform is not None and not callable(transform):
            raise ValueError("transform must be callable")
        if on_disconnect is not None and not callable(on_disconnect):
            raise ValueError("on_disconnect must be callable")
        self._closed = False
        self._error = None
        self._lock = threading.Lock()
        token = secrets.token_urlsafe(24)
        path = "/" + token + "/"
        stream = self
        page = ("""<!doctype html><meta charset="utf-8"><title>Tank camera</title>
<style>body{background:#111;color:white;font-family:sans-serif}img{max-width:100%}</style>
<h1>Tank camera</h1><p>Video only. Closing this page does not stop the tank.
Use tank.stop() in the notebook.</p>
<img id="video" alt="Tank camera"><p id="status" role="status">Connecting</p>
<script>
const image=document.getElementById("video"), status=document.getElementById("status");
let active=true, current=null;
window.addEventListener("pagehide",()=>{
  active=false; if(current) URL.revokeObjectURL(current);
});
async function update(){
  try{
    const response=await fetch("frame.jpg",{cache:"no-store"});
    if(!response.ok) throw new Error(await response.text());
    const blob=await response.blob();
    if(!active) return;
    const next=URL.createObjectURL(blob);
    image.src=next; image.hidden=false;
    if(current) URL.revokeObjectURL(current);
    current=next; status.textContent="Live video";
  }catch(error){
    image.hidden=true; status.textContent="Video error: "+error.message;
  }
  if(active) setTimeout(update,INTERVAL);
}
update();
</script>""").replace("INTERVAL", str(1000 / fps)).encode("utf-8")

        class Handler(BaseHTTPRequestHandler):
            """Serve only the token-scoped page and fresh JPEG route."""

            def setup(self):
                """Bound socket writes so a stalled browser cannot hold a worker."""
                super().setup()
                self.connection.settimeout(2)

            def do_GET(self):
                """Return video page, fresh JPEG, or explicit HTTP failure."""
                if self.path not in (path, path + "frame.jpg"):
                    self._respond(404, "text/plain", b"Not found")
                    return
                if self.path == path:
                    self._respond(200, "text/html; charset=utf-8", page)
                    return
                try:
                    with stream._lock:
                        if stream._closed:
                            raise RuntimeError("Video server closed")
                        frame = camera.capture(timeout=1)
                        jpeg = transform(frame) if transform else frame.jpeg
                        if not isinstance(jpeg, bytes) or not jpeg:
                            raise ValueError("stream transform must return nonempty JPEG bytes")
                        stream._error = None
                except Exception as exc:
                    error = f"{type(exc).__name__}: {exc}"
                    if on_disconnect is not None:
                        try:
                            on_disconnect()
                        except Exception as stop_error:
                            error += f"; motor stop failed: {stop_error}"
                    stream._error = error
                    self._respond(503, "text/plain; charset=utf-8", error.encode("utf-8"))
                    return
                self._respond(200, "image/jpeg", jpeg)

            def _respond(self, code, content_type, body):
                """Send an uncached response; browser disconnects are expected."""
                try:
                    self.send_response(code)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Referrer-Policy", "no-referrer")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.end_headers()
                    self.wfile.write(body)
                except (ConnectionError, TimeoutError):
                    return

            def log_message(self, format, *args):
                """Avoid logging the private access token in request paths."""

        self._server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.url = f"http://127.0.0.1:{self._server.server_port}{path}"
        self._worker = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": 0.1}, daemon=True,
                                        name="rasptank-video-http")
        try:
            self._worker.start()
        except BaseException:
            self._server.server_close()
            raise

    def status(self):
        """Return server running state and most recent capture/transform error."""
        return dict(running=not self._closed and self._worker.is_alive(),
                    error=self._error)

    def close(self):
        """Stop the HTTP listener idempotently; never close the borrowed camera."""
        if self._closed:
            return
        self._closed = True
        try:
            self._server.shutdown()
        finally:
            self._server.server_close()
            self._worker.join(1)
