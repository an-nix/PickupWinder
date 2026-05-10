import re
import sys

old = """    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        sock.connect(str(self.socket_path))
        sock.settimeout(1.0)
        self._sock = sock
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name="wendy_rpc_reader",
        )
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

    def send_raw(self, raw_payload: str, timeout_s: float | None = None) -> dict[str, Any] | None:
        if self._sock is None:
            self.connect()
        request = parse_json_rpc(raw_payload)
        request_id = request.get("id")
        if not raw_payload.endswith("\\n"):
            raw_payload = raw_payload + "\\n"

        if request_id is None:
            self._send(raw_payload)
            return None

        event = threading.Event()
        with self._lock:
            self._pending[request_id] = (event, None)
            self._send(raw_payload)

        deadline = time.monotonic() + (timeout_s if timeout_s is not None else self.timeout_s)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if event.wait(timeout=remaining):
                with self._lock:
                    _, response = self._pending.pop(request_id, (None, None))
                if response is None:
                    raise RuntimeError(f"Missing response for request id {request_id}")
                return response

        with self._lock:
            self._pending.pop(request_id, None)
        raise RuntimeError(f"JSON-RPC request timeout for id {request_id}")

    def _send(self, raw_payload: str) -> None:
        if self._sock is None:
            raise RuntimeError("RPC client is not connected")
        self._sock.sendall(raw_payload.encode("utf-8"))

    def _reader_loop(self) -> None:
        assert self._sock is not None
        while not self._stop_event.is_set():
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            self._buffer += chunk.decode("utf-8", errors="replace")
            while "\\n" in self._buffer:
                line, self._buffer = self._buffer.split("\\n", 1)
                if not line.strip():
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._handle_incoming(message)"""

new = """    def connect(self) -> None:
        if self._reader_thread is not None:
            return
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._connection_loop,
            daemon=True,
            name="wendy_rpc_client",
        )
        self._reader_thread.start()

    def _connection_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(self.timeout_s)
                sock.connect(str(self.socket_path))
                sock.settimeout(1.0)
                
                with self._lock:
                    self._sock = sock
                logger.info("Connected to RPC backend %s", self.socket_path)
            except OSError:
                time.sleep(1.0)
                continue
                
            self._reader_loop(sock)
            
            with self._lock:
                if self._sock is sock:
                    self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
            
            if not self._stop_event.is_set():
                logger.warning("Disconnected from RPC backend, retrying in 1s")
                time.sleep(1.0)

    def close(self) -> None:
        self._stop_event.set()
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._sock.close()
                self._sock = None
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

    def send_raw(self, raw_payload: str, timeout_s: float | None = None) -> dict[str, Any] | None:
        # Wait up to timeout to ensure we are connected
        deadline = time.monotonic() + (timeout_s if timeout_s is not None else self.timeout_s)
        while True:
            with self._lock:
                if self._sock is not None:
                    break
            if time.monotonic() > deadline:
                raise RuntimeError("RPC client is not connected")
            time.sleep(0.1)

        request = parse_json_rpc(raw_payload)
        request_id = request.get("id")
        if not raw_payload.endswith("\\n"):
            raw_payload = raw_payload + "\\n"

        if request_id is None:
            self._send(raw_payload)
            return None

        event = threading.Event()
        with self._lock:
            self._pending[request_id] = (event, None)
            self._send(raw_payload)

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if event.wait(timeout=remaining):
                with self._lock:
                    _, response = self._pending.pop(request_id, (None, None))
                if response is None:
                    raise RuntimeError(f"Missing response for request id {request_id}")
                return response

        with self._lock:
            self._pending.pop(request_id, None)
        raise RuntimeError(f"JSON-RPC request timeout for id {request_id}")

    def _send(self, raw_payload: str) -> None:
        with self._lock:
            sock = self._sock
        if sock is None:
            raise RuntimeError("RPC client is not connected")
        try:
            sock.sendall(raw_payload.encode("utf-8"))
        except OSError as e:
            with self._lock:
                if self._sock is sock:
                    self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
            raise RuntimeError(f"Send failed: {e}") from e

    def _reader_loop(self, sock: socket.socket) -> None:
        self._buffer = ""
        while not self._stop_event.is_set():
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            self._buffer += chunk.decode("utf-8", errors="replace")
            while "\\n" in self._buffer:
                line, self._buffer = self._buffer.split("\\n", 1)
                if not line.strip():
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._handle_incoming(message)"""

content = open("src/wendy/rpc.py").read()
if old in content:
    content = content.replace(old, new)
    content = "import logging\n\nlogger = logging.getLogger(__name__)\n\n" + content
    open("src/wendy/rpc.py", "w").write(content)
    print("PATCH OK")
else:
    print("NO MATCH")
