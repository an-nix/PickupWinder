from __future__ import annotations

from jsonrpc import UnixJsonRpcClient


def main() -> None:
    socket_path = "/tmp/pickup_winder_rpc.sock"
    client = UnixJsonRpcClient(socket_path, timeout_s=30.0)

    print(f"Connecting to JSON-RPC server at {socket_path}")
    try:
        response = client.call(
            "winder.spindle.run",
            {"duration_s": 10.0, "rpm": 100.0},
        )
        print("Response:", response)
    except TimeoutError:
        print("JSON-RPC request timed out. The motion may still be running on the ESP32.")
    except Exception as exc:
        print(f"Error during JSON-RPC call: {exc}")


if __name__ == "__main__":
    main()
