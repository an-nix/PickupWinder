from __future__ import annotations

from jsonrpc import UnixJsonRpcClient


def main() -> None:
    client = UnixJsonRpcClient("//tmp/pickup_winder_rpc.sock")
    print("ping ->", client.call("winder.ping"))
    print("status ->", client.call("winder.status"))
    print("config ->", client.call("winder.config"))


if __name__ == "__main__":
    main()
