"""Unix socket JSON-RPC support for the PickupWinder host application."""

from .client import UnixJsonRpcClient
from .handlers import RpcHandler, SystemRpcHandler
from .protocol import (
    JSONRPC_VERSION,
    JsonRpcError,
    JsonRpcInvalidRequestError,
    JsonRpcParseError,
    JsonRpcMethodNotFoundError,
    JsonRpcResponse,
    JsonRpcRequest,
    make_error_response,
    make_response,
    parse_json_rpc,
)
from .rpc_server import JsonRpcServer

__all__ = [
    "JsonRpcServer",
    "UnixJsonRpcClient",
    "RpcHandler",
    "SystemRpcHandler",
    "JSONRPC_VERSION",
    "JsonRpcError",
    "JsonRpcInvalidRequestError",
    "JsonRpcParseError",
    "JsonRpcMethodNotFoundError",
    "JsonRpcResponse",
    "JsonRpcRequest",
    "make_error_response",
    "make_response",
    "parse_json_rpc",
]
