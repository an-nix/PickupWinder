"""Compatibility shim — client moved to `pickup.client` package.

Import the modern client from `pickup.client`. This shim keeps old
`from daemon_client import DaemonClient` imports working during the
transition; prefer `from pickup.client import DaemonClient`.
"""

from pickup.client import DaemonClient

__all__ = ["DaemonClient"]
