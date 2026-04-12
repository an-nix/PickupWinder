"""Pickup package: contains client and controller modules for PickupWinder.

This package allows `pickup_test.py` to import stable module paths like
`pickup.daemon_client` whether the code is executed as a package or a
standalone script.
"""

__all__ = ["client", "daemon_client", "pickup_controller", "pru_client"]
