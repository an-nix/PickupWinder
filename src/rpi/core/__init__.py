from importlib import import_module


_EXPORTS = {
    "AppConfiguration": ".config",
    "ConfigurationManager": ".config",
    "MotionCoordinator": ".coordinator",
    "EventBus": ".events",
    "EventKind": ".events",
    "EngineState": ".shared_state",
    "RuntimeStatusService": ".status",
    "SharedState": ".shared_state",
    "WindingEngine": ".engine",
}


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_name, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value

__all__ = [
    "AppConfiguration",
    "ConfigurationManager",
    "MotionCoordinator",
    "EventBus",
    "EventKind",
    "EngineState",
    "RuntimeStatusService",
    "SharedState",
    "WindingEngine",
]
