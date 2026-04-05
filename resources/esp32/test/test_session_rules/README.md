# Session and Geometry Tests

Planned scope:

- Session start/pause/stop arbitration from multi-source inputs.
- Pot re-arm behavior after non-pot control source events.
- Geometry window-shift and trim nudge boundary handling.
- End-position arming behavior near target turns.

Implementation note:

These tests should be introduced with seams/mocks around WinderApp dependencies
(stepper, lateral controller, and persistent storage) to keep tests deterministic.
