# Async Streaming Refactoring - Migration & Deployment Guide

## What Changed

The JSON-RPC motion handlers now use **background threading** to prevent blocking during streaming operations. This means:

✅ Handler returns immediately (10ms instead of 5-10 seconds)  
✅ Server can process multiple requests concurrently  
✅ Session tracking via `session_id` for monitoring  
✅ No changes to SPI protocol or ESP32 firmware needed  

## Files Modified

### New Files
- `src/rpi/motion/streaming_manager.py` - Background streaming orchestration
- `doc/async_streaming_refactoring.md` - Detailed technical documentation
- `src/rpi/test_async_streaming.py` - Integration test demonstrating non-blocking behavior

### Modified Files
- `src/rpi/motion/axis_controller.py` - Uses background streaming
- `src/rpi/motion/multi_axis_controller.py` - Uses background streaming  
- `src/rpi/winding/app.py` - Creates shared StreamingManager
- `src/rpi/jsonrpc/handlers.py` - Added `session.status` and `session.wait` methods

## Testing Locally

Before deploying to Raspberry Pi, verify everything works:

### 1. Syntax Check
```bash
cd /home/nicolas/Documents/PlatformIO/Projects/PickupWinder
.venv/bin/python -m py_compile \
  src/rpi/motion/streaming_manager.py \
  src/rpi/motion/axis_controller.py \
  src/rpi/motion/multi_axis_controller.py \
  src/rpi/winding/app.py \
  src/rpi/jsonrpc/handlers.py
```

### 2. Import Check
```bash
.venv/bin/python -c "
import sys
sys.path.insert(0, 'src/rpi')
from motion.streaming_manager import StreamingManager
from motion.axis_controller import AxisController
from motion.multi_axis_controller import MultiAxisMotionController
from winding import WinderApp
from jsonrpc import AppRpcHandler
print('✓ All imports working')
"
```

### 3. Integration Test (requires SPI hardware)
```bash
# Make sure ESP32 is connected via SPI and powered on
.venv/bin/python src/rpi/test_async_streaming.py
```

The test will:
1. Start the WinderApp and JSON-RPC server
2. Send `winder.spindle.run()` command
3. Verify handler returns immediately with session_id
4. Poll `winder.session.status()` while streaming is active
5. Send `winder.motion.run()` command for multi-axis test
6. Use `winder.session.wait()` to wait for completion

## API Changes for Clients

### Backward Compatibility

The old API still works, but returns different data:

**Old (blocking):**
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "status": "spindle_started",
    "details": {
      "axis_id": 0,
      "duration_s": 5.0,
      "target_rpm": 100.0,
      "blocks_sent": 156
    }
  }
}
```

**New (non-blocking):**
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "status": "spindle_started",
    "details": {
      "axis_id": 0,
      "duration_s": 5.0,
      "target_rpm": 100.0,
      "session_id": 1,
      "status": "streaming_started"
    }
  }
}
```

### New Session Management Methods

#### `winder.session.status` (non-blocking)

Request:
```json
{
  "jsonrpc": "2.0",
  "id": 100,
  "method": "winder.session.status",
  "params": {
    "session_id": 1
  }
}
```

Response:
```json
{
  "jsonrpc": "2.0",
  "id": 100,
  "result": {
    "session_id": 1,
    "status": "running",
    "started_at": 1234567890.5,
    "completed_at": null,
    "block_count": 42,
    "error": null
  }
}
```

Status values: `"queued"`, `"running"`, `"completed"`, `"failed"`

#### `winder.session.wait` (optional blocking)

Request:
```json
{
  "jsonrpc": "2.0",
  "id": 200,
  "method": "winder.session.wait",
  "params": {
    "session_id": 1,
    "timeout_s": 10.0
  }
}
```

Response (after motion completes or timeout):
```json
{
  "jsonrpc": "2.0",
  "id": 200,
  "result": {
    "session_id": 1,
    "status": "completed",
    "block_count": 156,
    "error": null,
    "duration_s": 5.234
  }
}
```

## Deployment Steps

### 1. Backup Current Code
```bash
cd ~/Documents/PlatformIO/Projects/PickupWinder
git status  # Check if using git
# If yes: git commit -m "Backup before async refactor"
# If no: cp -r src/rpi src/rpi.backup
```

### 2. Copy New Code to Raspberry Pi
```bash
scp -r ./src/rpi/* pi@<raspberry-ip>:/home/pi/winder
```

### 3. Test on Raspberry Pi
```bash
ssh pi@<raspberry-ip>
cd /home/pi/winder
python test_async_streaming.py
```

### 4. Monitor Underrun Metrics

After deploying, check if underrun issues are resolved:

```bash
# Look for output like:
# status=buf=80.0ms queue_free=(4096,4096,4096) ring_free=(4090,4090) underrun=(1,0,0,0)

# The important part is 'underrun=(1,0,0,0)':
# - (1,0,0,0) = benign initial underrun (documented behavior)
# - (5,0,0,0) or higher = streaming underrun (problem)
# - Multiple runs with consistent (1,0,0,0) = success
```

## Troubleshooting

### Issue: Handler still blocks?
- Make sure you're running the new code (check file timestamps)
- Verify `StreamingManager` is being created: check app.py for `streaming_manager` creation

### Issue: Session ID not in response?
- Check that you're using the new code
- Verify response has `"details"` field with `"session_id"`

### Issue: Session status shows "failed"?
- Check the `"error"` field in session status
- Look for streaming errors in application logs
- Ensure ESP32 is responding (check SPI connection)

### Issue: Timeout errors?
- Default timeout is 5 minutes (300s)
- For very long motions, increase `timeout_s` in `session.wait` call
- Check if motion actually started on ESP32

## Performance Validation

Create this test script to measure responsiveness improvement:

```python
import time
import socket
import json

def time_rpc_response(method, params, socket_path="/tmp/winder.sock"):
    """Measure time for RPC handler to respond."""
    start = time.monotonic()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(socket_path)
    
    request = {
        "jsonrpc": "2.0",
        "id": 1,
        "method": method,
        "params": params
    }
    sock.sendall((json.dumps(request) + "\n").encode())
    
    response = sock.makefile().readline()
    elapsed = time.monotonic() - start
    sock.close()
    
    return elapsed

# Test spindle motion
elapsed = time_rpc_response("winder.spindle.run", {
    "duration_s": 10.0,
    "rpm": 100.0
})
print(f"Handler response time: {elapsed*1000:.1f}ms")
# Expected: ~10-50ms (was ~10s before)
```

## Rollback Plan

If you need to revert to the old blocking behavior:

```bash
# On Raspberry Pi:
cd /home/pi/winder
git checkout HEAD~1 src/rpi/  # Or restore from backup
```

Or manually revert:
1. Replace `axis_controller.py` - remove `streaming_manager` references, call `streamer.stream_all()` directly
2. Replace `multi_axis_controller.py` - same as above
3. Replace `winding/app.py` - remove `StreamingManager` creation
4. Remove `jsonrpc/handlers.py` new methods (session.status, session.wait)
5. Delete `motion/streaming_manager.py`

## Questions?

Check `doc/async_streaming_refactoring.md` for detailed technical explanation of the refactoring.

