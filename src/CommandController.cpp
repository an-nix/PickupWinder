#include "CommandController.h"
#include "Diag.h"
#include "SessionController.h"

CommandController::CommandController(LinkSerial& serial, WebInterface& web)
    : _serial(serial), _web(web)
{
}

void CommandController::begin()
{
    // Register command callbacks centrally for both WebSocket and Serial.
    _web.setCommandCallback([this](const String& cmd, const String& val) {
        CommandEntry c;
        cmd.toCharArray(c.cmd, sizeof(c.cmd));
        val.toCharArray(c.val, sizeof(c.val));
        this->pushCommand(c);
    });
    _serial.setCommandCallback([this](const String& cmd, const String& val) {
        CommandEntry c;
        cmd.toCharArray(c.cmd, sizeof(c.cmd));
        val.toCharArray(c.val, sizeof(c.val));
        this->pushCommand(c);
    });
}

// addListener removed — command dispatch now via drain() into TickInput

void CommandController::tick()
{
    // No-op: use drain() to copy buffered commands into a TickInput
}

void CommandController::drain(SessionController::TickInput& out)
{
    // Copy up to out.MAX_CMDS commands from internal ring buffer into out.commands
    while (_bufCount > 0 && out.cmdCount < SessionController::TickInput::MAX_CMDS) {
        out.commands[out.cmdCount] = _cmdBuf[_bufHead];
        _bufHead = (_bufHead + 1) % CMD_BUF_SIZE;
        --_bufCount;
        out.cmdCount++;
    }
    if (_bufCount > 0 && out.cmdCount >= SessionController::TickInput::MAX_CMDS) {
        Diag::warnf("CommandController: drain truncated %d commands", _bufCount);
    }
}

void CommandController::handleCommand(const String& cmd, const String& value)
{
    // Previously dispatched to listeners; now commands are drained and handled by SessionController
}

void CommandController::pushCommand(const String& cmd, const String& val)
{
    CommandEntry c;
    cmd.toCharArray(c.cmd, sizeof(c.cmd));
    val.toCharArray(c.val, sizeof(c.val));
    pushCommand(c);
}

void CommandController::pushCommand(const CommandEntry& c)
{
    if (_bufCount >= CMD_BUF_SIZE) {
        Diag::warnf("Command buffer full, dropping '%s'", c.cmd);
        return;
    }
    _cmdBuf[_bufTail] = c;
    _bufTail = (_bufTail + 1) % CMD_BUF_SIZE;
    ++_bufCount;
}
