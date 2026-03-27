#include "CommandController.h"
#include "Diag.h"
#include "SessionController.h"

CommandController::CommandController(LinkSerial& serial, WebInterface& web)
    : _serial(serial), _web(web)
{
}

void CommandController::begin()
{
    _cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(CommandEntry));

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

void CommandController::drain(SessionController::TickInput& out)
{
    // Drain up to MAX_CMDS from the FreeRTOS queue (non-blocking)
    CommandEntry entry;
    while (out.cmdCount < SessionController::TickInput::MAX_CMDS &&
           xQueueReceive(_cmdQueue, &entry, 0) == pdTRUE) {
        out.commands[out.cmdCount++] = entry;
    }
}

void CommandController::pushCommand(const CommandEntry& c)
{
    if (!_cmdQueue) return;
    if (xQueueSend(_cmdQueue, &c, 0) != pdTRUE) {
        Diag::warnf("Command queue full, dropping '%s'", c.cmd);
    }
}
