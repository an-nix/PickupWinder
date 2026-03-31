#include "CommandController.h"
#include "Diag.h"
#include "SessionController.h"

CommandController* CommandController::s_instance = nullptr;

CommandController::CommandController(LinkSerial& serial, WebInterface& web)
    : _serial(serial), _web(web)
{
    s_instance = this;
}

void CommandController::onTransportCommand(const char* cmd, const char* val) {
    if (!s_instance) return;
    CommandEntry c;
    strncpy(c.cmd, cmd, sizeof(c.cmd)); c.cmd[sizeof(c.cmd)-1] = '\0';
    strncpy(c.val, val, sizeof(c.val)); c.val[sizeof(c.val)-1] = '\0';
    s_instance->pushCommand(c);
}

void CommandController::begin()
{
    // Allocate the fixed-size queue once at startup.
    _cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(CommandEntry));

    // Register both transports against the same push path so all commands are
    // normalized before reaching the control task.
    _web.setCommandCallback(CommandController::onTransportCommand);
    _serial.setCommandCallback(CommandController::onTransportCommand);

}

void CommandController::drain(SessionController::TickInput& out)
{
    // Drain without blocking so the control task remains deterministic.
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
        // Dropping is preferable to blocking a transport callback or async task.
        Diag::warnf("Command queue full, dropping '%s'", c.cmd);
    }
}
