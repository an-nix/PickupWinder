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
    if (!cmd || !val) return;

    const size_t cmdLen = strlen(cmd);
    const size_t valLen = strlen(val);
    if (cmdLen >= CommandEntry::CMD_SZ || valLen >= CommandEntry::VAL_SZ) {
        s_instance->_rejectedOversize++;
        Diag::warnf("Command rejected (oversize): cmd=%u/%u val=%u/%u",
                    (unsigned)cmdLen, (unsigned)(CommandEntry::CMD_SZ - 1),
                    (unsigned)valLen, (unsigned)(CommandEntry::VAL_SZ - 1));
        return;
    }

    char canonicalCmd[CommandEntry::CMD_SZ];
    CommandId id = CommandId::Unknown;
    if (!CommandRegistry::normalizeAndValidate(cmd, val, canonicalCmd, sizeof(canonicalCmd), &id)) {
        (void)id;
        s_instance->_rejectedSchema++;
        Diag::warnf("Command rejected (schema): cmd='%s' val='%s'", cmd, val);
        return;
    }

    CommandEntry c;
    strncpy(c.cmd, canonicalCmd, sizeof(c.cmd)); c.cmd[sizeof(c.cmd)-1] = '\0';
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
        _droppedQueueFull++;
        // Dropping is preferable to blocking a transport callback or async task.
        Diag::warnf("Command queue full, dropping '%s'", c.cmd);
    } else {
        _enqueued++;
    }
}

CommandController::Stats CommandController::getStats() const {
    Stats s;
    s.enqueued = _enqueued;
    s.droppedQueueFull = _droppedQueueFull;
    s.rejectedOversize = _rejectedOversize;
    s.rejectedSchema = _rejectedSchema;
    return s;
}
