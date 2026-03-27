#pragma once
#include "LinkSerial.h"
#include "WebInterface.h"
#include "SessionController.h"
#include "Types.h"
#include <Arduino.h>

class CommandController
{
public:
   CommandController(LinkSerial& serial, WebInterface& web);
   void begin();
   void tick();
   void drain(SessionController::TickInput& out);
   void handleCommand(const String& cmd, const String& val);
   void pushCommand(const CommandEntry& c);
   // Listener API removed; commands are drained into SessionController::TickInput

private:
   LinkSerial& _serial;
   WebInterface& _web;
   // listeners removed
   // Simple ring buffer for incoming commands
   static constexpr int CMD_BUF_SIZE = 32;
   CommandEntry _cmdBuf[CMD_BUF_SIZE];
   int _bufHead = 0;
   int _bufTail = 0;
   int _bufCount = 0;
};