#pragma once
#include "LinkSerial.h"
#include "WebInterface.h"
#include "SessionController.h"
#include "Types.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class CommandController
{
public:
   CommandController(LinkSerial& serial, WebInterface& web);
   void begin();
   void drain(SessionController::TickInput& out);
   void pushCommand(const CommandEntry& c);

private:
   LinkSerial& _serial;
   WebInterface& _web;
   // FreeRTOS queue for thread-safe command passing between tasks
   static constexpr int CMD_QUEUE_SIZE = 32;
   QueueHandle_t _cmdQueue = nullptr;
};