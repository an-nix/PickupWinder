#pragma once
#include "LinkSerial.h"
#include "WebInterface.h"
#include "SessionController.h"
#include "Types.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @brief Transport-to-control command bridge.
 *
 * `CommandController` owns a FreeRTOS queue used to safely hand off commands
 * produced by asynchronous transports to the deterministic control task.
 */
class CommandController
{
public:
   /**
    * @brief Construct the command bridge.
    * @param serial Serial transport used to receive line-based commands.
    * @param web Web transport used to receive WebSocket commands.
    */
   CommandController(LinkSerial& serial, WebInterface& web);

   /**
    * @brief Transport callback adapter used for C-style function pointers.
    */
   static void onTransportCommand(const char* cmd, const char* val);

   /** @brief Create the queue and register transport callbacks. */
   void begin();

   /**
    * @brief Drain queued commands into a control tick packet.
    * @param out Tick packet populated with up to `MAX_CMDS` commands.
    */
   void drain(SessionController::TickInput& out);

   /**
    * @brief Push one parsed command into the inter-task queue.
    * @param c Fixed-size command entry.
    */
   void pushCommand(const CommandEntry& c);

private:
   LinkSerial& _serial;
   WebInterface& _web;
   // FreeRTOS queue for thread-safe command passing between tasks.
   static constexpr int CMD_QUEUE_SIZE = 32;
   QueueHandle_t _cmdQueue = nullptr;

   static CommandController* s_instance;
};