/**
 * @file multi_axis_executor.h
 * @brief Multi-axis planned segment executor task.
 */

#pragma once

#include <esp_err.h>

class CommRuntime;

/**
 * @brief Drains planned segments and feeds axis drivers with coordinated motion.
 */
class MultiAxisExecutor {
public:
    explicit MultiAxisExecutor(CommRuntime& runtime);
    esp_err_t start();

private:
    CommRuntime& runtime_;

    static void taskEntry(void* arg);
    void run();
};
