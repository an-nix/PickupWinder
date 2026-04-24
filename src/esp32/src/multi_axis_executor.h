#pragma once

#include <esp_err.h>

class CommRuntime;

class MultiAxisExecutor {
public:
    explicit MultiAxisExecutor(CommRuntime& runtime);

    esp_err_t start();

private:
    CommRuntime& runtime_;

    static void taskEntry(void* arg);
    void run();
};
