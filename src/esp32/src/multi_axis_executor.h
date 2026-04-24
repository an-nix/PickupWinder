#pragma once

#include <esp_err.h>

class CommInterface;

class MultiAxisExecutor {
public:
    explicit MultiAxisExecutor(CommInterface& owner);

    esp_err_t start();

private:
    CommInterface& owner_;

    static void taskEntry(void* arg);
    void run();
};
