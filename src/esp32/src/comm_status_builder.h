#pragma once

#include <stdint.h>

#include "messages.h"

class CommRuntime;

class CommStatusBuilder {
public:
    explicit CommStatusBuilder(const CommRuntime& runtime);

    void buildStatusFrame(uint8_t* out_frame) const;

private:
    const CommRuntime& runtime_;

    void populateAxisStatus(StatusPayload& payload) const;
    void populateProtocolStatus(StatusPayload& payload) const;
    void populatePlannerStatus(StatusPayload& payload) const;
    void populateEndstopStatus(StatusPayload& payload) const;
};
