#pragma once

#include <stdint.h>

#include "messages.h"

class CommInterface;

class CommStatusBuilder {
public:
    explicit CommStatusBuilder(const CommInterface& owner);

    void buildStatusFrame(uint8_t* out_frame) const;

private:
    const CommInterface& owner_;

    void populateAxisStatus(StatusPayload& payload) const;
    void populateProtocolStatus(StatusPayload& payload) const;
    void populatePlannerStatus(StatusPayload& payload) const;
    void populateEndstopStatus(StatusPayload& payload) const;
};
