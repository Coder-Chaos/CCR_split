#pragma once
// #include "common.hpp"
#include "net.hpp"
#include "can.hpp"

#define CAN_BUFFER_SIZE 13
namespace ccr_split {
int send_can(struct can_frame &send_frame, common::Net &tcp_server);
    
int recv_can(struct can_frame &recv_frame, common::Net &tcp_server);

int recv_tf(struct can_frame &recv_frame, common::Net &tcp_server);
}