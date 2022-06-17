#include "tcp_client.hpp"
namespace ccr_split {
int send_can(can_frame &send_frame, common::Net &tcp_server)
{
    uint8_t can_buff[CAN_BUFFER_SIZE];
    can_buff[0] = send_frame.can_dlc;   
    can_buff[1] = (send_frame.can_id >> 24) & 0xff;
    can_buff[2] = (send_frame.can_id >> 16) & 0xff;
    can_buff[3] = (send_frame.can_id >> 8) & 0xff;
    can_buff[4] = send_frame.can_id & 0xff;
    for(int i = 0; i < 8; i ++){
        can_buff[i + 5] = send_frame.data[i];
    }
    //printf("can_id:0x%x!\n", send_frame.can_id);
    int nb = tcp_server.TcpSendtoClient(can_buff, CAN_BUFFER_SIZE);
    return nb;

}

int recv_can(can_frame &recv_frame, common::Net &tcp_server)
{
    uint8_t can_buff[CAN_BUFFER_SIZE];
    //printf("recv_can start\n");
    int nb = tcp_server.TcpRecvfromClient(can_buff, sizeof(can_buff));
    recv_frame.can_dlc = can_buff[0];
    __u16 cob_id = (can_buff[3] << 8) + can_buff[4];
    //printf("recv_can done2\n");
    recv_frame.can_id = cob_id;
    //printf("can_id:0x%x; 0x%x\n", recv_frame.can_id);
    for(int i = 0; i < 8; i ++){
        recv_frame.data[i] = can_buff[i + 5];
        //printf("recv_frame.data[%d]: 0x%02x\n", i, recv_frame.data[i]);
    }
    //printf("recv_can done3\n");
    return nb;
}

int recv_tf(can_frame &recv_frame, common::Net &tcp_server)
{
    uint8_t can_buff[CAN_BUFFER_SIZE];
    //printf("recv_tf start\n");
    int nb = tcp_server.TcpRecvfromClient(can_buff, sizeof(can_buff));
    recv_frame.can_dlc = can_buff[0];
    __u32 cob_id = (can_buff[1] << 24) + (can_buff[2] << 16) + (can_buff[3] << 8) + can_buff[4];
    recv_frame.can_id = cob_id;
    //printf("tfcan_id:0x%x\n", recv_frame.can_id);
    for(int i = 0; i < 8; i ++){
        recv_frame.data[i] = can_buff[i + 5];
        //printf("recv_frame.data[%d]: 0x%02x\n", i, recv_frame.data[i]);
    }
    if (recv_frame.data[5] < 7 )
        return (-1);
    //printf("recv_can done3\n");
    return nb;
}

}