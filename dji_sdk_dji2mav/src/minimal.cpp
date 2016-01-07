#include <ros/ros.h>

#include <mavlink/common/mavlink.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#define BUFFER_LENGTH 2041

void send_thread(const int &sock);
void recv_thread(const int &sock);


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "minimal");
    ros::NodeHandle nh("~");

    int sock;
    if((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        ROS_ERROR("Socket invalid: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    /* Set the Communication Nonblocking */
    int flag = fcntl(sock, F_GETFL, 0);
    if(fcntl(sock, F_SETFL, flag | O_NONBLOCK) < 0) {
        ROS_ERROR("Set nonblocking fail: %s", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    //TODO: should be multi-threads
    send_thread(sock);
    //recv_thread(sock);

    return 0;
}


void send_thread(const int &sock){

    struct sockaddr_in locAddr;
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);
    if(-1 == bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr_in))) {
        ROS_ERROR("Bind fail: %s", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in gcAddr;
    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr("10.60.23.122");
    gcAddr.sin_port = htons(14550);

    mavlink_system_t mavSys;
    mavSys.sysid = 1;
    mavSys.compid = MAV_COMP_ID_IMU;

    mavlink_message_t msg;
    uint8_t sendBuf[BUFFER_LENGTH], recvBuf[BUFFER_LENGTH]; //TODO: not sure the size
    int bytes_sent, bytes_recv;

    while(ros::ok()) {
        /* Send Heartbeat */
        mavlink_msg_heartbeat_pack(mavSys.sysid, mavSys.compid, &msg, 
                MAV_TYPE_QUADROTOR, //quadrotor type
                MAV_AUTOPILOT_GENERIC, //full support for everything
                MAV_MODE_GUIDED_DISARMED, //autocontrol and disarmed
                0, //uint_32, custom_mode, defined by user/adopter
                MAV_STATE_STANDBY); //vehicle is grounded and on standby
        uint16_t len = mavlink_msg_to_send_buffer(sendBuf, &msg);
        bytes_sent = sendto(sock, sendBuf, len, MSG_DONTWAIT, (struct sockaddr *)&gcAddr, 
                sizeof(struct sockaddr_in));
        //memset(sendBuf, 0, BUFFER_LENGTH);

        /* Receive Data */
        socklen_t fromlen;
        bytes_recv = recvfrom(sock, (void *)recvBuf, BUFFER_LENGTH, MSG_DONTWAIT, 
                (struct sockaddr *)&gcAddr, &fromlen);
        if(bytes_recv > 0) {
            mavlink_message_t recvMsg;
            mavlink_status_t recvStatus;

            ROS_INFO("Bytes Received: %d bytes.", bytes_recv);
            printf("Datagram: ");
            for(int i = 0; i < bytes_recv; ++i) {
                unsigned char tmp = recvBuf[i];
                printf("%02x", tmp);
                if(mavlink_parse_char(MAVLINK_COMM_0, recvBuf[i], &recvMsg, &recvStatus)) {
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", recvMsg.sysid, recvMsg.compid, recvMsg.len, recvMsg.msgid);
                }
            }
            printf("\n");
        }
        //memset(recvBuf, 0, BUFFER_LENGTH);
        ros::Duration(1.0).sleep();
    }
}
