#ifndef MAVLINK_COMM_H
#define MAVLINK_COMM_H

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>

#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include <mavlink/v2.0/common/common.h>

class MavlinkComm {
public:
	MavlinkComm(const size_t& bind_port, const size_t& remote_port);
	// void ~MavlinkComm();
	void poll_data();
	mavlink_local_position_ned_t pos_msg;

private:
	char target_ip[50];
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr; 
	struct sockaddr_in locAddr;
	static const size_t BUFFER_LENGTH = 2041;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;

};

#endif