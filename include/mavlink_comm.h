/*
*   OctomapPlanner
*
*   Copyright (C) 2018  ArduPilot
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Author Ayush Gaud <ayush.gaud[at]gmail.com>
*/

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
#include <time.h>
#include <arpa/inet.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>

#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include <mavlink/v2.0/common/common.h>
// #include <mavlink/v2.0/mavlink_conversions.h>

#include "debug_definitions.h"

class MavlinkComm
{
public:
	MavlinkComm(const size_t& bind_port, const size_t& remote_port, boost::asio::io_service *io_service);
	~MavlinkComm();
	void run();
	mavlink_local_position_ned_t pos_msg;
	mavlink_attitude_t orientation_msg;
	uint8_t interval;
	void gotoNED(float x, float y, float z, float yaw);
private:
	char target_ip[50];
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr; 
	struct sockaddr_in locAddr;
	static const size_t BUFFER_LENGTH = 2041;
	uint8_t rec_buf[BUFFER_LENGTH], tx_buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;
	std::shared_ptr<boost::asio::io_service> io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	boost::mutex mtx_;
	std::shared_ptr<boost::asio::deadline_timer> timer;
	void poll_data();
};

#endif