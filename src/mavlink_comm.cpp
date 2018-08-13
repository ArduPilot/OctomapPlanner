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

#include "mavlink_comm.h"

MavlinkComm::MavlinkComm(const size_t& bind_port, const size_t& remote_port, boost::asio::io_service *_io_service) :
	io_service(_io_service),
	interval(10),
	timer(std::make_shared<boost::asio::deadline_timer>(*io_service, boost::posix_time::millisec(interval))),
	io_work(new boost::asio::io_service::work(*io_service))
{
	
	strcpy(target_ip, "127.0.0.1");
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(bind_port);

	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(remote_port);

	/* Bind the socket to port bind_port - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    }
    /* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
    INFO("Mavlink Initialized");

}

MavlinkComm::~MavlinkComm()
{
	io_work.reset();
	io_service->stop();
}

void MavlinkComm::poll_data()
{
	memset(rec_buf, 0, BUFFER_LENGTH);
	recsize = recvfrom(sock, (void *)rec_buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
	// Something received - parse packet
	if (recsize > 0)
	{	
		boost::lock_guard<boost::mutex> guard(mtx_);				
		mavlink_message_t msg;
		mavlink_status_t status;
		
		// printf("Bytes Received: %d\nDatagram: ", (int)recsize);
		for (ssize_t i = 0; i < recsize; ++i)
		{
			// temp = rec_buf[i];
			// printf("%02x ", (unsigned char)temp);
			if (mavlink_parse_char(MAVLINK_COMM_0, rec_buf[i], &msg, &status))
			{
				switch(msg.msgid)
			      	{
			      		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
							{
								// Packet received
								mavlink_msg_local_position_ned_decode(&msg, &pos_msg);
								// DBG("Pose x: " << pos_msg.x << " y: " << pos_msg.y << " z: " << pos_msg.z);
								break;
							}
						case MAVLINK_MSG_ID_ATTITUDE:
							{
								mavlink_msg_attitude_decode(&msg, &orientation_msg);
								// Dbg("Attitude roll: " <<  orientation_msg.roll << " pitch: " << orientation_msg.pitch << " yaw: " << orientation_msg.yaw);
							}
						default:
							{
								// Dbg("MSG ID: " << msg.msgid);
								break;
							}
					}
			}
		}
	}

	memset(rec_buf, 0, BUFFER_LENGTH);
}

void MavlinkComm::run()
{
	poll_data();
	timer->expires_from_now(boost::posix_time::millisec(interval));
	timer->async_wait(boost::bind(&MavlinkComm::run, this));
}

void MavlinkComm::gotoNED(float x, float y, float z, float yaw)
{	
	mavlink_message_t msg;
	uint16_t len;
	int bytes_sent;
	mavlink_msg_set_position_target_local_ned_pack(1, 240, &msg, 0, 1, 1, MAV_FRAME_LOCAL_NED, 0b0000101111111000, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0);
	len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	bytes_sent = sendto(sock, tx_buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	memset(tx_buf, 0, BUFFER_LENGTH);
}