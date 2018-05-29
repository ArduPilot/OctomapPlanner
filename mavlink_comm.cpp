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

}

MavlinkComm::~MavlinkComm()
{
	io_work.reset();
	io_service->stop();
}

void MavlinkComm::poll_data()
{
	memset(buf, 0, BUFFER_LENGTH);
	recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
	if (recsize > 0)
	{	
		// boost::mutex mtx_;
		boost::lock_guard<boost::mutex> guard(mtx_);				
		// Something received - print out all bytes and parse packet
		mavlink_message_t msg;
		mavlink_status_t status;
		
		// printf("Bytes Received: %d\nDatagram: ", (int)recsize);
		for (ssize_t i = 0; i < recsize; ++i)
		{
			// temp = buf[i];
			// printf("%02x ", (unsigned char)temp);
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
			{
				switch(msg.msgid)
			      	{
			      		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
							{
								// Packet received
								mavlink_msg_local_position_ned_decode(&msg, &pos_msg);
								printf("\nPose x: %f y: %f z: %f\n", pos_msg.x, pos_msg.y, pos_msg.z);
								printf("\n");
								break;
							}
						default:
							{
								printf("MSG ID: %d\n", msg.msgid);
								break;
							}
					}
			}
		}
	}
	// boost::this_thread::sleep( boost::posix_time::milliseconds(10));
	memset(buf, 0, BUFFER_LENGTH);
}

void MavlinkComm::run()
{
	poll_data();
	timer->expires_from_now(boost::posix_time::millisec(interval));
	timer->async_wait(boost::bind(&MavlinkComm::run, this));
}