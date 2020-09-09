#include <ros/ros.h>
#include <std_msgs/String.h>

#include <modbustest/modbus_address.h>
#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <modbus-rtu.h>
#include <sys/socket.h>


//#define SERVER_ID         17
#define MODBUS_RTU_MAX_ADU_LENGTH  256

std::string port;
int baud;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  int i;
  int mcon = 0;
  int mbfrtn = 0;
  uint8_t *query;
  int header_length;
  modbus_t *mb;
  modbus_mapping_t *mb_mapping;

  uint16_t tab_reg[32];
  std::stringstream err_ss;
  int count;

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "modbus_talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("mdbus_chatter", 1000);

	ros::Rate loop_rate(10);

	ROS_INFO("MODBUS communication intializing...");
	n.param<std::string>("modbus/port", port, "/dev/ttyUSB0");
	n.param<int>("modbus/baud", baud, 19200);

	//Start of modbus  
	ROS_INFO("Trying to intialize communication @ port-> %s with %i baud rate",port.c_str(), baud);

	mb = modbus_new_rtu(port.c_str(), baud, 'E', 8, 1);
	//  mb = modbus_new_rtu("/dev/ttyUSB0", 19200, 'E', 8, 1);
	//  mb = modbus_new_rtu("/dev/pts/18", 19200, 'E', 8, 1);
	modbus_set_slave(mb, SERVER_ID);
	query =  static_cast<uint8_t *>(malloc(MODBUS_RTU_MAX_ADU_LENGTH));

	// Read 5 registers from the address 0 
	header_length = modbus_get_header_length(mb);
	
	ROS_INFO("header length %i",header_length);

	//  modbus_set_debug(mb, TRUE);
	modbus_set_debug(mb, FALSE);

	// modbus_write_register(mb, 1001, 10);

	mb_mapping = modbus_mapping_new(UT_BITS_ADDRESS + UT_BITS_NB,	UT_INPUT_BITS_ADDRESS + UT_INPUT_BITS_NB,UT_REGISTERS_ADDRESS + UT_REGISTERS_NB, UT_INPUT_REGISTERS_ADDRESS + UT_INPUT_REGISTERS_NB);

    if (mb_mapping == NULL) {
        //fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        ROS_ERROR("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_free(mb);
        return -1;
    }

    /* Unit tests of modbus_mapping_new (tests would not be sufficient if two
       nb_* were identical) */

    if (mb_mapping->nb_bits != UT_BITS_ADDRESS + UT_BITS_NB) {
        ROS_ERROR("Invalid nb bits (%d != %d)\n", UT_BITS_ADDRESS + UT_BITS_NB, mb_mapping->nb_bits);
        modbus_free(mb);
        return -1;
    }

    if (mb_mapping->nb_input_bits != UT_INPUT_BITS_ADDRESS + UT_INPUT_BITS_NB) {
        ROS_ERROR("Invalid nb input bits: %d\n", UT_INPUT_BITS_ADDRESS + UT_INPUT_BITS_NB);
        modbus_free(mb);
        return -1;
    }

    if (mb_mapping->nb_registers != UT_REGISTERS_ADDRESS + UT_REGISTERS_NB) {
        ROS_ERROR("Invalid nb registers: %d\n", UT_REGISTERS_ADDRESS + UT_REGISTERS_NB);
        modbus_free(mb);
        return -1;
    }

    if (mb_mapping->nb_input_registers != UT_INPUT_REGISTERS_ADDRESS + UT_INPUT_REGISTERS_NB) {
        ROS_ERROR("Invalid nb input registers: %d\n", UT_INPUT_REGISTERS_ADDRESS + UT_INPUT_REGISTERS_NB);
        modbus_free(mb);
        return -1;
    }

  /** INPUT STATUS **/
  modbus_set_bits_from_bytes(mb_mapping->tab_input_bits, UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB, UT_INPUT_BITS_TAB);

  if (mb_mapping == NULL) 
  {
       ROS_ERROR("stderr: Failed to allocate the mapping: %s", modbus_strerror(errno));
       
       modbus_free(mb);
       return -1;
  }

  /** INPUT REGISTERS **/
  for (i=0; i < UT_INPUT_REGISTERS_NB; i++) 
  {
  mb_mapping->tab_input_registers[UT_INPUT_REGISTERS_ADDRESS+i] = UT_INPUT_REGISTERS_TAB[i];
  }

  //Trying connect MODBUS
  ROS_INFO("Trying to establish MODBUS communication @ port-> %s with %i baud rate", port.c_str(), baud);
  mbfrtn = modbus_connect(mb);

  ROS_INFO("Try1");

  if (mbfrtn == -1) {
    ROS_ERROR("stderr: Unable to connect %s", modbus_strerror(errno));
    modbus_free(mb);
    return -1;
  }

int rc=0;
struct timeval old_response_timeout;
struct timeval response_timeout;

/* Define a new and too short timeout! */
response_timeout.tv_sec = 1;
response_timeout.tv_usec = 0;
modbus_set_response_timeout(mb, &response_timeout);
count = 0;

  while (ros::ok())
  {
	modbus_set_slave(mb, SERVER_ID);
	ROS_INFO("reading registers");
	mbfrtn = modbus_read_registers(mb, 0, 4, tab_reg);

        do {
            mbfrtn = modbus_receive(mb, query);
            /* Filtered queries return 0 */
	ROS_INFO("reading registers %i",mbfrtn);
        } while (mbfrtn == 0);

	if (mbfrtn == -1) 
	{
	// Connection closed by the client or error 
	ROS_INFO("stderr: connection failed %s", modbus_strerror(errno));
	break;
	}


	for (i=0; i < mbfrtn; i++) {
	ROS_INFO("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
	}

	ROS_INFO("completed reading registers 1 %i",mbfrtn);  

	modbus_read_input_registers(mb, 1001, 4, tab_reg);

	if (mbfrtn == -1) 
	{
	// Connection closed by the client or error 
	ROS_INFO("stderr: connection failed %s", modbus_strerror(errno));
	break;
	}

	for (i=0; i < mbfrtn; i++) {
	ROS_INFO("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
	}

	ROS_INFO("completed reading registers 2 %i",mbfrtn);

	/*
	mbfrtn = modbus_reply(mb, query, mbfrtn, mb_mapping);

	if (mbfrtn == -1) 
	{
	ROS_INFO("stderr: reply error %s", modbus_strerror(errno));
	break;
	}
	*/

	std_msgs::String msg;

	std::stringstream ss;
	ss << "hello world " <<count;
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());

	chatter_pub.publish(msg);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
  }

  modbus_mapping_free(mb_mapping);
  modbus_close(mb);
  modbus_free(mb);
  ROS_INFO("MODBUS communication Terminated");
  return 0;
}
