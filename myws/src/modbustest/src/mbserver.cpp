#include <ros/ros.h>
#include <std_msgs/String.h>

#include <modbustest/modbus_address.h>
#include <modbustest/readholdingregisters.h>
#include <modbustest/readinputregisters.h>
#include <modbustest/mbwritemultipleregisters.h>
#include <modbustest/WriteMultipleRegisters.h>
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
int current_slave_id;
modbus_t *mb;
modbus_mapping_t *mb_mapping;

void modbus_writemultipleregisters(const modbustest::WriteMultipleRegisters::ConstPtr& msg)
{
	uint16_t tab_reg[32];
	modbus_set_slave(mb, msg->slaveid);
	int k=0;
	int mfrtn=-1;
	//MODBUS sends MSB first
	k = msg->write_multiple_registers.size() - 1;
	while (!msg->write_multiple_registers.empty())
	{
	tab_reg[k] = msg->write_multiple_registers.back();
	//msg->write_multiple_registers.pop_back();
	k--;
	}

	mfrtn = modbus_write_registers(mb, msg->write_multiple_registers_start_address, msg->write_multiple_registers_count, tab_reg);
	ROS_INFO("response: MODBUS write multiple register function returns::%i",mfrtn);
	modbus_set_slave(mb, current_slave_id);
}


bool writemultipleregisters(modbustest::mbwritemultipleregisters::Request  &req, modbustest::mbwritemultipleregisters::Response &res)
{
	uint16_t tab_reg[32];
	modbus_set_slave(mb, req.slaveid);
	int k=0;
	int mfrtn=-1;
	//MODBUS sends MSB first
	k = req.write_multiple_registers.size() - 1;
	while (!req.write_multiple_registers.empty())
	{
	tab_reg[k] = req.write_multiple_registers.back();
	req.write_multiple_registers.pop_back();
	k--;
	}

	mfrtn = modbus_write_registers(mb, req.write_multiple_registers_start_address, req.write_multiple_registers_count, tab_reg);
	res.result = mfrtn;
	ROS_INFO("response: MODBUS write multiple register function returns::%i",mfrtn);
	modbus_set_slave(mb, current_slave_id);
  	return true;
}

int main(int argc, char **argv)
{
	int i;
	unsigned int mb_read_input_registers_slaveindex;
	unsigned int mb_read_holding_registers_slaveindex;
	int mbfrtn;
	uint8_t *query;
	int header_length;
	int mbtoutsecs;

	uint16_t tab_reg[32];
	std::stringstream err_ss;
	int count;

	ros::init(argc, argv, "modbus_talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("mdbus_server", 1000);

	ros::ServiceServer writemultipleregisters_srv = n.advertiseService("modbus_writemultipleregisters", writemultipleregisters);
	ros::Publisher readholdingregisters_pub = n.advertise<modbustest::readholdingregisters>("modbus_read_holding_registers", 10);
	ros::Publisher readinputregisters_pub = n.advertise<modbustest::readinputregisters>("modbus_read_input_registers", 10);
//	ros::Subscriber sub = n.subscribe("modbus_writemultipleregisters", 1000, modbus_writemultipleregisters);
	ros::Rate loop_rate(100);


	ROS_INFO("MODBUS communication intializing...");
	n.param<std::string>("modbus/port", port, "/dev/ttyUSB0");
	n.param<int>("modbus/baud", baud, 19200);
	n.param<int>("modbus/timeout", mbtoutsecs, 10);

	//Getting slave details, coil counts and starting address register details from ros parameters
	std::vector<int> read_holding_registers_slaveids;
	n.getParam("modbus/read_holding_registers_slaveids", read_holding_registers_slaveids);

	std::vector<int> read_holding_registers_count;
	n.getParam("modbus/read_holding_registers_count", read_holding_registers_count);

	std::vector<int> read_holding_registers_start_address;
	n.getParam("modbus/read_holding_registers_start_address", read_holding_registers_start_address);

	std::vector<int> read_input_registers_slaveids;
	n.getParam("modbus/read_input_registers_slaveids", read_input_registers_slaveids);

	std::vector<int> read_input_registers_count;
	n.getParam("modbus/read_input_registers_count", read_input_registers_count);

	std::vector<int> read_input_registers_start_address;
	n.getParam("modbus/read_input_registers_start_address", read_input_registers_start_address);

	if(read_holding_registers_slaveids.size() <= 0) 
	{
	ROS_ERROR("Error: Parameters for read holding registers - slave id not loaded sucessfully!");
	return 0;
	}
	if(read_input_registers_slaveids.size() <= 0) 
	{
	ROS_ERROR("Error: Parameters for read input registers - slave id  not loaded sucessfully!");
	return 0;
	}

	if(( (read_holding_registers_slaveids.size()) != (read_holding_registers_count.size()) && (read_holding_registers_slaveids.size()) != (read_holding_registers_start_address.size()) ))
{
	ROS_ERROR("Error: Parameters corrsponding to slave ids for read holding registers  not loaded sucessfully!");
	return 0;
}

	if(( (read_input_registers_slaveids.size()) != (read_input_registers_count.size()) && (read_input_registers_slaveids.size()) != (read_input_registers_start_address.size()) ))
{
	ROS_ERROR("Error: Parameters corrsponding to slave ids for read input registers  not loaded sucessfully!");
	return 0;
}

	//Start of modbus  
	ROS_INFO("Trying to intialize communication @ port-> %s with %i baud rate",port.c_str(), baud);

	ROS_INFO("Trying to intialize communication with %li slaves with %li input registers and %li holding registers", read_holding_registers_slaveids.size(), read_holding_registers_count.size(), read_input_registers_count.size());

	mb = modbus_new_rtu(port.c_str(), baud, 'E', 8, 1);
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
	ROS_ERROR("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
	modbus_free(mb);
	return -1;
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
	response_timeout.tv_sec = mbtoutsecs;
	response_timeout.tv_usec = 0;
	modbus_set_response_timeout(mb, &response_timeout);
	count = 0;

	mb_read_input_registers_slaveindex = 0;
	mb_read_holding_registers_slaveindex = 0;

	modbustest::readholdingregisters obj_readholdingregisters;
	modbustest::readinputregisters obj_readinputregisters;

  while (ros::ok())
  {
	current_slave_id = read_holding_registers_slaveids[mb_read_holding_registers_slaveindex];
	modbus_set_slave(mb, current_slave_id);
	ROS_INFO("reading registers for slave ID: %i",current_slave_id);

	mbfrtn = modbus_read_registers(mb, read_holding_registers_start_address[mb_read_holding_registers_slaveindex], read_holding_registers_count[mb_read_holding_registers_slaveindex], tab_reg);

	if (mbfrtn == -1) 
	{
	// Connection closed by the client or error 
	ROS_WARN("stderr: connection failed %s", modbus_strerror(errno));
	continue;
	}

	obj_readholdingregisters.timestamp = ros::Time::now();
	obj_readholdingregisters.slaveid = current_slave_id;
	obj_readholdingregisters.read_holding_registers_count = read_holding_registers_count[mb_read_holding_registers_slaveindex];
	obj_readholdingregisters.read_holding_registers_start_address =  read_holding_registers_start_address[mb_read_holding_registers_slaveindex];

	obj_readholdingregisters.read_holding_registers.clear();

	for (i=0; i < mbfrtn; i++) {
	obj_readholdingregisters.read_holding_registers.push_back(tab_reg[i]);
//	ROS_INFO("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
	}

	current_slave_id = read_input_registers_slaveids[mb_read_input_registers_slaveindex];
	modbus_set_slave(mb, current_slave_id);

	ROS_INFO("reading registers for slave ID: %i",current_slave_id);

	obj_readinputregisters.timestamp = ros::Time::now();
	obj_readinputregisters.slaveid = read_input_registers_slaveids[mb_read_input_registers_slaveindex];
	obj_readinputregisters.read_input_registers_count = read_input_registers_count[mb_read_input_registers_slaveindex];
	obj_readinputregisters.read_input_registers_start_address = read_input_registers_start_address[mb_read_input_registers_slaveindex];
	obj_readinputregisters.read_input_registers.clear();

	mbfrtn = modbus_read_input_registers(mb, read_input_registers_start_address[mb_read_input_registers_slaveindex], read_input_registers_count[mb_read_input_registers_slaveindex], tab_reg);

	if (mbfrtn == -1) 
	{
	// Connection closed by the client or error 
	ROS_WARN("stderr: connection failed %s", modbus_strerror(errno));
	continue;
	}

	obj_readinputregisters.read_input_registers.clear();
	for (i=0; i < mbfrtn; i++) {
	obj_readinputregisters.read_input_registers.push_back(tab_reg[i]);
//	ROS_INFO("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
	}

	std_msgs::String msg;

	std::stringstream ss;
	ss << "hello world " <<count;
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());

	//Incrementing the slave id and looping back to start
	mb_read_holding_registers_slaveindex++;
	mb_read_input_registers_slaveindex++;
	//Restting after raching maximum value
	if(mb_read_holding_registers_slaveindex >= read_holding_registers_slaveids.size()) mb_read_holding_registers_slaveindex = 0;
	if(mb_read_input_registers_slaveindex >= read_input_registers_slaveids.size()) mb_read_input_registers_slaveindex = 0;

	chatter_pub.publish(msg);

	readholdingregisters_pub.publish(obj_readholdingregisters);
	readinputregisters_pub.publish(obj_readinputregisters);

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
