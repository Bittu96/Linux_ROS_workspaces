#include <ros/ros.h>
#include <modbustest/writemultipleregisters.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "write_multiple_registers_client");
  if (argc < 4)
  {
    ROS_INFO("usage: write_multiple_registers_client slaveid count start_address values");
    return 1;
  }

  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<modbustest::writemultipleregisters>("modbus_writemultipleregisters");
  modbustest::writemultipleregisters srv;

  srv.request.slaveid = atol(argv[1]);
  srv.request.write_multiple_registers_count = atol(argv[2]);
  srv.request.write_multiple_registers_start_address  = atol(argv[3]);

  srv.request.write_multiple_registers.clear();

  for(int i=4; i<argc;i++)
  {
	long int data = atol(argv[i]);
	srv.request.write_multiple_registers.push_back(data);
	ROS_INFO("(%i) Data: %ld", i, data);
  }

  if (client.call(srv))
  {
    ROS_INFO("Response: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
