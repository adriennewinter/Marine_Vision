// This code is written in C++ and is used to communicate with an I2C slave device. 
// It uses the ROS library to publish the data from the I2C slave device to a topic. 
// The code opens the I2C port, sets the port options, and sets the address of the device. 
// It then writes a command to the device and reads the data from the device. 
// The data is then published to a topic using the ROS library. Finally, the I2C port is closed.

//pin used for EZO-PRS pressure sensor I2C on Jetson Nano:
///                    green --> pin 3
///		       white --> pin 5
///                    red   --> pin 4 (5v)
///                    black --> pin 6 (gnd)


#include <linux/i2c-dev.h>  //library for I2C device
#include <sys/ioctl.h>      //library for ioctl function
#include <fcntl.h>          //library for open function
#include <unistd.h>         //library for close function
#include <cstdio>           //library for printf function
#include <iostream>         //library for I/O operations
#include <cstdio>           //library for standard input/output operations
#include <assert.h>         //library for program assertion
#include <stdarg.h>         //library for variable argument lists
#include <string>           //library for string type
#include <cstring>          //library for c string functions
#include <chrono>           //library for time operations
#include <thread>           //library for thread operations

#include <ros/ros.h>         //library for ROS
#include <sensor_msgs/FluidPressure.h> //library for ROS pressure sensor messages

#include <sstream>          //library for string stream operations

using namespace std;

#define snprintf _snprintf  //macro for snprintf function
#define exit _exit          //macro for exit function

int main(int argc, char **argv)
{
    //declare variables for use in the program
    int file;
    int adapter_nr = 1;
    char filename[12];
    int slaveAddress = 0x6A; // I2C slave address
    char writeBuffer[1] = {'r'}; // Buffer containing command
    std::string find = "find";
    char find_array[find.length() + 1];
    strcpy(find_array, find.c_str());
    char buffer[22];
    char PRS_data[20];
    double PRS_data_float;

    //open port for reading and writing
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0)
    {
       printf("Failed to open i2c port\n");
       exit(1);
    }

    //set the port options and set the address of the device
    if (ioctl(file, I2C_SLAVE, slaveAddress) < 0)
    {
       printf("Unable to get bus access to talk to slave\n");
       exit(1);
    }

    //write command to the device
    if (write(file, writeBuffer, 1) != 1)
    {
       printf("Error writing to i2c slave\n");
       exit(1);
    }
     
    //initialize ROS node
    ros::init(argc, argv, "ezo_prs_pressure");
    ros::NodeHandle nh;
    ros::Publisher ezo_prs_pub = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1000);
    ros::Rate loop_rate(10);

    int count = 0; //declare variable to count messages
    while (ros::ok())
    {
        //create message object
	sensor_msgs::FluidPressure msg;

        //create string stream object
        std::stringstream ss;

        //loop to read data from I2C
        while (1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            read(file, buffer, 20);
            if(buffer[0] == 1)
            {
                int i = 0;
                while(buffer[i+1]!= 0)
                {
                    //load data into PRS_data array
                    PRS_data[i] = buffer[i+1];            
                    i += 1; //increment counter for data array element
                }
		PRS_data_float = atof(PRS_data);
                if (write(file, writeBuffer, 1) != 1)
                {
                    printf("Error writing to i2c slave\n");
                    exit(1);
                }
                break;
            }
        }
        //add data to message
	msg.fluid_pressure = PRS_data_float;
	
	// populate timestamp in header field
	msg.header.stamp = ros::Time::now();

        //print message
        ROS_INFO("ezo_prs = (%f) kPa", msg.fluid_pressure);

        //publish message
        ezo_prs_pub.publish(msg);

        //run ROS loop
        ros::spinOnce();

        //wait for the specified rate
        loop_rate.sleep();
        ++count;

    }
	close(file);
	return 0;
}
