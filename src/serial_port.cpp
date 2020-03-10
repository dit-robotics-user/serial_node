
#include <ros/ros.h>
#include <serial/serial.h> //ROS已經內置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <stdlib.h>
#include <time.h>
#include <string.h>
//#include "crc.h"

using namespace std;

serial::Serial ser; //聲明串口對象
//crc::CRC cc_crc;

/*void delay(int duration)
{
    clock_t start_time = clock();
    while (clock() < start_time + duration)
        ;
}*/

// crc only for stm
uint32_t crc32_mpeg_2(uint8_t *data, uint8_t length)
{
    uint8_t i;
    int8_t j=3;
    uint32_t crc = 0xffffffff;  // Initial value

    while(length--)
    {
        crc ^= (uint32_t)(*(j+ data++)) << 24;// crc ^=(uint32_t)(*data)<<24; data++;
        j-=2;
        if(j==-5) j=3;
        for (i = 0; i < 8; ++i)
        {
            if ( crc & 0x80000000 )
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void tx_init(int32_t* tx)
{
    tx[0] = -6000;
    tx[1] = 123;
    tx[2] = -456;
    tx[3] = 789;
    tx[4] = crc32_mpeg_2((uint8_t*)tx, 16);
}

// int crc32_check(uint32_t rx_in_crc, uint32_t rx_self_crc)
// {
//     int count;
//     int errorcount;
//     float rate;
// }

//回調函數
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data); //發送串口數據
}

int main(int argc, char **argv)
{
    //初始化節點
    ros::init(argc, argv, "serial_node");
    //聲明節點句柄
    ros::NodeHandle nh;
    //訂閱主題，並配置回調函數
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //發佈主題
    ros::Publisher read_pub = nh.advertise<std_msgs::Int32MultiArray>("read", 1000);

    ros::Publisher write_pub = nh.advertise<std_msgs::String>("write", 1000);
    
    ros::Publisher rate_pub = nh.advertise<std_msgs::Int32>("success_rate", 1000);

    try
    {
        //設置串口屬性，並打開串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(12);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //檢測串口是否已經打開，並給出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //指定循環的頻率
    ros::Rate loop_rate(100);
    //ser.write("12345");
    // crc32
    //
    //uint32_t tx[mode, R1, R2, R3, crc];
    // mode(hex):
    //  - 1000: set position;   R1=x,       R2=y,       R3=theta
    //  - 2000: run script;     R1=num_of_script
    //  - 300x: speed mode;     R1=speed,   R2=theta,   x=dir(1:fwd, 2:bwd, 0:auto)
    //  - 40xx: pos mode;       R1=x,       R2=y,       R3=theta,       TD=turn_dir(1:L, 2:R, 3:auto),      LD=lin_dir(1:fwd, 2:bwd, 3:auto)   
    //  - 5000: stop;
    //  - 6000: idle, disable motor driver;
    int32_t tx[5]; //37 30 32 32 39
    tx_init(tx);
    int32_t rx[100];
    // [xxxxx, yyyyy, tttt, cccc_cccc_cc, (\n)]
    char tx_arr[10];
    sprintf(tx_arr, "%u", tx[0]/*, tx[1], tx[2], tx[3], tx[4]*/);
    //uint8_t tx_char[48] = (ui                                                                  nt8_t*)tx;
    uint8_t *crc = new uint8_t[12];
    bool sent = 0;
    int count = 0;
	int len = 4;
	uint32_t indata[6] = {0};
	int32_t tmp;
	
	int rcv_count = 0;
	int error_count = 0;
	int success_rate = 0;
	int transmit_param = 0;
	char* location;
	//int delay_ms = 10;
	
    while (ros::ok())
    {
        if (ser.available()>=24)
        {
            
            //ROS_INFO_STREAM("Reading from serial port\n");
            //std_msgs::String rx_str;
            std_msgs::Int32MultiArray rx_str;
			string test;
            test = ser.readline(24);
            
			rx_str.data.clear();
			
			for(int i=0; i<len+1; i++){
				tmp = (int32_t)test[4*i] + (int32_t)test[4*i+1]*256 +
					(int32_t)test[4*i+2]*65536 +(int32_t)test[4*i+3]*16777216;
				//tmp = (int32_t) test[i];				
				rx_str.data.push_back(tmp);
				indata[i] = tmp;
			}
			/*for (int i = 0; i < 50; i++) {
			    std::cout << dec << indata[i] << " ";
			    
			}
			std::cout << "\n";*/
			if (count > 50) {
			    success_rate = (int)(100*(rcv_count-error_count)/rcv_count);
			    rcv_count = 0;
			    error_count = 0;
			    count = 0;
			}
			
			if (indata[4] == crc32_mpeg_2((uint8_t*)indata, 16)) {
			    rcv_count ++;
			    //ROS_INFO_STREAM("Read: " << test);
                read_pub.publish(rx_str);
			}
			else {
			    rcv_count ++;
			    error_count++;
			}
			
        } else {
            transmit_param++;
        }
        
        if (count >= 0) {
            tx[0] ++;
            tx[1] ++;
            tx[2] ++;
            tx[3] = transmit_param;
            tx[4] =crc32_mpeg_2((uint8_t*)tx, 16);
            //count = 0;
        }
       
        //ROS_INFO_STREAM("CRC = " << tx[4]);
        std_msgs::String tx_str;
        //if (transmit_param == 2) {
        tx_str.data = ser.write((const uint8_t*)tx, sizeof(tx)+3);
        write_pub.publish(tx_str);
        //transmit_param = 0;
        //}//ROS_INFO_STREAM("Sending to serial port\n");
        //ROS_INFO_STREAM("Send: " << 
        //    tx[0] << " " << 
        //    tx[1] << " " << 
        //    tx[2] << " " << 
        //    tx[3] << " " <<
        //    tx[4]);
        
        
        // publish success rate
        std_msgs::Int32 success_rate_msg;
        success_rate_msg.data = success_rate;
        rate_pub.publish(success_rate_msg);
        count ++;
        //處理ROS的信息，比如訂閱消do息,並調用回調函數
        ros::spinOnce();
        loop_rate.sleep();
        //delay(delay_ms);
    }
}
