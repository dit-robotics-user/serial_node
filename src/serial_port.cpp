
#include <ros/ros.h>
#include <serial/serial.h> //ROS已經內置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
//#include "crc.h"

serial::Serial ser; //聲明串口對象
//crc::CRC cc_crc;

unsigned int CRC32(unsigned char *message, unsigned int l)
{
    size_t i, j;
    unsigned int crc, msb;

    crc = 0xFFFFFFFF;
    for (i = 0; i < l; i++)
    {
        // xor next byte to upper bits of crc
        crc ^= (((unsigned int)message[i]) << 24);
        for (j = 0; j < 8; j++)
        { // Do eight times.
            msb = crc >> 31;
            crc <<= 1;
            crc ^= (0 - msb) & 0x04C11DB7;
        }
    }
    return crc;
}

//回調函數
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data); //發送串口數據
}
// unsigned int crc32b(unsigned char *message, size_t l)
// {
//     size_t i, j;
//     unsigned int crc, msb;

//     crc = 0xFFFFFFFF;
//     for (i = 0; i < l; i++)
//     {
//         // xor next byte to upper bits of crc
//         crc ^= (((unsigned int)message[i]) << 24);
//         for (j = 0; j < 8; j++)
//         { // Do eight times.
//             msb = crc >> 31;
//             crc <<= 1;
//             crc ^= (0 - msb) & 0x04C11DB7;
//         }
//     }
//     return crc; // don't complement crc on output
// }
int main(int argc, char **argv)
{
    //初始化節點
    ros::init(argc, argv, "serial_example_node");
    //聲明節點句柄
    ros::NodeHandle nh;
    //訂閱主題，並配置回調函數
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //發佈主題
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    ros::Publisher write_pub = nh.advertise<std_msgs::String>("write", 1000);

    try
    {
        //設置串口屬性，並打開串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
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
    ser.write("021050332341");
    // crc32
    uint32_t tx[12] = {0, 9, 3, 4, 2, 0, 3, 6, 6, 7, 8, 7};
    //uint8_t tx_char[48] = (uint8_t*)tx;
    uint8_t *crc = new uint8_t[12];
    bool sent = 0;
    while (ros::ok())
    {
        if (ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port\n");
            std_msgs::String result;
            std_msgs::String send;
            result.data = ser.readline(ser.available());
            //string indata = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            //std::cout << crc32b((uint8_t*)result.data, sizeof((uint8_t*)result.data)) << "\n";
            read_pub.publish(result);

            //write
            send.data = ser.write("093420366787");
            sent = 1;
            if (sent == 1)
            {
                ROS_INFO_STREAM("Sending to serial port\n");
                ROS_INFO_STREAM("Send: " << send.data);
                write_pub.publish(send);
                sent = 0;
                //std::cout << send.data;
            }

            //
        }

        //處理ROS的信息，比如訂閱消息,並調用回調函數
        ros::spinOnce();
        loop_rate.sleep();
    }
}
