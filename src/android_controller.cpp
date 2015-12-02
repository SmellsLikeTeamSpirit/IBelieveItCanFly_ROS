#include <iostream>
#include <cmath>
#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp>



using namespace std;
using boost::asio::ip::tcp;

geometry_msgs::Twist message;
const short _PORT = 7575;

const double PI  = M_PI;
const double PI_2 = PI / 2;
const double PI_4 = PI / 4;
const double PI_3_4 =  3 * PI / 4;

void endianconvert(uint8_t *in) {
    uint8_t tmp[8];
    for (short i = 0; i < 8; ++i) {
        tmp[7-i] = in[i];
    }
    for (short i = 0; i < 8; ++i) {
        in[i] = tmp[i];
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "android_controller_node");
    ros::NodeHandle _nh;

    ros::Publisher speedPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    try
    {
        boost::asio::io_service ioserv;
        tcp::acceptor accptr(ioserv, tcp::endpoint(tcp::v4(), _PORT));

        while(ros::ok()) {
            tcp::socket sock(ioserv);
            accptr.accept(sock);
            ros::Time start = ros::Time::now();
            bool dropped = false;
            while(sock.is_open() && !dropped) {
                //sock.write_some(buffer(sense, sizeof(sense)));
                if(!sock.available()) {
                    //char sense[5] = "rual";

                    if((ros::Time::now() - start).toSec() > 30)
                        dropped = true;
                    continue;
                }

                uint8_t joystick;
                sock.read_some(boost::asio::buffer(static_cast<void*>(&joystick), sizeof(char)));
                uint8_t __angle[8], __power[8];
                sock.read_some(boost::asio::buffer(static_cast<void*>(__angle), sizeof(__angle)));
                endianconvert(__angle);
                sock.read_some(boost::asio::buffer(static_cast<void*>(__power), sizeof(__power)));
                endianconvert(__power);
                double angle = *((double*)__angle);
                double power = *((double*)__power);
                ROS_INFO("From jostick(%d) Angle : %f Power %f", (int)joystick, angle, power);
                if(joystick == 1) {
                    if(!power == 0.0) {
                        if(angle < PI_2 && angle > -PI_2 ) {
                            message.linear.x = power / 200;
                            message.linear.y = 0;
                            message.linear.z = 0;

                            message.angular.x = 0;
                            message.angular.y = 0;
                            message.angular.z = angle * 0.5;
                        }
                        else {
                            message.linear.x =  -(power / 200);
                            message.linear.y = 0;
                            message.linear.z = 0;
                            if(angle < 0) {
                                message.angular.x = 0;
                                message.angular.y = 0;
                                message.angular.z = -PI - angle;
                            }
                            else {
                                message.angular.x = 0;
                                message.angular.y = 0;
                                message.angular.z = PI - angle;
                            }
                        }
                    }
                    else {
                        message.linear.x = 0;
                        message.linear.y = 0;
                        message.linear.z = 0;

                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = 0;
                    }
                }
                else if(joystick == 2) {
                    if(angle < PI_4 && angle > -PI_4) {
                        message.linear.x = 0;
                        message.linear.y = 0;
                        message.linear.z = power / 200;

                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = 0;
                    }
                    else if((angle > PI_3_4 && angle < PI) || (angle < -PI_3_4 && angle > -PI) ) {
                        message.linear.x = 0;
                        message.linear.y = 0;
                        message.linear.z = - power / 200;

                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = 0;
                    }
                    else {
                        message.linear.x = 0;
                        message.linear.y = 0;
                        message.linear.z = 0;

                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = (angle * power) / 200 ;
                    }
                }
                speedPub.publish(message);
                start = ros::Time::now();
            }
            sock.close();
        }
    }
    catch (std::exception exc)
    {

    }

    return 0;
}
