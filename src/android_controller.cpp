#include <iostream>
#include <cmath>
#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp>
#include <signal.h>

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

void sigint_handler(int sig) {
    ROS_INFO("Exiting");
    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "android_controller_node");
    ros::NodeHandle _nh;

    ros::Publisher speedPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);


    signal(SIGINT, sigint_handler);

    ROS_INFO("Starting server...");

    try
    {
        boost::asio::io_service ioserv;
        tcp::acceptor accptr(ioserv, tcp::endpoint(tcp::v4(), _PORT));

        while(ros::ok()) {

            ROS_INFO("Waiting for client");
            tcp::socket sock(ioserv);
            accptr.accept(sock);
            ROS_INFO("New connection from %s", sock.remote_endpoint().address().to_string().c_str());
            ros::Time start = ros::Time::now();
            bool dropped = false;
            while(sock.is_open() && !dropped) {
                /*if(!sock.available()) {
                    if((ros::Time::now() - start).toSec() > 30) {
                        ROS_WARN("Timeout");
                        dropped = true;
                    }
                    continue;
                }*/

                uint8_t joystick;
                boost::system::error_code ec;
                boost::asio::read(sock, boost::asio::buffer(static_cast<void*>(&joystick), sizeof(char)), ec);
                if(ec == boost::asio::error::eof) {
                    ROS_WARN("Remote closed connection");
                    break;
                }

                uint8_t __angle[8], __power[8];
                boost::asio::read(sock, boost::asio::buffer(static_cast<void*>(__angle), sizeof(__angle)), ec);
                if(ec == boost::asio::error::eof) {
                    ROS_WARN("Remote closed connection");
                    break;
                }
                endianconvert(__angle);
                boost::asio::read(sock, boost::asio::buffer(static_cast<void*>(__power), sizeof(__power)), ec);
                if(ec == boost::asio::error::eof) {
                    ROS_WARN("Remote closed connection");
                    break;
                }
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
            ROS_WARN("Disconnecting");
        }
    }
    catch (std::exception exc)
    {
        ROS_ERROR("Exception thrown: %s", exc.what());
        exit(EXIT_FAILURE);
    }

    return 0;
}
