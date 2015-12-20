#include <iostream>
#include <cmath>
#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <signal.h>
#include <deque>
#include <cmath>

using namespace std;
using boost::asio::ip::tcp;

struct data_t
{
    uint8_t joystick;
    double power;
    double angle;
    ros::Time time;
};

geometry_msgs::Twist message;

boost::mutex queue_mutex;
boost::mutex connection_mutex;
bool connected = false;
boost::thread* worker_thread;

const short _PORT = 7575;

const double PI  = M_PI;
const double PI_2 = PI / 2;
const double PI_4 = PI / 4;
const double PI_3_4 =  3 * PI / 4;
const double straight_low_point =0.1;
const double straight_high_point =0.7;


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
    connection_mutex.lock();
    connected = false;
    connection_mutex.unlock();
    if(worker_thread != NULL) {
        worker_thread->join();
    }
    exit(EXIT_SUCCESS);
}

void execute_worker(deque<data_t> *process_queue, ros::Publisher* speedPub)
{
    double target_distance = 0.0;
    double target_angle = 0.0;
    ros::Time previous_time;
    while(true){
        connection_mutex.lock();
        if(!connected){
            connection_mutex.unlock();
            break;
        }
        connection_mutex.unlock();

        data_t current_data;

        queue_mutex.lock();
        if(!process_queue->empty()){
            current_data = process_queue->front();
            process_queue->pop_front();
        }
        else{
            queue_mutex.unlock();
            continue;
        }
        queue_mutex.unlock();

        if((ros::Time::now() - current_data.time).toSec() > 30)
            continue;

        if(current_data.joystick == 1) {
            if(!current_data.power == 0.0) {
                if(current_data.angle < PI_2 && current_data.angle > -PI_2 ) {
                    message.linear.x = current_data.power  / 200;
                    message.linear.y = 0;
                    message.linear.z = 0;

                    message.angular.x = 0;
                    message.angular.y = 0;
                    message.angular.z = current_data.angle * 0.5;
                }
                else {
                    message.linear.x =  -(current_data.power  / 200);
                    message.linear.y = 0;
                    message.linear.z = 0;
                    if(current_data.angle < 0) {
                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = -PI - current_data.angle;
                    }
                    else {
                        message.angular.x = 0;
                        message.angular.y = 0;
                        message.angular.z = PI - current_data.angle;
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
        else if(current_data.joystick == 2) {
            if(current_data.angle < PI_4 && current_data.angle > -PI_4) {
                message.linear.x = 0;
                message.linear.y = 0;
                message.linear.z = current_data.power / 200;

                message.angular.x = 0;
                message.angular.y = 0;
                message.angular.z = 0;
            }
            else if((current_data.angle > PI_3_4 && current_data.angle < PI)
                    || (current_data.angle < -PI_3_4 && current_data.angle > -PI) ) {
                message.linear.x = 0;
                message.linear.y = 0;
                message.linear.z = - current_data.power / 200;

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
                message.angular.z = (current_data.angle * current_data.power) / 200 ;
            }
        }
        else if(current_data.joystick == 3) {
            target_distance = current_data.power;
            ros::Rate frequency(10);
            while(true){
                double time_ellapsed = (ros::Time::now()-previous_time).toSec();
                double new_target_distance = target_distance - time_ellapsed*message.linear.x;
                message.linear.x = min(straight_high_point,max(straight_low_point,new_target_distance / 10));
                message.linear.y = 0;
                message.linear.z = 0;

                message.angular.x = 0;
                message.angular.y = 0;
                message.angular.z = 0 ;
                if(new_target_distance*target_distance<0){
                    target_distance=0;
                    break;
                }
                speedPub->publish(message);
                frequency.sleep();
            }
            message.linear.x = 0;
        }
        speedPub->publish(message);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "android_controller_node");
    ros::NodeHandle _nh;
    ros::Publisher speedPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    signal(SIGINT, sigint_handler);

    deque<data_t> process_queue;
    worker_thread = NULL;

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

            connection_mutex.lock();
            connected=true;
            connection_mutex.unlock();
            worker_thread = new boost::thread(execute_worker, &process_queue, &speedPub);

            while(sock.is_open()) {

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

                data_t data;
                data.joystick=joystick;
                data.angle=angle;
                data.power=power;
                data.time=ros::Time::now();
                queue_mutex.lock();
                process_queue.push_back(data);
                queue_mutex.unlock();
                start = ros::Time::now();
            }
            sock.close();
            connection_mutex.lock();
            connected=false;
            connection_mutex.unlock();
            ROS_WARN("Disconnecting");
            worker_thread->join();
            delete worker_thread;
            worker_thread = NULL;
        }
    }
    catch (std::exception exc)
    {
        ROS_ERROR("Exception thrown: %s", exc.what());
        exit(EXIT_FAILURE);
    }

    return 0;
}
