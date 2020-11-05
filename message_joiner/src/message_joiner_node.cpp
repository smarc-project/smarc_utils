#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cstdlib>
#include <unordered_map>

#include <sam_msgs/CircuitStatusStamped.h>
#include <sam_msgs/CircuitStatusStampedArray.h>


class CircuitStatusJoiner2 {
  std::unordered_map<unsigned, sam_msgs::CircuitStatusStamped> circuit_registry;
  ros::Publisher CircuitStatusPublisher;
  ros::Subscriber CircuitStatusSubscriber;
  ros::Timer timer1;
  unsigned pub_freq;

  void callbackPublish(const ros::TimerEvent&)
  {
    // ROS_INFO("Callback 1 triggered");
    sam_msgs::CircuitStatusStampedArray msg;
    for(int i = 0; i < 100; i++)
    {
      if (circuit_registry.find(i) != circuit_registry.end())
        {
          msg.array.push_back(circuit_registry[i]);
        }
    }
    CircuitStatusPublisher.publish(msg);
  }

  void callbackSubscriber(const sam_msgs::CircuitStatusStamped& msg) {
    // ROS_INFO("New message received. id: %d", msg.circuit.circuit_id);
    circuit_registry[msg.circuit.circuit_id] = msg;
  }

  public:
  CircuitStatusJoiner2(ros::NodeHandle *nh) 
    : pub_freq(2)
    {
    CircuitStatusPublisher = nh->advertise<sam_msgs::CircuitStatusStampedArray>("circuit_status_array", 10);    
    CircuitStatusSubscriber = nh->subscribe("circuit_status", 30, 
        &CircuitStatusJoiner2::callbackSubscriber, this);

    timer1 = nh->createTimer(ros::Duration(1.0/pub_freq), &CircuitStatusJoiner2::callbackPublish, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "message_joiner");

  ros::NodeHandle nh;

  // int pub_frequency;
  // ros::param::param<int>("pub_frequency", pub_frequency, 2);

  ROS_INFO("Message joiner started");

  CircuitStatusJoiner2 nc = CircuitStatusJoiner2(&nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }


  return 0;
}