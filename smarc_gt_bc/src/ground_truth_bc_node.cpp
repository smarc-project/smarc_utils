
/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


/**
 * @brief The GroundTruthBC class
 * This node broadcast the tf world --> map and
 * publishes the AUV ground truth pose from Gazebo
 */

class GroundTruthBC{

public:
    GroundTruthBC(ros::NodeHandle &nh): nh_(&nh){
        start_bc_ = false;

        nh_->param<std::string>((ros::this_node::getName() + "/gt_topic"), gt_topic_, "/pose_gt");
        nh_->param<std::string>((ros::this_node::getName() + "/parent_frame"), world_frame_, "/world");
        nh_->param<std::string>((ros::this_node::getName() + "/map_frame"), map_frame_, "/map");
        nh_->param<std::string>((ros::this_node::getName() + "/base_frame"), base_frame_, "/base_link");
        nh_->param<std::string>((ros::this_node::getName() + "/gt_map_topic"), gt_in_map_tp_, "/gt_in_map");
        nh_->param<bool>((ros::this_node::getName() + "/bc_base_map_tf"), active_nav_flag_, "false");

        initial_pose_subs_ = nh_->subscribe(gt_topic_, 1, &GroundTruthBC::getInitialPoseCB, this);
        gt_map_pub_ = nh_->advertise<nav_msgs::Odometry>(gt_in_map_tp_, 10);

        tf::TransformListener tf_listener;
        try {
            ROS_INFO("Waiting for transform %s --> %s", map_frame_.c_str(), world_frame_.c_str());
            tf_listener.waitForTransform(map_frame_, world_frame_, ros::Time(0), ros::Duration(100.));
			ros::Duration(1.).sleep();
            ROS_INFO("Got transform %s --> %s", map_frame_.c_str(), world_frame_.c_str());
            tf_listener.lookupTransform(map_frame_, world_frame_, ros::Time(0), tf_map_world_);
            ROS_INFO("Locked transform sensor --> frame");
        }
        catch(tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
        }
    }

    void broadcastOdomTf(){
        ros::Rate rate(30.0);
        tf::TransformBroadcaster br_map_pose;
        nav_msgs::OdometryPtr gt_msg;
        geometry_msgs::Pose gt_in_map;
        tf::Pose gt_world;
        nav_msgs::Odometry gt_in_map_msg;

        while (ros::ok()){
            ros::spinOnce();
            if(start_bc_){
                gt_msg = gt_readings_.back();
                tf::poseMsgToTF(gt_msg->pose.pose, gt_world);
                tf::Pose gt_odom =  tf_map_world_ * gt_world;
                gt_odom.getRotation().normalize();
                tf::poseTFToMsg(gt_odom, gt_in_map);

                // Publish GT in map frame
                gt_in_map_msg.header.stamp = gt_msg->header.stamp;
                gt_in_map_msg.header.frame_id = map_frame_;
                gt_in_map_msg.child_frame_id = base_frame_;
                gt_in_map_msg.pose.pose = gt_in_map;
                gt_map_pub_.publish(gt_in_map_msg);

                // If no active navigation nodes, broadcast tf map --> base_link
                if(!active_nav_flag_){
                    geometry_msgs::TransformStamped base_map_tf_bc;
                    base_map_tf_bc.header.stamp = gt_msg->header.stamp;
                    base_map_tf_bc.header.frame_id = map_frame_;
                    base_map_tf_bc.child_frame_id = base_frame_;
                    base_map_tf_bc.transform.translation.x = gt_in_map.position.x;
                    base_map_tf_bc.transform.translation.y = gt_in_map.position.y;
                    base_map_tf_bc.transform.translation.z = gt_in_map.position.z;
                    base_map_tf_bc.transform.rotation = gt_in_map.orientation;
                    br_map_pose.sendTransform(base_map_tf_bc);
                }
            }
            else{
                ROS_DEBUG("Tf world --> map not being broadcasted");
            }
            rate.sleep();
        }
    }

    void getInitialPoseCB(const nav_msgs::OdometryPtr &gt_pose){
        // Fix initial transform world --> odom
        if(start_bc_ == false){
            initial_map_ps_ = gt_pose;
        }
        start_bc_ = true;

        // Collect gt pose from Gazebo
        gt_readings_.push_back(gt_pose);
        unsigned int size_gt_q = 10;
        while(gt_readings_.size() > size_gt_q){
            gt_readings_.pop_front();
        }
    }

private:
    bool start_bc_;
    bool active_nav_flag_;
    std::string world_frame_;
    std::string map_frame_;
    std::string gt_in_map_tp_;
    std::string base_frame_;
    nav_msgs::OdometryPtr initial_map_ps_;
    ros::NodeHandle* nh_;
    std::string gt_topic_;
    ros::Subscriber initial_pose_subs_;
    ros::Publisher gt_map_pub_;
    std::deque<nav_msgs::OdometryPtr> gt_readings_;
    tf::StampedTransform tf_map_world_;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "base_map_tf_bc");
  ros::NodeHandle nh;

  GroundTruthBC world_odom_bc = GroundTruthBC(nh);
  world_odom_bc.broadcastOdomTf();

  return 0;
}
