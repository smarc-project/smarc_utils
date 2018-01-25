#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdomTfBC{

public:
    OdomTfBC(ros::NodeHandle &nh): nh_(&nh){
        start_bc_ = false;

        nh_->param<std::string>((ros::this_node::getName() + "/gt_topic"), gt_topic_, "/pose_gt");
        nh_->param<std::string>((ros::this_node::getName() + "/parent_frame"), parent_frame_, "/world");
        nh_->param<std::string>((ros::this_node::getName() + "/child_frame"), child_frame_, "/odom");
        nh_->param<std::string>((ros::this_node::getName() + "/base_frame"), base_frame_, "/base_link");
        nh_->param<std::string>((ros::this_node::getName() + "/gt_odom_topic"), gt_in_odom_tp_, "/gt_in_odom");
        
        initial_pose_subs_ = nh_->subscribe(gt_topic_, 1, &OdomTfBC::getInitialPoseCB, this);
        gt_odom_pub_ = nh_->advertise<nav_msgs::Odometry>(gt_in_odom_tp_, 10);
    }

    void broadcastOdomTf(){
        ros::Rate rate(30.0);
        tf::TransformBroadcaster br;
        tf::Transform tf_world_odom;
        
        nav_msgs::OdometryPtr gt_msg;
        geometry_msgs::Pose gt_in_odom;
        tf::Pose gt_world;
                
        while (ros::ok()){
            ros::spinOnce();
            if(start_bc_){
                // Construct transform world --> odom
                tf_world_odom.setOrigin(tf::Vector3(
                                         initial_odom_ps_->pose.pose.position.x,
                                         initial_odom_ps_->pose.pose.position.y,
                                         initial_odom_ps_->pose.pose.position.z));
                tf::Quaternion q_auv;
                tf::quaternionMsgToTF(initial_odom_ps_->pose.pose.orientation, q_auv);
                tf_world_odom.setRotation(q_auv);
                // Broadcast
                tf::StampedTransform tf_world_odom_stp = tf::StampedTransform(tf_world_odom, 
                                                                            ros::Time::now(), 
                                                                            parent_frame_, 
                                                                            child_frame_);
                br.sendTransform(tf_world_odom_stp);

                // Provide ground truth pose in odom frame
                gt_msg = gt_readings_.back();

                tf::poseMsgToTF(gt_msg->pose.pose, gt_world);
                tf::Pose gt_odom =  tf_world_odom.inverse() * gt_world;
                tf::poseTFToMsg(gt_odom, gt_in_odom);

                nav_msgs::Odometry gt_in_odom_msg;
                gt_in_odom_msg.header.stamp = gt_msg->header.stamp;
                gt_in_odom_msg.header.frame_id = child_frame_;
                gt_in_odom_msg.child_frame_id = "/lolo_auv/base_link";
                gt_in_odom_msg.pose.pose = gt_in_odom;
                gt_odom_pub_.publish(gt_in_odom_msg);
            }
            else{
                ROS_DEBUG("Tf world --> odom not being broadcasted");
            }
            rate.sleep();
        }
    }

    void getInitialPoseCB(const nav_msgs::OdometryPtr &gt_pose){
        // Fix initial transform world --> odom
        if(start_bc_ == false){
            initial_odom_ps_ = gt_pose;
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
    std::string parent_frame_;
    std::string child_frame_;
    std::string gt_in_odom_tp_;
    std::string base_frame_;
    nav_msgs::OdometryPtr initial_odom_ps_;
    ros::NodeHandle* nh_;
    std::string gt_topic_;
    ros::Subscriber initial_pose_subs_;
    ros::Publisher gt_odom_pub_;
    std::deque<nav_msgs::OdometryPtr> gt_readings_;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_broadcaster");
  ros::NodeHandle nh;

  OdomTfBC world_odom_bc = OdomTfBC(nh);
  world_odom_bc.broadcastOdomTf();

  return 0;
}
