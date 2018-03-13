#include "ros/ros.h"
#include "tf/tf.h"
#include "eigen3/Eigen/Core"

#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "visualization_msgs/MarkerArray.h"

class LandmarkVisualizer{
public:
    LandmarkVisualizer(std::string node_name, ros::NodeHandle &nh): node_name_(node_name), nh_(&nh){

        nh_->param<std::string>((node_name_ + "/map_srv"), map_srv_name_, "/gazebo/get_world_properties");
        nh_->param<std::string>((node_name_ + "/landmarks_srv"), lm_srv_name_, "/gazebo/get_model_state");

        // Build world map from Gazebo
        gazebo_client_ = nh_->serviceClient<gazebo_msgs::GetWorldProperties>(map_srv_name_);
        landmarks_client_ = nh_->serviceClient<gazebo_msgs::GetModelState>(lm_srv_name_);

        // Plot map in RVIZ
        vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/rviz/landmarks", 0 );

        // Get list of sim models from Gazebo
        while(!ros::service::waitForService(map_srv_name_, ros::Duration(10)) && ros::ok()){
            ROS_INFO_NAMED(node_name_,"Waiting for the gazebo world prop service to come up");
        }

        // Get states of the models from Gazebo (to build map)
        while(!ros::service::waitForService(lm_srv_name_, ros::Duration(10)) && ros::ok()){
            ROS_INFO_NAMED(node_name_,"Waiting for the gazebo model states service to come up");
        }

        // Build map for localization from Gazebo services and transform to odom frame coordinates
        gazebo_msgs::GetWorldProperties world_prop_srv;
        gazebo_msgs::GetModelState landmark_state_srv;
        if(gazebo_client_.call(world_prop_srv)){
            ROS_INFO("Fetching world from gazebo");
            int id = 0;
            for(auto landmark_name: world_prop_srv.response.model_names){
                // Get poses of all objects except basic setup
                if(landmark_name != "lolo_auv" && landmark_name != "ned" && landmark_name != "ocean" && landmark_name != "dummy_laser"){
                    landmark_state_srv.request.model_name = landmark_name;
                    if(landmarks_client_.call(landmark_state_srv)){
                        ROS_INFO("Fetching landmarks from gazebo");
                        // Test map in world frame. Only for visualization
                        map_world_.push_back(Eigen::Vector4d(id,
                                                            landmark_state_srv.response.pose.position.x,
                                                            landmark_state_srv.response.pose.position.y,
                                                            landmark_state_srv.response.pose.position.z));
                        id++;
                    }
                }
            }
        }
        else{
            ROS_WARN("Couldn't fetch world from Gazebo");
        }

        ros::Rate r(10);
        while(ros::ok()){
            createMapMarkers(map_world_);
            r.sleep();
        }

    }

    void createMapMarkers(std::vector<Eigen::Vector4d> map_world){

        visualization_msgs::MarkerArray marker_array;
        for (auto landmark: map_world){
            visualization_msgs::Marker markers;
            markers.header.frame_id = "world";
            markers.header.stamp = ros::Time();
            markers.ns = "map_array";
            markers.id = landmark(0);
            markers.type = visualization_msgs::Marker::CUBE;
            markers.action = visualization_msgs::Marker::ADD;
            markers.pose.position.x = landmark(1);
            markers.pose.position.y = landmark(2);
            markers.pose.position.z = landmark(3);
            markers.pose.orientation.x = 0.0;
            markers.pose.orientation.y = 0.0;
            markers.pose.orientation.z = 0.0;
            markers.pose.orientation.w = 1.0;
            markers.scale.x = 1;
            markers.scale.y = 1;
            markers.scale.z = 1;
            markers.color.a = 1.0;
            markers.color.r = 0.0;
            markers.color.g = 1.0;
            markers.color.b = 0.0;

            marker_array.markers.push_back(markers);
        }
        vis_pub_.publish(marker_array);

    }

    ~LandmarkVisualizer(){
        delete nh_;
    }

private:
    ros::NodeHandle* nh_;
    std::string node_name_;
    std::string lm_srv_name_;
    std::string map_srv_name_;
    std::vector<Eigen::Vector4d> map_world_;
    ros::ServiceClient gazebo_client_;
    ros::ServiceClient landmarks_client_;
    ros::Publisher vis_pub_;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "landmark_visualized_node");
    ros::NodeHandle nh;

    LandmarkVisualizer lm_vis(ros::this_node::getName(), nh);

    ros::spin();

}

