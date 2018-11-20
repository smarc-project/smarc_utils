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

#include "ros/ros.h"
#include "tf/tf.h"
#include "eigen3/Eigen/Core"

#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseArray.h>
#include "smarc_lm_visualizer/init_map.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


class LandmarkVisualizer{
public:
    LandmarkVisualizer(std::string node_name, ros::NodeHandle &nh): node_name_(node_name), nh_(&nh){

        nh_->param<std::string>((node_name_ + "/map_srv"), map_srv_name_, "/gazebo/get_world_properties");
        nh_->param<std::string>((node_name_ + "/landmarks_srv"), lm_srv_name_, "/gazebo/get_model_state");
        nh_->param<std::string>((node_name_ + "/landmark_mesh"), lm_mesh_, "package://smarc_worlds/world_models/large_rock/meshes/large_rock.dae");

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

        // Parse beacons names
        std::string delimiter = "_";

        if(gazebo_client_.call(world_prop_srv)){
            int id = 0;
            for(auto landmark_name: world_prop_srv.response.model_names){
                if(landmark_name.substr(0, landmark_name.find(delimiter)) == "lolo_auv"){
                    continue;
                }
                if(landmark_name.substr(0, landmark_name.find(delimiter)) == "sam_auv"){
                    continue;
                }
                // Get poses of all objects except basic setup
                if(landmark_name != "pipe_line" && landmark_name != "ned" && landmark_name != "ocean" && landmark_name != "dummy_laser"){
                    landmark_state_srv.request.model_name = landmark_name;
                    if(landmarks_client_.call(landmark_state_srv)){
                        // Test map in world frame. Only for visualization
                        map_world_.push_back(Eigen::Vector4d(id,
                                                            landmark_state_srv.response.pose.position.x,
                                                            landmark_state_srv.response.pose.position.y,
                                                            landmark_state_srv.response.pose.position.z));
                        id++;
                    }
                }
                if(landmark_name.substr(0, landmark_name.find(delimiter)) == "beacon"){
                    map_artificial_.push_back(Eigen::Vector3d(landmark_state_srv.response.pose.position.x,
                                                              landmark_state_srv.response.pose.position.y,
                                                              landmark_state_srv.response.pose.position.z));
                }
            }
        }
        else{
            ROS_WARN("Couldn't fetch world from Gazebo");
        }

        map_server_ = nh_->advertiseService("/rviz/map_server", &LandmarkVisualizer::map_service_cb, this);

        ros::Rate r(10);
        while(ros::ok()){
            createMapMarkers(map_world_);
            ros::spinOnce();
            r.sleep();
        }

    }

    bool map_service_cb(smarc_lm_visualizer::init_mapRequest &req,
                        smarc_lm_visualizer::init_mapResponse &res){

//        ROS_INFO_NAMED(node_name_, "Map provider service called");
        if(req.request_map == true){
            int i = 0;
            res.init_map.header.stamp = ros::Time::now();
            res.init_map.header.frame_id = "world";
            geometry_msgs::Point point;
            geometry_msgs::Pose pose;
            for(auto landmark: map_artificial_){
                point.x = landmark(0);
                point.y = landmark(1);
                point.z = landmark(2);
                pose.position = point;
                res.init_map.poses.push_back(pose);
            }
            return true;
        }
        else{
            return false;
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
            markers.type = visualization_msgs::Marker::MESH_RESOURCE;
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
            markers.mesh_resource = lm_mesh_;

            marker_array.markers.push_back(markers);
        }
        vis_pub_.publish(marker_array);

    }

    ~LandmarkVisualizer(){
        delete nh_;
    }

private:
    std::string node_name_;

    ros::NodeHandle* nh_;
    std::string lm_srv_name_;
    std::string map_srv_name_;
    std::vector<Eigen::Vector4d> map_world_;
    std::vector<Eigen::Vector3d> map_artificial_;
    ros::ServiceClient gazebo_client_;
    ros::ServiceClient landmarks_client_;
    ros::Publisher vis_pub_;
    ros::ServiceServer map_server_;
    std::string lm_mesh_;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "landmark_visualized_node");
    ros::NodeHandle nh;

    LandmarkVisualizer lm_vis(ros::this_node::getName(), nh);

}

