
#ifndef UNITY_WORLD
#define UNITY_WORLD

#include <ros/ros.h>
#include <map>
#include <vector>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <apriltag_object_detection/marker_to_collision_object.h>
#include <pr2_phantom/State.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

class UnityWorld {
  public:
    UnityWorld();
    ~UnityWorld(){};

    void objectsCallback(const visualization_msgs::MarkerArray &msg);
    void stateCallback(const pr2_phantom::State &msg);
    void planPickCallback(const geometry_msgs::PointStamped &msg);
    void planPlaceCallback(const geometry_msgs::PointStamped &msg);


  private:
    ros::NodeHandle nh_;

    uint object_lifetime_secs_ = 10;
    uint object_smoothing_queue_length = 5;

    std::vector<std::pair<std::string, visualization_msgs::Marker>> // TODO: List of queues!!!

    tf::TransformListener collision_object_transform_listener_;
    
    ros::Subscriber object_sub_; // subscribes to the apriltag objects
    ros::Subscriber state_sub_; // subscribes to the state of the pick-place node
    ros::Subscriber plan_pick_sub_;
    ros::Subscriber plan_place_sub_;
    
    moveit::planning_interface::PlanningSceneInterface *psi_;

    void remove_collision_objects();
    void add_collision_objects();
    void update from planning_scene();
    void get_collision_objects(std::vector<std::string> collision_object_ids);
    void get_collision_object(std::string collision_object_id);

};

#endif
