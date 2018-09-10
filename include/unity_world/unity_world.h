
#ifndef UNITY_WORLD
#define UNITY_WORLD

#include <ros/ros.h>
#include <unity_world/limited_marker_queue.h>
#include <map>
#include <vector>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>

class UnityWorld {
  public:
    UnityWorld();
    ~UnityWorld(){};

    // subscriber callbacks:
    void objectsCallback(const visualization_msgs::MarkerArray &msg);

    // service callbacks:
    bool setupPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool resetPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


  private:
    ros::NodeHandle nh_;

    uint object_lifetime_secs_ = 10;
    uint object_smoothing_queue_length_ = 5;

    std::map<std::string, LimitedMarkerQueue> object_smoothing_queues_;

    // used to transform the recieved markers
    tf::TransformListener collision_object_transform_listener_;

    ros::Subscriber object_sub_; // subscribes to the apriltag objects

    ros::ServiceServer setupPlanningSceneService_;
    ros::ServiceServer resetPlanningSceneService_;

    moveit::planning_interface::PlanningSceneInterface *psi_;

    void remove_collision_objects();
    void add_collision_objects();
    void update_from_planning_scene(bool use_queue);
    void get_collision_objects(std::vector<std::string> collision_object_ids);
    void get_collision_object(std::string collision_object_id);
    bool markerMsgToCollisionObjectMsg(
      visualization_msgs::Marker marker,
      moveit_msgs::CollisionObject &collision_object);

};

#endif
