
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
#include <moveit_msgs/AttachedCollisionObject.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
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
    std::string object_frame_id_ = "/odom_combined";
    bool publish_collision_object_marker_ = true;

    std::map<int, LimitedMarkerQueue> object_smoothing_queues_;

    // used to transform the recieved markers
    tf::TransformListener collision_object_transform_listener_;
    tf::TransformBroadcaster collision_object_transform_broadcaster_;

    ros::Subscriber object_sub_; // subscribes to the apriltag objects

    ros::ServiceServer setupPlanningSceneService_;
    ros::ServiceServer resetPlanningSceneService_;

    ros::Timer publishing_timer_;
    ros::Publisher collision_object_publisher_;
    ros::Publisher marker_publisher_;

    moveit::planning_interface::PlanningSceneInterface *psi_;

    bool publish_from_psi_;

    void publishing_timer_callback(const ros::TimerEvent&);
    void remove_collision_objects();
    void add_collision_objects();
    void get_update_from_planning_scene();
    bool collisionObjectMsgToMarkerMsg(
      moveit_msgs::CollisionObject collision_object,
      visualization_msgs::Marker &marker);

};

#endif
