
#ifndef LIMITED_MARKER_QUEUE
#define LIMITED_MARKER_QUEUE

#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iterator>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

class LimitedMarkerQueue {
  public:
    // option for later: LimitedMarkerQueue(int max_length);
    LimitedMarkerQueue();
    LimitedMarkerQueue(int max_length, ros::Duration element_lifetime);
    ~LimitedMarkerQueue() {};

    void push(visualization_msgs::Marker marker);
    bool get_mean_marker(visualization_msgs::Marker &mean_marker);
    bool get_mean_collision_object(moveit_msgs::CollisionObject &mean_collision_object);
    void reset();

  private:

    int max_length_;
    bool queue_changed_;
    ros::Duration element_lifetime_;
    std::list<visualization_msgs::Marker> marker_list_;
    visualization_msgs::Marker buffer_marker_;
    moveit_msgs::CollisionObject buffer_collision_object_;

    void update_marker_list();
    bool markerMsgToCollisionObjectMsg(
      visualization_msgs::Marker marker,
      moveit_msgs::CollisionObject &collision_object);
};


#endif

