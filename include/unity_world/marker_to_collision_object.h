#ifndef MARKER_TO_COLLISION_MODEL
#define MARKER_TO_COLLISION_MODEL

#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

bool markerMsgToCollisionObjectMsg(
    visualization_msgs::Marker marker,
    moveit_msgs::CollisionObject &collision_object,
    tf::TransformListener *listener);

bool markerMsgToCollisionObjectMsg(
    visualization_msgs::Marker marker, std::string frame_id,
    moveit_msgs::CollisionObject &collision_object,
    tf::TransformListener *listener);

#endif
