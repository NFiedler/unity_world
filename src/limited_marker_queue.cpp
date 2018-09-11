#include <unity_world/limited_marker_queue.h>

LimitedMarkerQueue::LimitedMarkerQueue() {
  LimitedMarkerQueue(5, ros::Duration(10));
}

LimitedMarkerQueue::LimitedMarkerQueue(int max_length, ros::Duration element_lifetime) : max_length_(max_length), element_lifetime_(element_lifetime), marker_list_(), queue_changed_(true) {}


void LimitedMarkerQueue::push(visualization_msgs::Marker marker) {
  // add element and possibly invalidate the list
  marker_list_.push_back(marker);
  queue_changed_ = true;
}

bool LimitedMarkerQueue::get_mean_marker(visualization_msgs::Marker &mean_marker){
  // update the marker list before accessing it
  update_marker_list();
  int marker_list_size = marker_list_.size();
  if (marker_list_size < 1) {
    return false;
  }
  if (!queue_changed_) {
    mean_marker = visualization_msgs::Marker(buffer_marker_);
    return true;
  }


  // reset the marker by setting it to the most recent element in the queue.
  mean_marker = marker_list_.back();

  // setting up the iterator to ignore the last element of the list.
  std::list<visualization_msgs::Marker>::iterator pre_last = marker_list_.end();
  pre_last--;

  // iterate through list
  for (std::list<visualization_msgs::Marker>::iterator marker_iterator = marker_list_.begin(); marker_iterator != pre_last; marker_iterator++) {
    // position
    mean_marker.pose.position.x += marker_iterator->pose.position.x;
    mean_marker.pose.position.y += marker_iterator->pose.position.y;
    mean_marker.pose.position.z += marker_iterator->pose.position.z;

    // orientation
    mean_marker.pose.orientation.x += marker_iterator->pose.orientation.x;
    mean_marker.pose.orientation.y += marker_iterator->pose.orientation.y;
    mean_marker.pose.orientation.z += marker_iterator->pose.orientation.z;
    mean_marker.pose.orientation.w += marker_iterator->pose.orientation.w;

    // the shape etc. is not changed
  }

  mean_marker.pose.position.x /= marker_list_size;
  mean_marker.pose.position.y /= marker_list_size;
  mean_marker.pose.position.z /= marker_list_size;

  mean_marker.pose.orientation.x /= marker_list_size;
  mean_marker.pose.orientation.y /= marker_list_size;
  mean_marker.pose.orientation.z /= marker_list_size;
  mean_marker.pose.orientation.w /= marker_list_size;

  buffer_marker_ = visualization_msgs::Marker(mean_marker); // copy the marker to the buffer.
  markerMsgToCollisionObjectMsg(buffer_marker_, buffer_collision_object_); // convert the marker to a collision object to provide and buffer both options
  queue_changed_ = false;

  return true;
}

bool LimitedMarkerQueue::get_mean_collision_object(moveit_msgs::CollisionObject &mean_collision_object){
  //TODO: Find a better way to do this
  visualization_msgs::Marker dummy_marker;
  bool result = get_mean_marker(dummy_marker);
  mean_collision_object = moveit_msgs::CollisionObject(buffer_collision_object_);
  return result;
}

void LimitedMarkerQueue::reset() {
  marker_list_.clear();
}

void LimitedMarkerQueue::update_marker_list() {
  // limit length
  while (marker_list_.size() > max_length_) {
      marker_list_.pop_front();
      queue_changed_ = true;
  }

  // limit element age
  while ((ros::Time::now() - marker_list_.front().header.stamp) > element_lifetime_) {
      marker_list_.pop_front();
      queue_changed_ = true;
  }
}

bool LimitedMarkerQueue::markerMsgToCollisionObjectMsg(visualization_msgs::Marker marker, moveit_msgs::CollisionObject &collision_object) {
  moveit_msgs::CollisionObject result;

  // header
  result.header = marker.header;

  // id
  result.id = std::to_string(marker.id);

  // primitive
  shape_msgs::SolidPrimitive primitive;

  if (marker.type == visualization_msgs::Marker::CUBE) {
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = marker.scale.x;
    primitive.dimensions[1] = marker.scale.y;
    primitive.dimensions[2] = marker.scale.z;
  } else {
    ROS_ERROR("Marker type not supported");
    return false;
  }

  result.primitives.push_back(primitive);

  // primitive pose
  //tf::StampedTransform marker_transform;
  //tf::Transform object_transform;

  //tf::TransformListener listener;
  //try {
    //listener.waitForTransform(frame_id, marker.header.frame_id, ros::Time(0),
                              //ros::Duration(5.0));
    //listener.lookupTransform(frame_id, marker.header.frame_id, ros::Time(0),
                             //marker_transform);
  //} catch (tf::TransformException ex) {
    //ROS_ERROR("%s", ex.what());
    //return false;
  //}

  //tf::poseMsgToTF(marker.pose, object_transform);

  //geometry_msgs::Pose pose;
  //tf::poseTFToMsg(marker_transform * object_transform, pose);

  result.primitive_poses.push_back(marker.pose);

  // operation
  result.operation = moveit_msgs::CollisionObject::ADD;

  collision_object = result;

  return true;
}

