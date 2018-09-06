#include <unity_world/limited_marker_queue.h>

LimitedMarkerQueue::LimitedMarkerQueue(int max_length, ros::Duration element_lifetime) : max_length_(max_length), element_lifetime_(element_lifetime), marker_list_() {}


void LimitedMarkerQueue::push(visualization_msgs::Marker marker) {
  // add element and possibly invalidate the list
  marker_list_.push_back(marker);
}

bool LimitedMarkerQueue::get_mean_marker(visualization_msgs::Marker &mean_marker){
  // update the marker list before accessing it
  update_marker_list();
  int marker_list_size = marker_list_.size();
  if (marker_list_size < 1) {
      return false;
  }


  // reset the marker by setting it to the most recent element in the queue.
  mean_marker = marker_list_.back();

  // iterate through list
  for (visualization_msgs::Marker::iterator marker_iterator = marker_list_.begin(); marker_iterator < marker_list_.end() - 1; marker_iterator++) {
    // position
    mean_marker.pose.position.x += (*marker_iterator)->pose.position.x;
    mean_marker.pose.position.y += (*marker_iterator)->pose.position.y;
    mean_marker.pose.position.z += (*marker_iterator)->pose.position.z;

    // orientation
    mean_marker.pose.orientation.x += (*marker_iterator)->pose.orientation.x;
    mean_marker.pose.orientation.y += (*marker_iterator)->pose.orientation.y;
    mean_marker.pose.orientation.z += (*marker_iterator)->pose.orientation.z;
    mean_marker.pose.orientation.w += (*marker_iterator)->pose.orientation.w;

    // the shape etc. is not changed
  }

  mean_marker.pose.position.x /= marker_list_size;
  mean_marker.pose.position.y /= marker_list_size;
  mean_marker.pose.position.z /= marker_list_size;

  mean_marker.pose.orientation.x /= marker_list_size;
  mean_marker.pose.orientation.y /= marker_list_size;
  mean_marker.pose.orientation.z /= marker_list_size;
  mean_marker.pose.orientation.w /= marker_list_size;

  return true;
}

void LimitedMarkerQueue::reset() {
  marker_list_.clear();
}

void LimitedMarkerQueue::update_marker_list() {
  // limit length
  while (marker_list_.size() > max_length_) {
      marker_list_.pop_front();
  }

  // limit element age
  while ((ros::Time::now() - marker_list_.front().header.stamp) > element_lifetime_) {
      marker_list_.pop_front();
  }
}

