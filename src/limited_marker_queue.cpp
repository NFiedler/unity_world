#include <unity_world/limited_marker_queue.h>

LimitedMarkerQueue::LimitedMarkerQueue(int max_length, ros::Duration element_lifetime) : max_length_(max_length), element_lifetime_(element_lifetime), marker_list_() {}


void LimitedMarkerQueue::push(visualization_msgs::Marker marker) {
  marker_list_.push_back(marker);

  // limit length
  if (marker_list_.size() > max_length_) {
      marker_list_.pop_front();
  }

  // limit element age
  while (ros::Time::now() - marker_list_.front().header.stamp > element_lifetime_) {
      marker_list_.pop_front();
  }
}

void LimitedMarkerQueue::get_mean_marker(visualization_msgs::Marker &mean_marker){

}

