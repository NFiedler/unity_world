#include <unity_world/limited_marker_queue.h>

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
  queue_changed_ = false;

  return true;
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
  }
}

