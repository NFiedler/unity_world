
#ifndef LIMITED_MARKER_QUEUE
#define LIMITED_MARKER_QUEUE

#include <list.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class LimitedMarkerQueue {
  public:
    // option for later: LimitedMarkerQueue(int max_length);
    LimitedMarkerQueue(int max_length, ros::Duration element_lifetime);
    ~LimitedMarkerQueue() {};

    void push(visualization_msgs::Marker marker);
    bool get_mean_marker(visualization_msgs::Marker &mean_marker);
    void reset();

  private:

    int max_length_;
    ros::Duration element_lifetime_;
    std::list<visualization_msgs::Marker> marker_list_;

    void update_marker_list();


#endif

