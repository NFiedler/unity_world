#include <unity_world/unity_world.h>


UnityWorld::UnityWorld() : nh_() {

  // setup planning scene interface
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  // setup subscribers
  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &UnityWorld::objectsCallback, this);


  // setup services
  setupPlanningSceneService_ = nh_.advertiseService("setupPlanningSceneService", &UnityWorld::setupPlanningSceneCallback, this);
  resetPlanningSceneService_ = nh_.advertiseService("resetPlanningSceneService", &UnityWorld::resetPlanningSceneCallback, this);

  // setup publishers

  collision_object_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 5);

  // setup timers

  publishing_timer_ = nh_.createTimer(ros::Duration(0.1), &UnityWorld::publishing_timer_callback, this);

}

void UnityWorld::objectsCallback(const visualization_msgs::MarkerArray &msg) {
  visualization_msgs::Marker marker;
  for (auto &msg_marker : msg.markers) {
    // copy the marker to have a mutable version
    marker = visualization_msgs::Marker(msg_marker);
    // transform the pose
    tf::StampedTransform marker_transform;
    tf::Transform object_transform;

    tf::TransformListener listener;
    try {
      listener.waitForTransform(object_frame_id, marker.header.frame_id, ros::Time(0),
                                ros::Duration(5.0));
      listener.lookupTransform(object_frame_id, marker.header.frame_id, ros::Time(0),
                               marker_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    tf::poseMsgToTF(marker.pose, object_transform);

    // geometry_msgs::Pose pose;
    tf::poseTFToMsg(marker_transform * object_transform, marker.pose);
    marker.header.frame_id = object_frame_id;

    // the []-operator accesses existing elements and creates them when they are not in the map already.
    object_smoothing_queues_[marker.id].push(marker);
  }
}

bool UnityWorld::setupPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  add_collision_objects();
  res.success = true;
  res.message = "added collision objects to planning scene";
  return true;
}

bool UnityWorld::resetPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  remove_collision_objects();
  res.success = true;
  res.message = "removed collision objects from planning scene";
  return true;
}

void UnityWorld::publishing_timer_callback(const ros::TimerEvent&) {
  // publish all mean collision objects which are updated recent enough
  moveit_msgs::CollisionObject collision_object;
  for (auto& object_queue : object_smoothing_queues_) {
    if (object_queue.second.get_mean_collision_object(collision_object)) {
      collision_object_publisher_.publish(collision_object);
    }
  }
}

void UnityWorld::remove_collision_objects() {
  moveit_msgs::CollisionObject current_collision_object;
  std::map<std::string, moveit_msgs::CollisionObject> collision_objects = psi_->getObjects();
  for (std::map<std::string, moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++ )
  {
    current_collision_object = it->second;
    current_collision_object.operation = current_collision_object.REMOVE;
    psi_->applyCollisionObject(current_collision_object);
  }
}

void UnityWorld::add_collision_objects() {
  moveit_msgs::CollisionObject collision_object;
  for (auto &object_queue : object_smoothing_queues_) {
    if (!object_queue.second.get_mean_collision_object(collision_object)) {
      // impossible to create a mean collision_object
      // (too old)
      continue;
    }

    // set in conversion function in limited_marker_queue
    // collision_object.operation = collision_object::ADD;

    psi_->applyCollisionObject(collision_object);
  }

}

void UnityWorld::update_from_planning_scene(bool use_queue) {
  // TODO
}

void UnityWorld::get_collision_objects(std::vector<std::string> collision_object_ids) {
  // TODO
}

void UnityWorld::get_collision_object(std::string collision_object_id) {
  // TODO
}

bool UnityWorld::collisionObjectMsgToMarkerMsg(moveit_msgs::CollisionObject collision_object, visualization_msgs::Marker &marker) {
  visualization_msgs::Marker result;

  // header
  result.header = collision_object.header;

  // id
  result.id = std::stoi(collision_object.id);

  shape_msgs::SolidPrimitive primitive = collision_object.primitives[0];

  if (primitive.type == primitive.BOX) {
    result.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = primitive.dimensions[0];
    marker.scale.y = primitive.dimensions[1];
    marker.scale.z = primitive.dimensions[2];
  } else {
    ROS_ERROR("CollisionObject type not supported");
    return false;
  }

  result.pose = collision_object.primitive_poses[0];

  marker = result;

  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "UnityWorld");

  UnityWorld unity_world;

  ros::spin();
  return 0;
};
