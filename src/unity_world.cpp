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
  // TODO


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
  // TODO
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
  visualization_msgs::Marker marker;
  moveit_msgs::CollisionObject collision_object;
  for (auto &object_queue : object_smoothing_queues_) {
    if (!object_queue.second.get_mean_marker(marker)) {
      // impossible to create a mean marker
      // (too old)
      continue;
    }
    if (!markerMsgToCollisionObjectMsg(marker, collision_object)) { // conversion without transform nesessary!
      continue;
    }

    // set in conversion function
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

bool UnityWorld::markerMsgToCollisionObjectMsg(visualization_msgs::Marker marker, moveit_msgs::CollisionObject &collision_object) {
  moveit_msgs::CollisionObject result;

  // header
  result.header = marker.header;

  // id
  result.id = marker.id;

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


int main(int argc, char **argv) {

  ros::init(argc, argv, "UnityWorld");

  UnityWorld unity_world;

  ros::spin();
  return 0;
};
