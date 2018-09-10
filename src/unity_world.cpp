#include <unity_world/unity_world.h>


UnityWorld::UnityWorld() : nh_() {

  // setup planning scene interface
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  // setup subscribers
  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &MoveitConnector::objectsCallback, this);


  // setup services
  setupPlanningSceneService_ = nh.advertiseService("setupPlanningSceneService", setupPlanningSceneCallback);
  resetPlanningSceneService_ = nh.advertiseService("resetPlanningSceneService", resetPlanningSceneCallback);

}
void UnityWorld::setupPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  add_collision_objects();
  res.success = true;
  res.message = "added collision objects to planning scene";
  return true;
}

void UnityWorld::resetPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  remove_collision_objects();
  res.success = true;
  res.message = "removed collision objects from planning scene";
  return true;
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

int main(int argc, char **argv) {

  ros::init(argc, argv, "UnityWorld");

  UnityWorld unity_world;

  ros::spin();
  return 0;
};
