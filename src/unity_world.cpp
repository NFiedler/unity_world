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


int main(int argc, char **argv) {

  ros::init(argc, argv, "UnityWorld");

  UnityWorld unity_world;

  ros::spin();
  return 0;
};
