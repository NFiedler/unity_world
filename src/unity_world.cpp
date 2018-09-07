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
