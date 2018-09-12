#include <unity_world/unity_world.h>


UnityWorld::UnityWorld() : nh_() {

  // setup planning scene interface
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  publish_from_psi_ = false;

  // setup subscribers
  object_sub_ = nh_.subscribe(
      "apriltag_objects", 1, &UnityWorld::objectsCallback, this);


  // setup services
  setupPlanningSceneService_ = nh_.advertiseService("UnityWorld/setupPlanningScene", &UnityWorld::setupPlanningSceneCallback, this);
  resetPlanningSceneService_ = nh_.advertiseService("UnityWorld/resetPlanningScene", &UnityWorld::resetPlanningSceneCallback, this);

  // setup publishers

  collision_object_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>("UnityWorld/collision_object", 5);
  marker_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>("UnityWorld/collision_object_marker", 5);

  // setup timers

  publishing_timer_ = nh_.createTimer(ros::Duration(0.1), &UnityWorld::publishing_timer_callback, this);

  ROS_INFO("Initialized UnityWorld");

}

void UnityWorld::objectsCallback(const visualization_msgs::MarkerArray &msg) {

  visualization_msgs::Marker marker;
  for (auto &msg_marker : msg.markers) {
    // copy the marker to have a mutable version
    marker = visualization_msgs::Marker(msg_marker);
    // transform the pose
    tf::StampedTransform marker_transform;
    tf::Transform object_transform;

    try {
      ros::Time now = ros::Time(0);
      collision_object_transform_listener_.waitForTransform(object_frame_id_, marker.header.frame_id, now,
                                ros::Duration(5));
      collision_object_transform_listener_.lookupTransform(object_frame_id_, marker.header.frame_id, now,
                               marker_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    tf::poseMsgToTF(marker.pose, object_transform);

    // geometry_msgs::Pose pose;
    tf::poseTFToMsg(marker_transform * object_transform, marker.pose);
    marker.header.frame_id = object_frame_id_;

    // the []-operator accesses existing elements and creates them when they are not in the map already.
    object_smoothing_queues_[marker.id].push(visualization_msgs::Marker(marker));
  }
}

bool UnityWorld::setupPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  add_collision_objects();
  res.success = true;
  res.message = "added collision objects to planning scene";
  ROS_INFO("Added collision objects to planning scene");
  return true;
}

bool UnityWorld::resetPlanningSceneCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  remove_collision_objects();
  res.success = true;
  res.message = "removed collision objects from planning scene";
  ROS_INFO("Removed collision objects from planning scene");
  return true;
}

void UnityWorld::publishing_timer_callback(const ros::TimerEvent&) {

  tf::StampedTransform object_transform;
  tf::Quaternion tf_quaternion;
  if (publish_from_psi_) {
    // publishing objects in the psi, not the ones from the queue

    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_collision_objects_map = psi_->getAttachedObjects();
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = psi_->getObjects();
    geometry_msgs::Quaternion quaternion_msg;
    tf::Vector3 tf_vector3;
    moveit_msgs::CollisionObject current_collision_object;

    // 'casting' AttachedCollisionObjects to CollisionObjects
    for ( std::pair<std::string, moveit_msgs::AttachedCollisionObject> attached_collision_object_pair : attached_collision_objects_map ) {
      current_collision_object = attached_collision_object_pair.second.object;
      current_collision_object.header.stamp = ros::Time::now();

      // Broadcasting object transform

      object_transform.setOrigin(tf::Vector3(current_collision_object.primitive_poses[0].position.x, current_collision_object.primitive_poses[0].position.y, current_collision_object.primitive_poses[0].position.z));
      tf::quaternionMsgToTF(current_collision_object.primitive_poses[0].orientation, tf_quaternion);
      object_transform.setRotation(tf_quaternion);
      collision_object_transform_broadcaster_.sendTransform(tf::StampedTransform(object_transform, ros::Time::now(), current_collision_object.header.frame_id, current_collision_object.id));

      // remove attached objects from objects to ignore attached objects which are still part of the collision objects
      if (collision_objects_map.find(attached_collision_object_pair.first) != collision_objects_map.end()) {
        collision_objects_map.erase(attached_collision_object_pair.first);
        // ROS_WARN_STREAM("The same object (" << ait->first << ") in attached and not attached collision objects!");
      }



      // transform object pose to odom_combined
      try {
        collision_object_transform_listener_.waitForTransform(object_frame_id_, std::string("/").append(static_cast<std::string>(current_collision_object.id)), ros::Time(0),
                                ros::Duration(.5));
        collision_object_transform_listener_.lookupTransform(object_frame_id_, std::string("/").append(static_cast<std::string>(current_collision_object.id)),
                               ros::Time(0), object_transform);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        continue;
      }
      tf::quaternionTFToMsg(object_transform.getRotation(), quaternion_msg);
      tf_vector3 = object_transform.getOrigin();
      current_collision_object.header.frame_id = object_frame_id_;
      current_collision_object.primitive_poses[0].position.x = tf_vector3.x();
      current_collision_object.primitive_poses[0].position.y = tf_vector3.y();
      current_collision_object.primitive_poses[0].position.z = tf_vector3.z();
      current_collision_object.primitive_poses[0].orientation = quaternion_msg;

      // current_unity_object.header = current_collision_object.header;
      // current_unity_object.object_name = current_collision_object.id;
      // current_unity_object.pose = current_collision_object.primitive_poses[0];
      collision_object_publisher_.publish(current_collision_object);
      // unity_object_pub.publish(current_unity_object);
    }

    for ( std::pair<std::string, moveit_msgs::CollisionObject> collision_object_pair : collision_objects_map) {
      current_collision_object = collision_object_pair.second;
      current_collision_object.header.stamp = ros::Time::now();

      // Broadcasting object transform

      object_transform.setOrigin(tf::Vector3(current_collision_object.primitive_poses[0].position.x, current_collision_object.primitive_poses[0].position.y, current_collision_object.primitive_poses[0].position.z));
      tf::quaternionMsgToTF(current_collision_object.primitive_poses[0].orientation, tf_quaternion);
      object_transform.setRotation(tf_quaternion);
      collision_object_transform_broadcaster_.sendTransform(tf::StampedTransform(object_transform, ros::Time::now(), current_collision_object.header.frame_id, current_collision_object.id));
      // current_unity_object.header = current_collision_object.header;

      // no transform necessary because odom_combined is already the frame

      // current_unity_object.object_name = current_collision_object.id;
      // current_unity_object.pose = current_collision_object.primitive_poses[0];
      collision_object_publisher_.publish(current_collision_object);
      // unity_object_pub.publish(current_unity_object);
    }
    return;
  }

  // publish all mean collision objects which are updated recent enough
  moveit_msgs::CollisionObject collision_object;
  visualization_msgs::Marker marker;
  for (auto& object_queue : object_smoothing_queues_) {
    if (object_queue.second.get_mean_collision_object(collision_object)) {
      collision_object_publisher_.publish(collision_object);
      if(publish_collision_object_marker_ && object_queue.second.get_mean_marker(marker)) {
        marker.lifetime = 0.2;
        marker.color.a = 1;
        marker.color.b = 1;
        marker_publisher_.publish(marker);
      }

      // Broadcasting object transform

      object_transform.setOrigin(tf::Vector3(collision_object.primitive_poses[0].position.x, collision_object.primitive_poses[0].position.y, collision_object.primitive_poses[0].position.z));
      tf::quaternionMsgToTF(collision_object.primitive_poses[0].orientation, tf_quaternion);
      object_transform.setRotation(tf_quaternion);
      collision_object_transform_broadcaster_.sendTransform(tf::StampedTransform(object_transform, ros::Time::now(), collision_object.header.frame_id, collision_object.id));
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

  // after removing the objects from the planning scene, the object queues get reset to the last pose in the planning scene
  get_update_from_planning_scene();
  publish_from_psi_ = false;

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

  // now, the content of the planning scene gets published as collision objects
  publish_from_psi_ = true;

}

void UnityWorld::get_update_from_planning_scene() {

  for (auto &object_queue : object_smoothing_queues_) {
    // reset all queues
    object_queue.second.reset();
  }

  visualization_msgs::Marker marker;

  // adding all CollisionObjects to the queue. No object should be attached at this point.
  for (auto &collision_object : psi_->getObjects()) {
    collisionObjectMsgToMarkerMsg(collision_object.second, marker); // getObjects() returns a map, therefore the element is a pair.
    object_smoothing_queues_[marker.id].push(visualization_msgs::Marker(marker));
  }

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
