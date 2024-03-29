cmake_minimum_required(VERSION 2.8.3)
project(gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  ApplyJointEfforts.msg
  ContactsState.msg
  ContactState.msg
  LinkState.msg
  LinkStates.msg
  JointStates.msg
  ModelState.msg
  ModelStates.msg
  ODEJointProperties.msg
  ODEPhysics.msg
  WorldState.msg
  )

add_service_files(DIRECTORY srv FILES
  AddJoint.srv
  AddLink.srv
  ApplyBodyWrench.srv
  DeleteModel.srv
  GetLinkState.srv
  GetPhysicsProperties.srv
  SetJointProperties.srv
  SetModelConfiguration.srv
  SpawnModel.srv
  ApplyJointEffort.srv
  GetJointProperties.srv
  GetModelProperties.srv
  GetWorldProperties.srv
  StartTimer.srv
  SetLinkProperties.srv
  SetModelState.srv
  BodyRequest.srv
  GetLinkProperties.srv
  GetModelState.srv
  JointRequest.srv
  SetLinkState.srv
  SetPhysicsProperties.srv
  SetJointTrajectory.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
  )
