cmake_minimum_required(VERSION 3.0.2)
project(personality_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)


add_message_files(DIRECTORY msg FILES
  Stats.msg
  PersonalityScene.msg
  PersonalityScenePoint.msg
  PersonalityTimeWindow.msg
)

generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)
