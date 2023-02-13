cmake_minimum_required(VERSION 3.0.2)
project(roblab)


#############################新しいmsgの作成ここから#################################
#msg作成に必要なパッケージを探す　message_generationは必須
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)
catkin_python_setup()

## msgフォルダ内のカスタムメッセージファイルを追記
 add_message_files(
   FILES
   JoyJoy.msg
   LRF_info.msg
 )

## msg作成に必要な依存関係を記述
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

##他のパッケージがこのパッケージを依存したときに依存に加わる必要があるパッケージを追加
#catkin_package(
#  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#)

#############################新しいmsgの作成ここまで#################################

#############################ライブラリの作成ここから###########################################

#必要なパッケージを探す
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  sensor_msgs
#)


#ヘッダーファイルの場所を指定ここではincludeを表す。
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##includeが他のパッケージから見えるようにし、パスの自動解決と依存関係の自動追加を設定する。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roblab
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs
)

## ライブラリを追加
 add_library(roblab
   src/lib/differential.cpp
   src/lib/dob.cpp
   src/lib/integral.cpp
   src/lib/lpf.cpp
   src/lib/pid.cpp
   src/lib/trapezoid_control.cpp
   src/lib/timer.cpp
   src/lib/ros_clock_timer.cpp
   src/lib/cycle_count_timer.cpp
 )
 target_link_libraries(roblab ${catkin_LIBRARIES})

##################################ライブラリの作成ここまで#######################################################


########################joyjoyノードbuild関係################################
add_executable(joyjoy src/joyjoy.cpp)
add_dependencies(joyjoy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joyjoy ${catkin_LIBRARIES})
##################################################################################

########################additionalノードbuild関係################################
add_executable(additional src/additional.cpp)
add_dependencies(additional ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(additional ${catkin_LIBRARIES})
##################################################################################

###########################テスト用###########################
add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
#################################################################################


###########################カスタムmsg使用に必要（依存する側に記述）####################################
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  sensor_msgs
#  roblab
#)
#add_dependencies(joyjoy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#####################################################################################

catkin_install_python(
  PROGRAMS script/webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})