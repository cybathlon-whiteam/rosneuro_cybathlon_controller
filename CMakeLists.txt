cmake_minimum_required(VERSION 3.0)
project(rosneuro_cybathlon_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 geometry_msgs
			 rosneuro_msgs
			 dynamic_reconfigure
			 )

generate_dynamic_reconfigure_options(
	cfg/AllController.cfg
	cfg/NavigationController.cfg
  cfg/Bars.cfg
  cfg/Range.cfg
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	geometry_msgs
	rosneuro_msgs
  DEPENDS
)


###########
## Build ##
###########

include_directories(include 
					${catkin_INCLUDE_DIRS} 
					)

add_library(${PROJECT_NAME}	
				src/NavigationController.cpp
				src/AllController.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
					  )

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
				 ${PROJECT_NAME}_gencfg
)

#################
## Executables ##
#################
add_executable(navigation_controller src/navigation_controller.cpp)
target_link_libraries(navigation_controller ${PROJECT_NAME}
								 ${catkin_LIBRARIES})
