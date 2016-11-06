PROJECT(delta_calibration)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(arch ${})

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Wall -g -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -rdynamic")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

# End Build Notes and flags

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)
# End ROS things

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

# Executable path
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# QT stuff
SET(QT_USE_QTNETWORK true)
SET(QT_USE_QTOPENGL true)

#===============================================================================
# Targets selection
# OPTION(TESTER_TARGETS "Enable compilation of tester targets" FALSE)
# OPTION(RESEARCH_TARGETS "Enable compilation of research targets" FALSE)
# OPTION(UTIL_TARGETS "Enable compilation of research targets" FALSE)
OPTION(CALIBRATION_TARGETS "Enable complication of delta-calibration" FALSE)
#===============================================================================

#===============================================================================
# OpenKinect Library
# SET(CMAKE_MODULE_PATH
#     ${CMAKE_MODULE_PATH}
#     "${CMAKE_SOURCE_DIR}/src/kinect/Modules")
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/kinect/include)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/kinect/lib)
# ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/kinect/lib)
#===============================================================================

#===============================================================================
# Ceres-Solver Library, Xml2
# IF (RESEARCH_TARGETS)
# 
#   FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options)
#   INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# 
#   FIND_PACKAGE(LibXml2 REQUIRED)
#   INCLUDE_DIRECTORIES(${LIBXML2_INCLUDE_DIR})
# 
#   FIND_PACKAGE(OpenCV REQUIRED)
# 
# ENDIF (RESEARCH_TARGETS)
#===============================================================================

# FIND_PACKAGE(freenect2 REQUIRED)
# INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
FIND_PACKAGE(Qt4 REQUIRED)
# FIND_PACKAGE(OpenGL REQUIRED)
# FIND_PACKAGE(GLUT REQUIRED)
# FIND_PACKAGE(USB REQUIRED)
FIND_PACKAGE(Lua51 REQUIRED)
# FIND_PACKAGE(SDL REQUIRED)
# FIND_PACKAGE(SDL_ttf REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# SET(LIBQR_LIBRARIES /usr/local/lib/libdecodeqr.so)

INCLUDE(${QT_USE_FILE})

# FIND_PATH(GLEW_INCLUDE_PATH GL/glew.h
#     /usr/include
#     /usr/local/include
#     /sw/include
#     /opt/local/include
#     DOC "The directory where GL/glew.h resides")

# FIND_LIBRARY(GLEW_LIBRARY
#   NAMES GLEW glew
#   PATHS
#   /usr/lib64
#   /usr/lib
#   /usr/local/lib64
#   /usr/local/lib
#   /sw/lib
#   /opt/local/lib
#   DOC "The GLEW library")


INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/cobot)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/gui)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/gui/universal_view_log)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/hardware)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/human_tracker)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/kinect)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/libraries)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/map)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/navigation)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/net)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/opengl)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/perception_tools)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/delta_calibration)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared/math)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared/util)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/vector_localization)

# Subdirectories are commented because for one or more of the
# following reasons:
# 1. The directory is empty
# 2. The current source produces compilation errors and was omitted previously
# 3. The directory contains code not used in the current build

# add_subdirectory(${PROJECT_SOURCE_DIR}/src/cobot)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/cobot_sim)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/datasets)
IF(CALIBRATION_TARGETS)
  add_subdirectory(${PROJECT_SOURCE_DIR}/src/delta_calibration)
ENDIF(CALIBRATION_TARGETS)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/docking)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/gui)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/hardware)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/human_detector)
#kinect elsewhere
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/kinect_calibration)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/libraries)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/long-term)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/manager)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/map)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/map_switcher)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/navigation)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/net)
#opengl elsewhere
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/path_planner)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/planar_slam)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/polygon_triangulation)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/pose_estimation)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/remembrances)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/rgb_feature_detection)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/system)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/test)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/vector_localization)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/vector_mapping)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src/vector_slam)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/wifi)

# INCLUDE_DIRECTORIES(${GTK2_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(${USB_INCLUDE_DIR})

SET(libs roslib roscpp ncurses popt pthread  glut
     ${LUA_LIBRARIES} rt
    ${BUILD_SPECIFIC_LIBRARIES} glog rosbag X11)

# SET(HW_SRCS
#   src/hardware/serial.cc
# )

# SET(DRIVE1_SRCS
#   src/hardware/drive.cpp
#   src/hardware/fcs.cpp
# )

# SET(DRIVE2_SRCS
#   src/hardware/drive.cpp
#   src/hardware/fcs.cpp
# )

# SET(DRIVE3_SRCS
#   src/hardware/drive.cpp
#   src/hardware/serial.cc
#   src/hardware/fcs.cpp
#   src/hardware/companion/companion_drive/amcparse.cpp
#   src/hardware/companion/companion_drive/packet.cpp
#   src/hardware/companion/companion_drive/packet_helpers.cpp
#   src/hardware/companion/companion_drive/time_help.cpp
#   src/hardware/companion/companion_drive/companion_drive.cpp
# )

# SET(LASER_SRCS
#   src/hardware/laser_scanner.cpp
# )

SET(SHARED_SRCS
#   src/shared/math/gvector.h
#   src/shared/math/line.h
#   src/shared/math/gvector.cpp
#   src/shared/math/geomalgo.h
#   src/shared/math/geometry.h
#   src/shared/util/helpers.cpp
#   src/shared/util/proghelp.cc
  src/shared/util/pthread_utils.cpp
#   src/shared/util/terminal_utils.cpp
  src/shared/util/configreader.cpp
#   src/shared/util/watch_files.cpp
)

# SET(JOYSTICK_SRCS
#   src/hardware/joystick.cc
# )

# SET(MAP_SRCS
#   src/map/navigation_map.cpp
#   src/map/vector_map.cpp
#   src/map/stargazer_map.cpp
#   src/map/bitmap_map.cpp
# )

# SET(MAP_3D_SRCS
#   src/map/vector_map3d.cpp
#   src/opengl/glInfo.cpp
#   src/kinect/grahams_scan.cpp
#   src/kinect/plane_filtering.cpp
#   src/map/plane_polygon.cpp
# )

SET(FSPF_SRCS
  src/fspf/grahams_scan.cpp
  src/fspf/plane_filtering.cpp
  src/fspf/plane_polygon.cpp
)

# SET(PERCEPTION_SRCS
#   src/perception_tools/perception_2d.cpp
  #src/delta_calibration/icp.cpp
# )
# 
# SET(NAVIGATION_SRCS
#   src/navigation/navigation.cpp
#   ${MAP_SRCS}
# )

# SET(GUI_SRCS
# #   src/gui/vector_display.cpp
# #   src/gui/gui_publisher_helper.cpp
#   src/gui/gltext.cpp
# )

# SET(COBOT_CORE_SRCS
#   src/cobot/cobot.cpp
#   src/navigation/motion.cpp
#   ${LOCALIZATION_SRCS}
#   ${NAVIGATION_SRCS}
# )

# qt4_wrap_cpp(GUI_MOC_SRCS
#   src/delta_calibration/post_processing.cpp
# )

# qt4_wrap_cpp(HUMAN_TRACKER_MOCS
#     src/human_tracker/benchmark.h
# )
# 
# qt4_wrap_cpp(GLWIDGET_MOC_SRCS
#   src/opengl/glwidget.h
# )
# 
# qt4_add_resources(RC_SRCS
#   icons/gui_icons.qrc
#   ${SHARED_RC}
# )


#================================================================================
#Creating libraries to eliminate redundant compile steps

ADD_LIBRARY(shared_libraries SHARED ${SHARED_SRCS} ${HW_SRCS})
ADD_DEPENDENCIES(shared_libraries rosbuild_precompile)
TARGET_LINK_LIBRARIES(shared_libraries shared_libraries ${libs})

# ADD_LIBRARY(cobot_core_library ${COBOT_CORE_SRCS})
# ADD_DEPENDENCIES(cobot_core_library shared_libraries)
# TARGET_LINK_LIBRARIES(cobot_core_library shared_libraries re2)

# ADD_LIBRARY(map_libraries SHARED ${MAP_SRCS})
# ADD_DEPENDENCIES(map_libraries shared_libraries)
# TARGET_LINK_LIBRARIES(map_libraries shared_libraries ${libs} re2)

ADD_LIBRARY(fspf_library ${FSPF_SRCS})
ADD_DEPENDENCIES(fspf_library shared_libraries)
TARGET_LINK_LIBRARIES(fspf_library shared_libraries ${libs})

# ADD_LIBRARY(perception_library ${PERCEPTION_SRCS})
# ADD_DEPENDENCIES(perception_library shared_libraries)
# TARGET_LINK_LIBRARIES(perception_library shared_libraries ${libs})

# ADD_LIBRARY(dijkstra_libraries STATIC
#     src/libraries/dijkstra/dgraph.cpp
#     src/libraries/dijkstra/dijkstra.cpp
#     src/libraries/dijkstra/radixheap.cpp)

# ROSBUILD_ADD_LIBRARY(vector_display
#     src/gui/py_vector_display.cpp
#     src/gui/vector_display_thread.cpp)
# TARGET_LINK_LIBRARIES(vector_display gui_libraries)

# ADD_LIBRARY(gui_libraries SHARED ${GUI_SRCS} ${GUI_MOC_SRCS})
# ADD_DEPENDENCIES(gui_libraries shared_libraries map_libraries ${GUI_MOC_SRCS})
# TARGET_LINK_LIBRARIES(gui_libraries
#     shared_libraries
# #     map_libraries
#     ${libs}
#     ${QT_LIBRARIES})
# TARGET_LINK_LIBRARIES(gui_libraries ${QT_LIBRARIES})

#===============================================================================
# The Main Executable Targets
#

# add_subdirectory(${PROJECT_SOURCE_DIR}/src/kinect)

#===============================================================================
# Highly Experimental Research Code
# IF (RESEARCH_TARGETS)
# 
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/opengl)
# 
# ENDIF (RESEARCH_TARGETS)

#===============================================================================
# Utilities

# IF (UTIL_TARGETS)
# 
# ENDIF (UTIL_TARGETS)

#===============================================================================
# Tester executables

# IF (TESTER_TARGETS)
# 
# 
# ENDIF (TESTER_TARGETS)

#===============================================================================

# Change file attribute for the wifi scanner
# ADD_CUSTOM_COMMAND(TARGET cobot_wifi
#       POST_BUILD
#       COMMAND sudo chown root:root ${EXECUTABLE_OUTPUT_PATH}/cobot_wifi
#       VERBATIM
# )
# ADD_CUSTOM_COMMAND(TARGET cobot_wifi
#       POST_BUILD
#       COMMAND sudo chmod 4755 ${EXECUTABLE_OUTPUT_PATH}/cobot_wifi
#       VERBATIM
# )
