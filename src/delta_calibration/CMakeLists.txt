set(CMAKE_AUTOMOC ON)

# SET(target delta_calibration)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     delta_calibration.cpp
#     icp.cpp
#     ../perception_tools/kdtree.cpp)
# ADD_DEPENDENCIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     perception_library
#     ${OpenCV_LIBRARIES})
# TARGET_LINK_LIBRARIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     ${OpenCV_LIBRARIES} ${libs})


SET(target delta_calculate)
ROSBUILD_ADD_EXECUTABLE(${target}
    delta_calibration.cpp
    delta_calculate.cpp
    icp.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})

SET(target record_turtlebot_data)
ROSBUILD_ADD_EXECUTABLE(${target}
icp.cpp
delta_calibration.cpp
partial_calibrate.cpp
record_turtlebot_data.cpp
../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
ceres
shared_libraries
fspf_library
${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
ceres
shared_libraries
fspf_library
${OpenCV_LIBRARIES} ${libs})

SET(target partial_calibrate)
ROSBUILD_ADD_EXECUTABLE(${target}
    test_calibrate.cpp
    icp.cpp
    partial_calibrate.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})

SET(target test_turtlebot_calibrate)
ROSBUILD_ADD_EXECUTABLE(${target}
    test_turtlebot_calibrate.cpp
    icp.cpp
    partial_calibrate.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})

SET(target delta_calibrate)
ROSBUILD_ADD_EXECUTABLE(${target}
    delta_calibrate.cpp
    icp.cpp
    partial_calibrate.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})
# SET(target test_normals)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     test_normals.cpp
#     icp.cpp
#     ../perception_tools/kdtree.cpp)
# ADD_DEPENDENCIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     perception_library
#     ${OpenCV_LIBRARIES})
# TARGET_LINK_LIBRARIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     ${OpenCV_LIBRARIES} ${libs})
#
SET(target visualizes_poses)
ROSBUILD_ADD_EXECUTABLE(${target}
    visualize_poses.cpp
    icp.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})
#
 SET(target find_z)
ROSBUILD_ADD_EXECUTABLE(${target}
    find_z.cpp
    icp.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})
#
# SET(target check_transform)
# ROSBUILD_ADD_EXECUTABLE(${target} check_transform.cpp
#                                   icp.cpp
#                                   ../perception_tools/kdtree.cpp)
# ADD_DEPENDENCIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     ${OpenCV_LIBRARIES})
# TARGET_LINK_LIBRARIES(${target}
#     ceres
#     shared_libraries
#     fspf_library
#     ${OpenCV_LIBRARIES} ${libs})
