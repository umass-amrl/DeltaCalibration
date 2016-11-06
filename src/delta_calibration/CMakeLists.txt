set(CMAKE_AUTOMOC ON)

# SET(target check_transform)
# ROSBUILD_ADD_EXECUTABLE(${target} CheckTransform.cpp)
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

# SET(target obj_icp)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     obj_icp.cpp
#     ../perception_tools/kdtree.cpp)
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
# 
# SET(target test_delta_calibration)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     test_delta_cal.cpp
#     ../perception_tools/kdtree.cpp
#     icp.cpp)
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
#     
# SET(target keyframe_test)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     keyframe_test.cpp
#     ../perception_tools/kdtree.cpp
#     icp.cpp)
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
# 
SET(target delta_calibration)
ROSBUILD_ADD_EXECUTABLE(${target}
    delta_calibration.cpp
    icp.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    perception_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries 
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})
    
SET(target test_normals)
ROSBUILD_ADD_EXECUTABLE(${target}
    test_normals.cpp
    icp.cpp
    ../perception_tools/kdtree.cpp)
ADD_DEPENDENCIES(${target}
    ceres
    shared_libraries
    fspf_library
    perception_library
    ${OpenCV_LIBRARIES})
TARGET_LINK_LIBRARIES(${target}
    ceres
    shared_libraries 
    fspf_library
    ${OpenCV_LIBRARIES} ${libs})
#     
# SET(target slam_calibration)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     slam_calibration.cpp
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
# SET(target deltacal_post_process)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     post_processing.cpp
#     icp.cpp
#     ../perception_tools/kdtree.cpp)
# ADD_DEPENDENCIES(${target}
#     slider.h
#     ceres
#     shared_libraries
#     fspf_library
#     perception_library
#     ${OpenCV_LIBRARIES})
# TARGET_LINK_LIBRARIES(${target}
#     ceres
#     shared_libraries 
#     fspf_library
#     gui_libraries
#     ${OpenCV_LIBRARIES} ${libs})
# 
# SET(target obj_transform)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     obj_transform.cpp
#     icp.cpp
#     ../perception_tools/kdtree.cpp)
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

# SET(target check_transform_overlap)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     CheckTransform_overlap.cpp
#     ../perception_tools/kdtree.cpp)
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

# SET(target check_transform_color)
# ROSBUILD_ADD_EXECUTABLE(${target}
#     CheckTransform_color.cpp
#     ../perception_tools/kdtree.cpp)
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

# SET(target cal_live_transform)
# ROSBUILD_ADD_EXECUTABLE(${target} LiveTransform.cpp)
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
