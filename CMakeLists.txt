cmake_minimum_required(VERSION 3.6)
project(JDRobot)

set(CMAKE_CXX_STANDARD 11)

set(HEADER_FILES
        Param.h
        CV/CVClass.h
        Communicate/Communication.h
        Communicate/SocketClass/SocketClass.h)

set(SOURCE_FILES
        main.cpp
        CV/CVCalib.cpp
        CV/CVClass.cpp
        Communicate/Communication.cpp
        Communicate/SocketClass/SocketClass.cpp
        ${HEADER_FILES} CV/AprilTagsClass/AprilTagsClass.cpp CV/AprilTagsClass/AprilTagsClass.h Communicate/SocketClass/SocketClass.cpp Communicate/SocketClass/SocketClass.h Threads/Threads.cpp Threads/Threads.h)

add_executable(JDRobot ${SOURCE_FILES})

include_directories(/home/jdrobot/apriltags/build/include
        /usr/include/eigen3)


link_directories("/usr/local/lib")
target_link_libraries(JDRobot libapriltags.a)
target_link_libraries(JDRobot opencv_calib3d)
target_link_libraries(JDRobot opencv_core)
target_link_libraries(JDRobot opencv_features2d)
target_link_libraries(JDRobot opencv_flann)
target_link_libraries(JDRobot opencv_highgui)
target_link_libraries(JDRobot opencv_imgcodecs)
target_link_libraries(JDRobot opencv_imgproc)
target_link_libraries(JDRobot opencv_ml)
target_link_libraries(JDRobot opencv_objdetect)
target_link_libraries(JDRobot opencv_videostab)
target_link_libraries(JDRobot opencv_video)
target_link_libraries(JDRobot opencv_videoio)
target_link_libraries(JDRobot opencv_superres)
target_link_libraries(JDRobot opencv_stitching)
target_link_libraries(JDRobot opencv_shape)
target_link_libraries(JDRobot opencv_photo)
target_link_libraries(JDRobot pthread)