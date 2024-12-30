cmake_minimum_required(VERSION 3.10)
project(collision-avoidance-metric)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# message("CONDA_PREFIX = $ENV{CONDA_PREFIX}")
# list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
# list(APPEND CMAKE_PREFIX_PATH "/home/sujay/miniconda3/envs/collision-metric")

# Add OpenCV include directory
include_directories(
    /usr/include/eigen3/
    /usr/include/opencv4
    /usr/include/xtensor
    /home/sujay/Code/collision-avoidance-metric/include
    # /home/sujay/miniconda3/envs/collision-metric/include
)

# Specify the OpenCV static libraries
set(OpenCV_LIBS
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
    /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
    /usr/local/lib/libOpen3D.so
)

# Add additional dependencies
set(Extra_LIBS
    -lpng 
    -ljpeg 
    -lz 
    -ltiff 
    -lwebp 
    -pthread
)

# Specify your source files
add_executable(collision-avoidance-metric test/test.cpp src/CollisionAvoidanceMetric.cpp src/GripperPaths.cpp src/PointCloud.cpp src/Utils.cpp)

# Link the libraries
target_link_libraries(collision-avoidance-metric ${OpenCV_LIBS} ${Extra_LIBS})
