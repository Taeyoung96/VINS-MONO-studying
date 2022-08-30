# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM ros:melodic

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y cmake libeigen3-dev libpcl-dev \
    ros-melodic-rviz ros-melodic-pcl-ros ros-melodic-eigen-conversions \
    ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-message-filters ros-melodic-tf \ 
    libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget

# Install ceres-solver
WORKDIR /home/thirdParty
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
WORKDIR /home/thirdParty/ceres-solver-1.14.0

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ../../ceres-solver-1.14.0 && make -j2 && make install


WORKDIR /home/catkin_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]