# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM osrf/ros:foxy-desktop

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip
RUN apt-get install -y ros-foxy-cv-bridge ros-foxy-pcl-conversions ros-foxy-xacro ros-foxy-robot-state-publisher \
    ros-foxy-image-transport ros-foxy-image-transport-plugins ros-foxy-pcl-ros libpcap-dev

WORKDIR /root/workspace

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]