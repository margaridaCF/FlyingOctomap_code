FROM px4io/px4-dev-ros:2017-12-08

MAINTAINER Magarida Faria

RUN apt-get update \
	&& sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
	&& sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
	&& wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
	&& apt-get update \
	&& apt-get install -y python-catkin-tools gdb ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-pcl-conversions ros-kinetic-pcl-ros geographiclib-tools rosbash ros-kinetic-rviz ros-kinetic-velodyne-simulator ros-kinetic-rqt-robot-plugins python-rospkg libqt4-dev ros-kinetic-geometry2 ros-kinetic-robot-state-publisher ros-kinetic-joint-state-publisher ros-kinetic-message-to-tf ros-kinetic-depthimage-to-laserscan\
	&& mkdir /Firmware \
	&& cd /Firmware \
	&& wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
	&& chmod +x install_geographiclib_datasets.sh \
	&& sudo  ./install_geographiclib_datasets.sh 
RUN mkdir /ros_ws \
	&& mkdir /ros_ws/src \
	&& cd /ros_ws/src \
	&& git clone https://github.com/grvcTeam/grvc-utils.git \
	&& git clone https://github.com/grvcTeam/grvc-ual.git \
	&& cd grvc-ual \
	&& git checkout 464e04e99bf1a465b6e27ecbd4ffc731ecf6c504 \
	&& cd ../grvc-utils \
	&& git checkout d2332c4ebea43e143c621af44b21059fa0082e6a \
	&& cd /ros_ws/src/grvc-ual \
	&& sed -i 's/mbzirc/aeroarms/g' px4_bringup/launch/spawn_robot.launch
