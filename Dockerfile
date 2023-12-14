#---
# name: kinect
# alias: igen
# depends: []
# config: config.py
# test: test.py
# docs: README.md
#---
FROM dustynv/ros:noetic-desktop-l4t-r32.7.1

SHELL ["/bin/bash", "-c"]
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ARG MAKEFLAGS=-j$(nproc)
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV PACKAGE_SOURCE=/workspace/ros_catkin_ws/src/
ENV ROS_WS=/workspace/ros_catkin_ws

WORKDIR /workspace/ros_catkin_ws/src

# installing libfreenect2

#RUN git clone https://github.com/OpenKinect/libfreenect2.git
RUN git clone https://github.com/geoffviola/libfreenect2
RUN sudo apt-get update && \
	sudo apt-get install -y \
	build-essential \
	cmake \
	pkg-config \
	libusb-1.0-0-dev \
	libturbojpeg0-dev \
	libglfw3-dev \
	&& rm -rf /var/lib/apt/lists/* \
    && apt-get clean

WORKDIR /workspace/ros_catkin_ws/src/libfreenect2

RUN mkdir build 
WORKDIR /workspace/ros_catkin_ws/src/libfreenect2/build
RUN cmake .. \
	-DCUDA_PROPAGATE_HOST_FLAGS=off \
	-DCMAKE_INSTALL_PREFIX=/workspace/ros_catkin_ws/src/freenect2 \
	-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2/
RUN make
RUN make install
#RUN cmake -Dfreenect2_DIR=/root/freenect2/lib/cmake/freenect2

RUN sudo mkdir -p /etc/udev/rules.d/
RUN sudo cp /workspace/ros_catkin_ws/src/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# ROS PACKAGE DEPENDENCIES THAT WE NEED TO BUILD FROM SOURCE
# Reference: http://wiki.ros.org/noetic/Installation/Source
# tf, compressed_depth_image_transport (strangely needs to be the indigo-devel branch), kinect2_registration, cv_bridge?,
# nodelet, compressed_image_transport, libpcl-all-dev, bondcpp, angles on the master branch, tf2 

# Note that some poeple just changed the python37 requirement on the cv_bridge cmakelists to make it work. maybeI should do that instead 
# of setting all of the python directories 

# Install geometry package (depends on tf)
WORKDIR /workspace/ros_catkin_ws/src
RUN git clone https://github.com/ros/geometry.git 
RUN git clone https://github.com/ros-perception/image_transport_plugins.git
WORKDIR /workspace/ros_catkin_ws/src/image_transport_plugins
RUN git checkout -b hydro-devel
WORKDIR /workspace/ros_catkin_ws/src
RUN git clone https://github.com/ros/nodelet_core.git
RUN git clone https://github.com/ros/bond_core.git 
RUN sudo apt-get update && \
	sudo apt-get install -y \
	libpcl-dev \
	vim && \
	rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN rosdep update

RUN git clone https://github.com/afong3/vision_opencv.git && \
	cd vision_opencv && \
	git checkout noetic 

## Cloning a fork of the original iai_kinectv2 because Opencv4 compatability
# RUN rosdep install -r --from-paths .
# WORKDIR /workspace/ros_catkin_ws/src
RUN git clone https://github.com/ros/angles.git && \
	cd /workspace/ros_catkin_ws/src/angles && \
	git checkout master

WORKDIR /workspace/ros_catkin_ws/src
RUN rm -r common_msgs 
RUN git clone https://github.com/ros/geometry2.git
RUN git clone https://github.com/ros/common_msgs.git

RUN git clone https://github.com/ros/actionlib.git
RUN git clone https://github.com/ros/dynamic_reconfigure.git
RUN git clone https://github.com/ros-perception/perception_pcl.git
WORKDIR /workspace/ros_catkin_ws/src/perception_pcl
RUN git checkout melodic-devel
WORKDIR /workspace/ros_catkin_ws/src
RUN git clone https://github.com/ros-perception/pcl_msgs.git
WORKDIR /workspace/ros_catkin_ws
RUN source /opt/ros/noetic/setup.bash && \
	./src/catkin/bin/catkin_make_isolated --install \
	-DCMAKE_BUILD_TYPE=Release  --only-pkg-with-deps \
	-j2 pcl_ros

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
