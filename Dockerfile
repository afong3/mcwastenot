FROM ubuntu:22.04

RUN apt-get update
RUN apt-get install vim -y

# install ROS2 Iron

RUN apt update
RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe # might be weird here

RUN apt update
RUN apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update 
RUN apt-get update
RUN apt install python3-flake8-docstrings -y
RUN apt install python3-pip -y
RUN apt install python3-pytest-cov -y
RUN apt install python3-flake8-blind-except -y
RUN apt install python3-flake8-builtins -y
RUN apt install python3-flake8-class-newline -y
RUN apt install python3-flake8-comprehensions -y
RUN apt install python3-flake8-deprecated -y
RUN apt install python3-flake8-import-order -y
RUN apt install python3-flake8-quotes -y
RUN apt install python3-pytest-repeat -y
RUN apt install python3-pytest-rerunfailures -y --fix-missing

ENV DEBIAN_FRONTEND noninteractive

RUN 2 | apt install ros-dev-tools -y

ENV DEBIAN_FRONTEND interactive


RUN mkdir -p /ros2_iron/src
WORKDIR /ros2_iron
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos /ros2_iron/src

RUN apt upgrade -y

RUN rosdep init
RUN rosdep update
#RUN apt-get update # might have to run this after the rosdep install, then recall the rosdep install command
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" 

RUN colcon build --symlink-install

RUN ls /ros2_iron/src
RUN ls /ros2_iron/install/

RUN . /ros2_iron/install/local_setup.bash

CMD ["bash"]