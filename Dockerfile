FROM osrf/ros:iron-desktop

# Install Moveit2 https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html
RUN . /opt/ros/iron/setup.sh # instead of source

RUN apt update
RUN apt dist-upgrade -y
RUN rosdep update

RUN apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget && \
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

RUN apt remove ros-$ROS_DISTRO-moveit*

ENV COLCON_WS=/ws_moveit2
RUN mkdir -p $COLCON_WS/src
WORKDIR $COLCON_WS/src

RUN git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

RUN apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["bash"]