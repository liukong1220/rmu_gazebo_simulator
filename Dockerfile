FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim lsb-release gnupg -y && \
    sudo pip install vcstool2 xmacro

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh
CMD [ "/bin/zsh" ]

# Add Gazebo package repository and install Ignition Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress

# create workspace
#
# The simulator no longer carries its own dependency manifest: every rmoss
# source dependency and the robot description are owned by the ATS workspace
# root manifest (<workspace>/dependencies.repos), which imports them into
# src/dependencies and src/ats_robot_description. Importing the nested manifest
# here would resolve a second copy of rmoss_interfaces/sdformat_tools.
RUN mkdir -p ~/ros_ws && \
    cd ~/ros_ws && \
    git clone https://github.com/liukong1220/ATS_2026_snetry_test.git ats_ws && \
    vcs import --recursive ats_ws < ats_ws/dependencies.repos && \
    cp -r ats_ws/src ./src

WORKDIR /root/ros_ws

# install dependencies and some tools
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
source ~/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

RUN rm -rf /var/lib/apt/lists/*
