FROM ros:noetic

# install apt deps
RUN apt-get update && \
    apt-get install -y git libgflags-dev libpopt-dev \
                       libgoogle-glog-dev liblua5.1-0-dev \
                       libboost-all-dev libqt5websockets5-dev \
                       python-is-python3 libeigen3-dev sudo 

# multiplexers to run roscore and other infra in background
RUN apt-get install -y tmux

# install ros apt deps
RUN apt-get install -y ros-noetic-tf ros-noetic-angles

ARG HOST_UID
RUN useradd dev -m -s /bin/bash -u $HOST_UID -G sudo
USER dev
WORKDIR /home/dev
RUN rosdep update

# clone deps
RUN git clone https://github.com/ut-amrl/amrl_maps.git && \
    git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodules

# set up .bashrc
SHELL ["/bin/bash", "-l", "-c"]
RUN echo -e "source /opt/ros/noetic/setup.bash\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/cs378_starter\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.profile
RUN echo -e "source /opt/ros/noetic/setup.bash\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/cs378_starter\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc


# build deps
RUN source ~/.profile && cd amrl_msgs && make -j
RUN source ~/.profile && cd ut_automata && make 

# add launcher
ENV CS378_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
RUN chmod u+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash" ]
