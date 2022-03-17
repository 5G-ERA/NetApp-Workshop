FROM ros:galactic-ros-base

RUN cd /root/ \
    && mkdir dev_ws

COPY ./ /root/ML_repo

RUN mv /root/ML_repo/src /root/dev_ws/src \
    && mv /root/ML_repo/install_dependencies.sh /root/dev_ws 

RUN cd /root/dev_ws \
    && apt-get update \
    && rosdep update \
    && bash install_dependencies.sh 

RUN . /opt/ros/galactic/setup.sh \
    && cd /root/dev_ws \
    && colcon build

RUN cd \
    && mv ML_repo/docker/run.sh .

CMD ["/bin/bash", "/root/run.sh"]
