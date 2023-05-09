FROM arm64v8/ros:humble-ros-base AS base
SHELL [ "/bin/bash", "-c" ]
RUN apt-get update && apt-get install -y python3 python3-pip && apt autoremove && apt clean
ENV PYTHONWARNINGS ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
COPY ./scripts /app/scripts
RUN rm /ros_entrypoint.sh && \
    echo "source /app/scripts/container/ros_entrypoint.sh" >> ~/.bashrc

FROM base AS build
COPY ./ros2_ws /app/ros2_ws
# Start remove blocking input() from firmware check file
WORKDIR /app/ros2_ws/src/rvr_node/sphero_sdk_wrapper/sphero_sdk_raspberry_python/sphero_sdk/common/firmware
RUN sed -i '/input()/d' cms_fw_check_base.py
# End remove blocking input()
WORKDIR /app/ros2_ws
RUN python3 -m pip install \
        -r ./src/rvr_node/requirements.txt \
        -r ./src/robot_control/requirements.txt && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && rosdep install --from-paths src -y --ignore-src && \
    colcon build && \
    rm -rf /app/ros2_ws/src && \
    rm -rf /app/ros2_ws/build
EXPOSE 8080
ENTRYPOINT [ "/app/scripts/container/ros_entrypoint.sh" ]