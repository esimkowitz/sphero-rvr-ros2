FROM arm64v8/ros:humble-ros-base AS base
SHELL [ "/bin/bash", "-c" ]
RUN apt-get update && apt-get install -y python3 python3-pip && apt autoremove && apt clean
ENV PYTHONWARNINGS ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
COPY ./scripts /app/scripts
RUN rm /ros_entrypoint.sh && \
    echo "source /app/scripts/container/ros_entrypoint.sh" >> ~/.bashrc

FROM base AS build
COPY ./ros2_ws /app/ros2_ws
WORKDIR /app/ros2_ws/src/rvr_node/sphero_sdk_raspberry_python/sp
RUN . ../sphero-sdk-first-time-setup.sh
WORKDIR /app/ros2_ws
RUN python3 -m pip install -r ./src/rvr_node/requirements.txt && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && rosdep install --from-paths src -y --ignore-src && \
    colcon build

FROM build
EXPOSE 8080
ENTRYPOINT [ "/app/scripts/container/ros_entrypoint.sh" ]