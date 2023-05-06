FROM arm64v8/ros:humble-ros-base AS base
SHELL [ "/bin/bash", "-c" ]

FROM base AS build
RUN apt-get update && apt-get install -y python3 python3-pip && apt autoremove && apt clean
ENV PYTHONWARNINGS ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
COPY . /app
WORKDIR /app/sphero-sdk/sphero-sdk-raspberry-python
RUN . ../sphero-sdk-first-time-setup.sh
WORKDIR /app
RUN cd ros2_ws && \
    python3 -m pip install -r ./src/rvr_node/requirements.txt && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && rosdep install --from-paths src -y --ignore-src && \
    colcon build
RUN rm /ros_entrypoint.sh && echo "source /app/scripts/container/ros_entrypoint.sh" >> ~/.bashrc

FROM base
COPY --from=build /app /app
EXPOSE 8080
ENTRYPOINT [ "/app/scripts/container/ros_entrypoint.sh" ]