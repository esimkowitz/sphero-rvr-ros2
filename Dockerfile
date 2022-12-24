FROM arm64v8/ros:humble-ros-base
SHELL [ "/bin/bash", "-c" ]
RUN apt-get update && apt-get install -y python3 python3-pip
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

RUN rm /ros_entrypoint.sh && echo "source /app/ros_entrypoint.sh" >> ~/.bashrc
EXPOSE 8080
ENTRYPOINT [ "/app/ros_entrypoint.sh" ]