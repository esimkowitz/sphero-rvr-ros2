FROM arm64v8/ros:humble-ros-base
RUN apt-get update && apt-get install -y python3 python3-pip

COPY . /rvr

WORKDIR /rvr/src/sphero-sdk-raspberry-python

RUN ["/bin/bash", "-c", ". ../sphero-sdk-first-time-setup.sh"]

WORKDIR /rvr

ENTRYPOINT [ "src/ros_entrypoint.sh" ]