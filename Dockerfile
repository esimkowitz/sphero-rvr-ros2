FROM arm64v8/ros:humble-ros-base
RUN apt-get update && apt-get install -y python3 python3-pip

COPY . /app

WORKDIR /app/sphero-sdk/sphero-sdk-raspberry-python

RUN ["/bin/bash", "-c", ". ../sphero-sdk-first-time-setup.sh"]

WORKDIR /app

ENTRYPOINT [ "/ros_entrypoint.sh" ]