# Containerized RO2 Node for Sphero RVR

Pyhton3 based [ROS2](https://docs.ros.org/en/humble/#) node for interacting with [Sphero RVR](https://www.sphero.com/rvr).

Runs in Docker for easy integration and streamlined dependency management.

[![Docker Image CI](https://github.com/esimkowitz/sphero-rvr-ros2/actions/workflows/build-docker-image.yml/badge.svg)](https://github.com/esimkowitz/sphero-rvr-ros2/actions/workflows/build-docker-image.yml)

**This is a VERY early work in progress**

## Current Functionality

- Change leds using the `rvr_change_leds` topic.
- Start the RVR rolling using the `rvr_start_roll` topic.
- Stop the RVR using the `rvr_stop_roll` topic.
- Change the RVR heading using the `rvr_set_heading` topic.
- Reset the RVR heading using the `rvr_reset_heading` topic.

## Running directly via Docker

For now I am running the Docker container in interactive mode so that I can start a TMux session and quickly evaluate changes. I am using the following commands to do this:

```Bash
docker run -it --rm -v /dev/ttyS0:/dev/ttyS0 --privileged esimkowitz/sphero-rvr-ros2:latest -- bash
```

I have noticed that the exec shell will exit whenever there is a nonzero exit code for a command so I've gotten into the habit of starting a sh session and then another bash session within that, that way if bash exits I can quickly restart it. Otherwise, the container will close on any nonzero error code.

## Running via Docker Compose

I've added a [docker-compose.yml](docker-compose.yml) file to run the ROS2 node in the background. You can run it via 

### Run and build via script

I've added a shell script that can build the image before starting the Docker Compose for easier development: [run_docker_compose.sh](run_docker_compose.sh).

### Run using pre-built image

```Bash
sudo docker compose up
```

## Publish commands to node

You can publish commands by opening a Bash shell on the container running in Docker Compose.

To start the Bash shell, run the following command:

```Bash
sudo docker compose exec rvr_server bash
```

Then, you can use the commands in the [test_commands.sh](test_commands.sh) file to publish commands to the running node.

## Credit

Thank you [@DomnikN](https://github.com/DominikN) for your very well documented project [DominikN/ros2_docker_examples](https://github.com/DominikN/ros2_docker_examples).

Thank you [@gumstix/Altium](https://github.com/gumstix) for your [RVR ROS2 Node demo](https://github.com/gumstix/PKG900000001506/tree/master/demo/Sphero%20RVR/ros2%20node).

## License

MIT &copy; Evan Simkowitz, 2022
