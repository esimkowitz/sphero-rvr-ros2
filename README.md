# Containerized RO2 Node for Sphero RVR

Pyhton 3 [ROS2](https://docs.ros.org/en/humble/#) node for interacting with [Sphero RVR](https://www.sphero.com/rvr).

Runs in Docker for easy integration and streamlined dependency management.

[![Docker Image CI](https://github.com/esimkowitz/sphero-rvr-ros2/actions/workflows/build-docker-image.yml/badge.svg)](https://github.com/esimkowitz/sphero-rvr-ros2/actions/workflows/build-docker-image.yml)

## Current Functionality

The rvr_node node supports the following commands:

- Start the RVR rolling in a given heading using the `rvr_start_roll` topic.
- Stop the RVR using the `rvr_stop_roll` topic.
- Adjust the RVR heading using the `change_heading` action.
- Ability to publish to the ROS2 topics anywhere on the network (provided you are on the same domain id as the RVR)

The robot_control node hosts a Flask website for sending commands remotely to the RVR. [See below](#control-using-the-robot_control-webpage) for more information.

The whole setup is fairly stable now thanks to improvements I've made in the executor loop, as well as mocked interfaces I've added to help when testing off of the Raspberry Pi.

## Future work

In the future, I plan to add the following:

- Sensor streaming to topics
- URDF specifications for better positioning, drive data, and simulations
- Potentially more advanced commands from the SDK like drive-to-x, etc.

### Run via Docker Compose

The easiest way to run the node is in Docker Compose using the script provided here: [run_docker_compose.sh](scripts/helper/run_docker_compose.sh).

The script specifies two optional flags:

- `-b`: Specifies whether to build the container image from source. By default, the script will use the version from Docker Hub.
- `-m`: Specifies whether to mock the RVR interface, which is useful when testing functionality off of the Raspberry Pi, such as on your development computer. By default, the interface is not mocked.

and one optional parameter:
- `-r`: Specifies which ROS2 Domain ID you want your RVR to be registered on. Default is 0.

The [docker-compose.yml](docker-compose.yml) file has the details on the run configuration, it functions the same as the command [below](#running-directly-via-docker) except without an interactive shell.

## Running directly via Docker

If you need to debug something directly on the container, you can log in using interactive mode using the following command:

```Bash
docker run -it --rm -v /dev/ttyS0:/dev/ttyS0 -e MOCK_RVR=false --privileged esimkowitz/sphero-rvr-ros2:latest -- bash
```

I have noticed that the exec shell will exit whenever there is a nonzero exit code for a command so I've gotten into the habit of starting a sh session and then another bash session within that, that way if bash exits I can quickly restart it. Otherwise, the container will close on any nonzero error code.

## Control using the robot_control webpage

By default, the docker-compose configuration will also launch a Flask app that can be used to control the robot remotely, mainly to demonstrate the performance of the ROS driver. This solution has been adapted from a previous robot project I was working on, [esimkowitz/RobotControl](https://github.com/esimkowitz/RobotControl).

To load the page, visit `http://<robot-ip>:8080` in your web browser. There is an on-screen joystick similar to the old Sphero app that lets you command both the speed and the heading. There is also support for the WASD and arrow keys.

Button functionality:

- Up/W: roll forward at a set speed
- Down/S: roll backward at a set speed
- Right/D: turn to the right by 10 degrees
- Left/A: turn to the left by 10 degrees

## Publish commands to node via ROS CLI

You can publish commands by opening a Bash shell on the container running in Docker Compose.

To start the Bash shell, run the following command:

```Bash
docker compose exec rvr_server bash
```

Then, you can use the commands in the [test_commands.sh](scripts/helper/test_commands.sh) file to publish commands to the running node.

## Credit

Thank you [@DomnikN](https://github.com/DominikN) for your very well documented project [DominikN/ros2_docker_examples](https://github.com/DominikN/ros2_docker_examples).

Thank you [@gumstix/Altium](https://github.com/gumstix) for your [RVR ROS2 Node demo](https://github.com/gumstix/PKG900000001506/tree/master/demo/Sphero%20RVR/ros2%20node).

## License

MIT &copy; Evan Simkowitz, 2023, see [LICENSE](LICENSE.md) for additional licenses.
