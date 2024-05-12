# Base image
FROM osrf/ros:noetic-desktop-full-focal

# Install Gazebo 11, ROS dependencies, and rostest
RUN apt-get update && apt-get install -y \
  gazebo11 \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-controller \
  ros-noetic-robot-state-publisher \
  ros-noetic-robot-localization \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-tools \
  ros-noetic-rostest \  
  git \
  && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 via Debian Packages
RUN apt update && apt install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3

# Make workspace
WORKDIR /catkin_ws

# Clone the necessary packages from GitHub
RUN git clone https://github.com/DrMoik/ros1_ci.git /catkin_ws/src/
# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make"

# Source the ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create a startup script
RUN echo "#!/bin/bash \n\
source /opt/ros/noetic/setup.bash \n\
source /catkin_ws/devel/setup.bash \n\
roslaunch tortoisebot_gazebo tortoisebot_playground.launch & \n\
rosrun tortoisebot_waypoints tortoisebot_action_server.py & \n\
wait" > /start.sh && chmod +x /start.sh

# Command to run when starting the container
CMD ["/bin/bash", "-c", "/start.sh"]
