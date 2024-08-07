FROM osrf/ros:humble-desktop-full

SHELL ["bash", "-c"]

# Create the user 'mustafa'
RUN useradd -m -s /bin/bash mustafa

# Add 'mustafa' to sudoers with no password requirement
RUN echo 'mustafa ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers


# Create and set permissions for the directory where you want to copy files
RUN mkdir -p /home/mustafa && chown -R mustafa:mustafa /home/mustafa

# Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3*

RUN apt install python3-pip -y && pip3 install gymnasium && pip3 install stable-baselines3 &&  pip install numpy==1.26.4 && pip3 install tensorboard && pip install stable-baselines3[extra]

# Switch to the user 'mustafa'
USER mustafa

# Create a directory for the workspace in the user's home directory
RUN mkdir -p /home/mustafa/rl_planner/src

# Clone the given repository to the workspace
RUN cd /home/mustafa/rl_planner/src && git clone https://github.com/mustafakurban/rl_test_enviroment.git

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> /home/mustafa/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/mustafa/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /home/mustafa/.bashrc

# Build the workspace
RUN source /opt/ros/humble/setup.bash && cd /home/mustafa/rl_planner && colcon build --symlink-install

# Set the working directory
WORKDIR /home/mustafa
