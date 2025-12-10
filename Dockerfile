# Start with the official ROS noetic base image
FROM arm64v8/ros:noetic

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
  git \
  python3-pip \
  build-essential \
  nano \
  vim \
  && rm -rf /var/lib/apt/lists/*
#  APT stores downloaded package index files in /var/lib/apt/lists/. 
#  After installation, these are no longer needed.


# Install Python package pyzmq
RUN pip3 install --upgrade pip setuptools wheel \
 && pip3 install --upgrade numpy \
 && pip3 install pyzmq ping3

# Set up the workspace in the container
WORKDIR /workspace

# Copy your ROS package into the container
COPY src/ /workspace/src

# Build workspace
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set up the workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# entry point script
COPY entrypoint_noetic.sh /entrypoint_noetic.sh 

ENTRYPOINT ["/bin/bash", "/entrypoint_noetic.sh"]
