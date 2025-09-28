# Base OS: Ubuntu 22.04
FROM ubuntu:22.04

# Set non-interactive for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    build-essential \
    software-properties-common \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda (latest Python 3)
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -p /opt/conda \
    && rm /tmp/miniconda.sh

# Set conda path
ENV PATH=/opt/conda/bin:$PATH

# Create a conda environment inside the image with Python & dev tools
# Accept ToS non-interactively and create environment
RUN conda config --set always_yes yes --set changeps1 no \
    && conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main \
    && conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r \
    && conda create -y -n ros2_cpp python=3.11 cmake eigen ninja pyyaml numpy matplotlib \
    && conda clean -afy


# Activate conda environment by default
SHELL ["conda", "run", "-n", "ros2_cpp", "/bin/bash", "-c"]

# Install ROS2 Humble (system-level)
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update \
    && apt-get install -y ros-humble-desktop ros-humble-turtlesim ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Default command
CMD ["bash"]
