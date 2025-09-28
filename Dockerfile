FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Basic tools
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libffi-dev \
    liblzma-dev \
    python3-openssl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install pyenv
RUN git clone https://github.com/pyenv/pyenv.git /root/.pyenv

ENV PYENV_ROOT="/root/.pyenv"
ENV PATH="$PYENV_ROOT/bin:$PATH"

# Install Python 3.10.x via pyenv
RUN pyenv install 3.10.14 \
    && pyenv global 3.10.14

# Ensure pyenv shims work in bash
RUN echo 'export PYENV_ROOT="$HOME/.pyenv"' >> /root/.bashrc \
    && echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> /root/.bashrc \
    && echo 'eval "$(pyenv init -)"' >> /root/.bashrc

# Install pip packages for ROS2 dev using pyenv Python
RUN export PYENV_ROOT="/root/.pyenv" \
    && export PATH="$PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH" \
    && python -m ensurepip --upgrade \
    && python -m pip install --upgrade pip \
    && python -m pip install colcon-common-extensions numpy matplotlib pyyaml

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update \
    && apt-get install -y ros-humble-desktop ros-humble-turtlesim ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Setup colorful prompt with pyenv/conda (Python version leftmost) and container user@container_name 
RUN echo '# Function to show Python info' >> /root/.bashrc \ 
    && echo 'parse_python() {' >> /root/.bashrc \ 
        && echo ' if [ -n "$CONDA_DEFAULT_ENV" ]; then' >> /root/.bashrc \ 
            && echo ' py_info="$CONDA_DEFAULT_ENV"' >> /root/.bashrc \ 
        && echo ' elif command -v pyenv >/dev/null 2>&1; then' >> /root/.bashrc \ 
            && echo ' py_info="$(pyenv version-name 2>/dev/null)"' >> /root/.bashrc \ 
        && echo ' else' >> /root/.bashrc \ 
            && echo ' py_info="$(python --version 2>&1 | awk '\''{print $2}'\'')"' >> /root/.bashrc \ 
        && echo ' fi' >> /root/.bashrc \ 
        && echo ' echo "($py_info)"' >> /root/.bashrc \ 
    && echo '}' >> /root/.bashrc \ 
    \ 
    && echo '# Colored prompt: yellow Python version, green container user@container_name, blue cwd' >> /root/.bashrc \
    && echo 'export PS1="\[\e[33m\]\$(parse_python)\[\e[m\]\[\e[32m\]\$(whoami)\[\e[m\]:\[\e[34m\]\w\[\e[m\]\$ "' >> /root/.bashrc \
    \
    && echo '# Aliases for colorized commands' >> /root/.bashrc \
    && echo "alias ls='ls --color=auto'" >> /root/.bashrc \
    && echo "alias ll='ls -l --color=auto'" >> /root/.bashrc \
    && echo "alias grep='grep --color=auto'" >> /root/.bashrc \
    \
    && echo '# Source ROS2 Humble and workspace' >> /root/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
