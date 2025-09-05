FROM osrf/ros:humble-desktop-full
ARG USERNAME=wolfi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-venv
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    cmake git tree ros-humble-rqt* ros-humble-rviz2 ros-humble-rmw-cyclonedds-cpp ros-humble-robot-localization\
    && rm -rf /var/lib/apt/lists/*
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
RUN pip3 install --no-cache-dir \ 
    "numpy==1.23.5" \
    "scipy==1.10.1" \
    "pandas==1.5.3" \
    "black==24.4.2" \
    "filterpy==1.4.5"

# ROS setup automatisch sourcen
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /home/$USERNAME/.bashrc
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]