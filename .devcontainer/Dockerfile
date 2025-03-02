FROM ros:humble
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3-pip \
        --no-install-recommends openssh-client \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir rerun-sdk

ENV SHELL /bin/bash

# Add ROS setup to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

USER $USERNAME
CMD ["/bin/bash"]