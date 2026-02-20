FROM osrf/ros:humble-desktop-jammy

ARG USERNAME=kodifly

# Create a non-root user and set up the environment
RUN useradd ${USERNAME} --create-home --shell /bin/bash  && \
    echo ${USERNAME}':newpassword' | chpasswd && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} && \
    chmod 777 /home/${USERNAME}

# Update 'admin' user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video && adduser ${USERNAME} plugdev && adduser ${USERNAME} sudo

# add user to groups for serial access
RUN sudo usermod -a -G tty ${USERNAME} && \
    sudo usermod -a -G dialout ${USERNAME}

# Create .ssh dir with secure permissions
RUN mkdir -p /home/${USERNAME}/.ssh && \
    chmod 700 /home/${USERNAME}/.ssh && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh

# Mount secret and copy with proper permissions
RUN --mount=type=secret,id=host_ssh_key,target=/run/secrets/host_ssh_key \
    install -m 600 /run/secrets/host_ssh_key /home/${USERNAME}/.ssh/id_rsa && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/id_rsa

# Add GitHub to known_hosts
RUN ssh-keyscan github.com >> /home/${USERNAME}/.ssh/known_hosts && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/known_hosts

# Optional: configure SSH to use the key by default
RUN echo "IdentityFile /home/${USERNAME}/.ssh/id_rsa" >> /home/${USERNAME}/.ssh/config && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/config && \
    chmod 600 /home/${USERNAME}/.ssh/config



SHELL ["/bin/bash", "-c"]

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Build ROS workspace
WORKDIR /home/${USERNAME}/workspace/src
RUN git clone git@github.com:gc625-kodifly/livox_ros2_driver.git  
WORKDIR /home/${USERNAME}/workspace
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/${USERNAME}/workspace/install/setup.bash" >> ~/.bashrc

COPY --chmod=755 docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["/bin/bash"]
