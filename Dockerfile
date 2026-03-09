FROM ros:noetic-ros-base-focal

# All dependencies baked in — no network needed at runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-cv-bridge \
    python3-opencv \
    python3-numpy \
    python3-pip \
    ipython3 \
    libusb-1.0-0-dev \
    udev \
    setserial \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt

COPY catkin_ws/src /catkin_ws/src
WORKDIR /catkin_ws

# Build the workspace (joy_manager_msgs + anymal_custom_control)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_ENABLE_TESTING=OFF"

# Auto-source workspace on shell entry
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
