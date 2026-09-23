# ========== ros2_img ========== 
FROM ubuntu:22.04 AS ros2_img

RUN apt-get update
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y --no-install-recommends tzdata

ENV TZ=Asia/Hong_Kong
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata 

RUN apt-get install --no-install-recommends locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y

ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends build-essential dos2unix \
        python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool || true && \
    dpkg --configure -a && \
    apt-get install -y --no-install-recommends build-essential dos2unix \
        python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init
RUN rosdep update

RUN echo '. /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

RUN apt install --no-install-recommends dumb-init

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN dos2unix /entrypoint.sh 
ENTRYPOINT [ "dumb-init", "--", "/entrypoint.sh" ]

# ========== xr_teleop_img ========== 
# FROM ros2_img AS xr_teleop_img

# RUN mkdir /cert
# WORKDIR /cert
# ENV CERT_FILE=cert.pem
# ENV KEY_FILE=key.pem
# ENV XR_TELEOP_CERT=/cert/${CERT_FILE}
# ENV XR_TELEOP_KEY=/cert/${KEY_FILE}

# RUN openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout ${XR_TELEOP_KEY} -out ${XR_TELEOP_CERT}

# ENV WS_NAME=xr_teleoperate
# RUN echo '. /root/${WS_NAME}/install/setup.bash' >> ~/.bashrc

# RUN mkdir -p /root/${WS_NAME}/src
# WORKDIR /root/${WS_NAME}
# COPY ./src ./src

# RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y
# RUN rm -rf /var/lib/apt/lists/*

# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+

# RUN rm -rf ./src 
