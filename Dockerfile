FROM ros:noetic

# Set up the timezone
ENV TZ=Asia/Jakarta
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# # Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     ros-noetic-rosbridge-suite \
#     ros-noetic-vision-opencv \
#     ros-noetic-image-common \
#     ros-noetic-web-video-server \
#     && rm -rf /var/lib/apt/lists/*

# Install other dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libserialport-dev \
    libcurl4-openssl-dev \
    curl \
    unzip \
    nano \
    apache2 \
    php \
    libapache2-mod-php \
    php-zip \
    php-mbstring \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN /bin/bash -c 'pip3 install mysql-connector-python prometheus_client schedule'

# Make directory for application and application data
RUN /bin/bash -c 'mkdir -p /root/slff-data'
RUN /bin/bash -c 'mkdir -p /root/slff'

# Copy application source
COPY ./src /root/slff/src
COPY ./slff_entrypoint.sh /slff_entrypoint.sh

# Setup tinyfilemanager
RUN rm -rf /var/www/html/*
COPY ./src/slff/misc/tinyfilemanager /var/www/html

# #Setup rclone
# RUN curl https://rclone.org/install.sh | bash
# COPY ./src/slff/misc/rclone/rclone.conf /root/.config/rclone/rclone.conf

# Executable permission
RUN /bin/bash -c 'chmod +x /slff_entrypoint.sh'
RUN /bin/bash -c 'chmod +x /root/slff/src/slff/scripts/*'

# Compile application source
RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && cd /root/slff && catkin_make'

ENTRYPOINT [ "/slff_entrypoint.sh" ]
CMD [ "roslaunch", "slff", "slff.launch" ]