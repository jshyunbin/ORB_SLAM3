# Registered at https://hub.docker.com/repository/docker/jshyunbin/orb_slam3/general
FROM ubuntu:22.04

# copy files over
ADD . /ORB_SLAM3

# use non-interactive
ENV TZ=America/Los_Angeles
ENV DEBIAN_FRONTEND=noninteractive 

# install pangolin prereq
RUN apt-get update && apt install -y sudo python3-dev python3-setuptools python3-wheel apt-transport-https
RUN echo "Y" | bash /ORB_SLAM3/Thirdparty/Pangolin/scripts/install_prerequisites.sh

# install orb slam prereq
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update

RUN apt install -y build-essential libopencv-dev libboost-dev libboost-serialization-dev libssl-dev librealsense2-dkms librealsense2-utils librealsense2-dev

# build
RUN cd /ORB_SLAM3 && bash build.sh
