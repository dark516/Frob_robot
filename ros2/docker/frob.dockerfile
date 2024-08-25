FROM osrf/ros:iron-desktop-full
MAINTAINER Alexander Kulagin sashakulagin2007@gmail.com

RUN apt-get update

WORKDIR /root/ros
RUN apt update && apt install -y -q --no-install-recommends tmux wget curl gnupg git \
								python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
								iproute2 iputils-ping
RUN mkdir -p ~/.config/tmux src && \
    wget https://raw.githubusercontent.com/CyberFatherRT/dotfiles/master/tmux/tmux.conf -O ~/.tmux.conf && \
	git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm



WORKDIR /root/ros_ws
#COPY ../../src/ros/. ./src/
# RUN git clone https://github.com/dark516/ros-robot /tmp && \
# 	mv /tmp/ros/* src && \
# 	rm -rf /tmp/ros

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
ENTRYPOINT ["tmux"]
