FROM dark516/ros
MAINTAINER Alexander Kulagin sashakulagin2007@gmail.com

RUN apt-get update

WORKDIR /root/ros
RUN apt update && apt install -y -q --no-install-recommends tmux wget curl gnupg git \
								python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
								iproute2 iputils-ping
RUN mkdir -p ~/.config/tmux src && \
    wget https://raw.githubusercontent.com/CyberFatherRT/dotfiles/master/tmux/tmux.conf -O ~/.tmux.conf && \
	git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm

RUN bash -c ". /opt/ros/noetic/setup.bash && catkin_make"
RUN echo "source ~/ros/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/ros
COPY ../../src/ros/. ./src/
# RUN git clone https://github.com/dark516/ros-robot /tmp && \
# 	mv /tmp/ros/* src && \
# 	rm -rf /tmp/ros
RUN apt install -y ros-noetic-gmapping ros-noetic-joy

WORKDIR /root
RUN git clone https://github.com/WiringPi/WiringPi.git
RUN cd WiringPi && ./build

WORKDIR /root/ros
RUN bash -c ". /opt/ros/noetic/setup.bash && catkin_make"
RUN apt-get install ros-noetic-pid

RUN echo "source ~/ros/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["tmux"]
