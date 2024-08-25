FROM ubuntu:focal

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt update && \
    apt install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -q -y --no-install-recommends dirmngr gnupg2 && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros1-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"

RUN echo "deb [ signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

RUN apt update && \
    apt install -y --no-install-recommends ros-noetic-ros-core=1.5.0-1* && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install --no-install-recommends -y build-essential python3-rosdep python3-rosinstall python3-vcstools && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update # --rosdistro $ROS_DISTRO
RUN apt update && \
    apt install -y --no-install-recommends ros-noetic-ros-base=1.5.0-1* && \
    rm -rf /var/lib/apt/lists/*
RUN apt update && \
    apt install -y --no-install-recommends ros-noetic-robot=1.5.0-1* && \
    rm -rf /var/lib/apt/lists/*
RUN apt update && apt install -y --no-install-recommends ros-noetic-desktop=1.5.0-1* && \
    rm -rf /var/lib/apt/lists/* # buildkit
RUN apt update && apt install -y --no-install-recommends ros-noetic-desktop-full=1.5.0-1* && \
    rm -rf /var/lib/apt/lists/* # buildkit
