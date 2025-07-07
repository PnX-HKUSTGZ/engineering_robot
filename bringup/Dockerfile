FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    wget \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

COPY . /root/code/rm_setup

RUN chmod +x /root/code/rm_setup/bringup.sh

WORKDIR /root/code/rm_setup

RUN /bin/bash -c "./bringup.sh -y"