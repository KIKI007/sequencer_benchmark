FROM ubuntu:22.04
LABEL maintainer="qiqiustc@gmail.com"
LABEL version="1.0"
LABEL description="Computing Assembly Sequences."
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y libboost-all-dev cmake g++ gcc
RUN apt install -y git
RUN apt install -y libtbb-dev vim
