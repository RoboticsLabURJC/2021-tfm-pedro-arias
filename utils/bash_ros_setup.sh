#!/bin/bash
export PS1="\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\[\033[1;31m\]rbash\[\033[1;32m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "

export ROS_MASTER_URI="http://10.42.0.1:11311/"
export ROS_IP="10.42.0.180"

