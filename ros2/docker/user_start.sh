#!/bin/bash

set -e

ip=`ip addr show wlp4s0 | grep inet | head -n 1 | awk '{print $2}' | cut -d "/" -f 1 `
foo=`grep "\- robot-user:" user-compose.yaml | cut -d ":" -f 2 | tr -d "\r"`

if [ ${ip} = ${foo} ]; then
    echo "it's all good, man"
    docker compose -f user-compose.yaml up -d --build
    docker attach ros
else
    echo "change ip in docker compose file"
    echo
    printf "\e[38;5;76mhost ip:    $ip\n"
    printf "\e[38;5;196mcompose ip: $foo\n"
    exit 1
fi
