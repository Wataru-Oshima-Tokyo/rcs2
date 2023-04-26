#!/bin/bash

docker build  . -t rcs_server

IP="$1"
if [ -z "$1" ]
    then
        echo "No argument given, If an IP is required, please provide it as an argument. Using [localhost] for this run."
        IP="localhost"
fi

docker run -it \
  --rm \
  --privileged \
  --net=host \
  --volume="${PWD}/":"/external:rw" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  rcs_server bash -c "cd external/rcs_server;  ./server_bringup $IP"
