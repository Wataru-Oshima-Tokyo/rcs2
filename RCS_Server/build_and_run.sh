#!/bin/bash
architecture="$1"
if [ -z "$1" ]
    then
        echo "No argument given, you want make a specific docker please put the architecture such as arm64 or amd64,,, , please provide it as an argument."
        echo "in this case , since no argument is given, I will create a amd64 one"
        distro="amd64"
fi

docker build -f rcs_server_${architecture} -t rcs_server .


IP="$2"
if [ -z "$2" ]
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
