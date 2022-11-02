docker run \
       -it --privileged \
       --name roft-samples-cnt \
       --user user \
       --net=host \
       -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v /dev:/dev \
       --gpus all --runtime=nvidia \
       roft-samples-img
