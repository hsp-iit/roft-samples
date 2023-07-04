# roft-samples

This repository hosts the code for running a sample application using [ROFT](https://github.com/hsp-iit/roft) and the `iCub` robot.

It allows tracking objects with the robot gaze and interact with it via handover actions in real-time:

<p align="center"><img src="https://github.com/hsp-iit/roft-samples/blob/master/assets/demo.webp"/></p>


The application needs:
1. a robot control module
2. an instance of the ROFT 6D object pose tracker
3. a segmentation algorithm
4. a 6D object pose estimation algorithm

We provide (1) and (2) within this repository in the form of a docker container.
A secondary container hosts (3) and (4) but it is not public yet, at the moment.


## Build the docker image for (1) and (2)

<details><summary>Click to expand.</summary>

Build the docker image:
```console
cd dockerfiles
bash build.sh # This will create an image named roft-samples-image:devel
```
</details>

## Build the docker image for (3) and (4)
<details><summary>Click to expand.</summary>

Not available at the moment.
</details>

## Setup a cluster with two machines

<details><summary>Click to expand.</summary>

Here we provide instructions on how to setup a cluster of two machines `machine_1` and `machine_2` (of course other configurations are possible). 
- `machine_1` needs:
   - an NVIDIA optical flow-enabled GPU (most GeForce RTX >= 20x0 cards)
   - the `roft-samples-image` docker image available
   - a docker engine (the most updated possible)
- `machine_2` needs:
   - two NVIDIA GPUs for running segmentation and pose estimation modules
   - the `ghcr.io/hsp-iit/ycb-pretrained-cv-models` docker image available (to be made available to users soon)
   - a docker engine

### Swarm setup
First we need to setup a `docker swarm` cluster with `machine_1` being the leader and `machine_2` a worker:

On `machine_1`:
```console
docker swarm init
docker swarm join-token worker
```

The output of the second command shall be copy-pasted on `machine_2`. After that, verify that all nodes are visible by issuing `docker node ls` on `machine_1`.

### Label assignment
For simplicity, we assign labels to the two machines as we use this mechanism to assign containers - and possibly swap `machine_1` and/or `machine_2` with others providing the same requirements if needed.

On `machine_1`:
```console
docker node update --label-add roft_deployer=true <machine_1_hostname>
docker node update --label-add ycb_cv_deployer=true <machine_2_hostname>
```

### GPUs configuration

We need to make the cluster aware of the GPUs available on each worker. For each machine do the following.

Find the GPU ids first:

```console
nvidia-smi -a | grep UUID | awk '{print substr($4,0,12)}'
```

Then edit `/etc/docker/daemon.json` such that it looks like:
```json
{
  "runtimes": {
    "nvidia": {
      "path": "/usr/bin/nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-runtime": "nvidia",
  "node-generic-resources": [
    "NVIDIA-GPU=<gpu_id_0>",
    "NVIDIA-GPU=<gpu_id_1>"
    ]
}
```

where `<gpu_id_x>` are provided by the output of the previous command.

Then enable GPU advertising by uncommenting the line `swarm-resource = "DOCKER_RESOURCE_GPU"` in  `/etc/nvidia-container-runtime/config.toml`.

Finally, restart docker by issuing `sudo systemctl restart docker.service`.

Nodes can be inspected using `docker node inspect <node_name>` to verify that the GPUs are correctly exposed.

</details>

## Deploy the stack

Assumptions:
- a `yarpserver` running on port`10000`. Please insert the IP address of the server in: https://github.com/hsp-iit/roft-samples/blob/fa4bb5ce925ba611dac09a3edb6be04031417760/dockercompose/docker-compose.yml#L10
- a stream of 640x480 RGB and depth images at 60 Hz is available within the ports `/depthCamera/rgbImage:o` and `/depthCamera/depthImage:o`.

#### Start the stack

```console
cd roft-samples/dockercompose
docker stack deploy -c docker-compose.yml roft-samples-handover-stack
```

An instance of `yarpmanager` will open automatically on a `roft-deployer` while the segmentation and pose estimation modules will be running within a `ycb_cv_deployer` headlessly. See [here](#setup-a-cluster-with-two-machines) for more details.

In the `yarpmanager` window open the application `ROFT Handover with iCub (embedded)` and just `run all` + `connect all`.
> Some modules / ports might remain unavailable - those are used to add speech functionality to the application and are optional.

#### Stop the stack

To stop the stack simply `stop all` the applications in the `yarpmanager`, wait for them to be closed and then do:
```
docker stack rm roft-samples-handover-stack
```

## :warning: Notes on the input RGBD image stream

<details><summary>Click to expand.</summary>

The demo has been tested solely using the RGBD streamer provided within this repository, `roft-samples-rs`, that works with `RealSense` cameras. 

Although all the software required to run it is provided in the `roft-samples-image` docker image, it cannot be run within the container as the `docker stack deploy` does not offer any mechanism to use `RealSense` cameras within the container being created. Indeed, it requires either the `--privileged` option to be available or the support to Linux `cgroups` to grant the container the access to the camera device. Although these options are both available when using `docker run` or `docker compose`, these are not available when using `docker stack deploy`.

Hence, the streamer should be installed outside the container. If the `robotology-superbuild` is used, the streamer can be easily installed as follows - assuming that `librealsense` is installed in the system:

```console
git clone https://github.com/xenvre/robots-io
cd robots-io && mkdir build && cd build
cmake -DUSE_YARP=ON -DUSE_ICUB=ON ../
make install
git clone https://github.com/hsp-iit/roft-samples
cd roft-samples && mkdir build && cd build
cmake -DBUILD_REALSENSE=ON ../
make install
```
The `roft-samples-rs` executable will then be available within the `bin` environmental path exposed by the `robotology-superbuild`. To start the camera streaming simply do:

```console
roft-samples-rs
```

### How to change the intrinsic parameters

The default configuration of the modules assumes that a `RealSense D405 camera` is used. The intrinsic parameters of such camera can be modified in several ways.

#### Persistent change
Once the stack is running, `docker exec` a bash shell interactively within the container running inside the `roft-deployer`:

```console
docker exec -it <name_of_the_container> bash
```

Then modify the following entries:

https://github.com/hsp-iit/roft-samples/blob/37c0c313433bcb0fbe198f7cdac064a773471626/src/roft/app/conf/config_d405.ini#L1-L9

within the file `~/.local/share/yarp/contexts/roft/config_d405.ini` - that is installed at docker build time (check [here](https://github.com/hsp-iit/roft-samples/blob/caca62f9989235531fe77160ada7f07d21964c80/dockerfiles/Dockerfile#L162)).

Then, commit the docker image:

```console
docker commit <name_of_the_container> roft-samples-image:devel
```

Finally, stop the stack and deploy it again to use the updated intrinsics.

#### Temporary change

If you only need to change the intrinsics temporarily, you can simply override them within the `Parameters` field of the `yarpmanager` for the module `roft` by appending:
```console
--CAMERA::fx <new_fx_value> --CAMERA::fy <new_fy_value> --CAMERA::cx <new_cx_value> --CAMERA::cy <new_cy_value>
```

</details>
