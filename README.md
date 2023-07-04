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

Build the docker image:
```console
cd dockerfiles
bash build.sh # This will create an image named roft-samples-image
```

## Build the docker image for (3) and (4)
Not available at the moment.

## Setup a cluster with two machines

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
First we need to setup a `docker swarm` cluster with `machine_1` the leader and `machine_2` a worker:

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

## Deploy the stack

We assume that a `yarpserver` is already running within the network on port `10000`. Please insert the IP address of the server in: https://github.com/hsp-iit/roft-samples/blob/fa4bb5ce925ba611dac09a3edb6be04031417760/dockercompose/docker-compose.yml#L10

Then do the following:

```console
cd roft-samples/dockercompose
docker stack deploy -c docker-compose.yml roft-samples-handover-stack
```

After that, an instance of `yarpmanager` will open automatically on a `roft-deployer` while the segmentation and pose estimation modules will be running within a `ycb_cv_deployer` headlessly.

In the `yarpmanager` above please open the application `ROFT Handover with iCub (embedded)` and just `run all` + `connect all`. Some modules / ports might remain unavailable - those are used to add speech functionality to the appliation and are optional.

To stop the stack simply do
```
docker stack rm roft-samples-handover-stack
```

