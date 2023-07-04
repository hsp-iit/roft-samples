## Setup up the GPU-enabled swarm (once)

### Manager
On the chosen manager machine please do:

```console
docker swarm init
```

### Workers

For each worker to be added, on the `manager` please do:

```console
docker swarm join-token worker
```

The output will be something like
```console
docker swarm join --token <a_token> <manager_ip>:<manager_port>
```

Simply copy-paste, or use ssh to ease the process, the command **on the worker node**. However, please substitute the `<manager_ip>` with its hostname.

After that, we can also configure labels to workers devoted to run ML modules.

For each worker, on the `manager` please do:

```console
docker node update --label-add ycb_cv_deployer=true <worker_hostname>
```

### Configure GPUs on all machines (manager and workers)

For each machine, please do the following.

Find the GPU-ID first:

```console
nvidia-smi -a | grep UUID | awk '{print substr($4,0,12)}'
```

You will get something like `GPU-xxxxxxx`.

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

Please add as many GPUs as desired.

Then enable GPU advertising by uncommenting the line `swarm-resource = "DOCKER_RESOURCE_GPU"` in  `/etc/nvidia-container-runtime/config.toml`.

Finally, restart docker by issuing `sudo systemctl restart docker.service`.

## Deploy the stack (for each run)

```console
cd roft-samples/dockercompose
docker stack deploy -c docker-compose.yml roft-samples-handover-stack
```

After running the demo, stop all services and rm the stack:

```console
docker service update --replicas 0 roft
docker service update --replicas 0 detectron2
docker service update --replicas 0 dope
docker stack rm roft-samples-handover-stack
```
