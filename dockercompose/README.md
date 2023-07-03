### Setting up GPU-enabled swarm (once)

#### Manager

#### Workers

### Deploy the stack (for each run)

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
