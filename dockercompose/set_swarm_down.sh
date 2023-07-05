# Define the worker hostname
WORKER_USERNAME=<username>
WORKER_HOSTNAME=<hostname>

# Check if we need to leave the swarm from within the worker
WORKER_STATUS="$(ssh ${WORKER_USERNAME}@${WORKER_HOSTNAME} "docker info --format '{{.Swarm.LocalNodeState}}'")"
if [ $WORKER_STATUS == "active" ]; then
    ssh icub@${WORKER_HOSTNAME} "docker swarm leave" &> /dev/null
    docker node rm --force ${WORKER_HOSTNAME} &> /dev/null
fi

# Check if we need to leave the swarm from within the manager
if [ "$(docker info --format '{{.Swarm.LocalNodeState}}')" == "active" ]; then
    docker swarm leave --force &> /dev/null
fi
