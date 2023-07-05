# Define the worker hostname
WORKER_USERNAME=<username>
WORKER_HOSTNAME=<hostname>

# Check if we need to start the swarm on the manager
MANAGER_STATUS="$(docker info --format '{{.Swarm.LocalNodeState}}')"
if [ $MANAGER_STATUS == "inactive" ]; then
    docker swarm init &> /dev/null
    docker node update --label-add roft_deployer=true ${HOSTNAME} &> /dev/null
fi

# Then check if we need to join the swarm on the worker
WORKER_STATUS="$(ssh ${WORKER_USERNAME}@${WORKER_HOSTNAME} "docker info --format '{{.Swarm.LocalNodeState}}'")"
if [ $WORKER_STATUS == "inactive" ]; then
    ssh icub@${WORKER_HOSTNAME} `docker swarm join-token worker | tail -n 2` &> /dev/null
    docker node update --label-add ycb_cv_deployer=true ${WORKER_HOSTNAME} &> /dev/null
fi
