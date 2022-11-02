# roft-samples

A suite of applications based on ROFT

## Run the dockerized environment
1. Build the docker image:
    ```console
    cd dockerfiles
    bash build.sh # This will create an image named roft-samples-image
    ```
    > It is important to build the image on the target machine to ensure efficiency on the target platform.
1. Run the container:
   ```console
   cd dockerfiles
   bash run.sh # This will create and enter inside a container named roft-samples-cnt
   ```
   > Warning: This run command uses `--privileged` and `--volume=/dev:/dev` to simplify using the RealSense from within the docker. Please be careful.
   
   > You might need to `xhost +` your host to allow the container accessing the X server.
   
   > If you need to open multiple sessions within the container you can simply use `docker exec -it roft-samples-cnt bash`.
   
After the first run, you might access the container again using `docker start roft-samples-cnt` and `docker exec -it roft-samples-cnt bash`.

Once inside the container, run `yarpmanager` and refer to the applications `ROFT` and `ROFT_Handover_with_iCub`.
