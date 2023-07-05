## Detailed instructions on how to run the handover demo on iCubGenova11

:warning: Differently from the generic documentation in the main `README.md` of this repository, these instructions will let you initialize and shutdown the docker swarm every time the demonstration is executed - helper scripts are provided. This is important to restore the status of the machines involved in the demonstration such that docker can run locally in non-swarm mode - as required normally.

#### Objects
- Go and pick YCB `006_mustard_bottle` and `004_sugar_box`

#### External display
- Turn on the wall-mounted display

#### Robot
- Turn on the robot power supply, CPU and motors

#### Microphone receiver system
- Turn on the Sennheiser microphone receiver
- Turn on the Sennheiser wireless micrphone unit

#### IITICUBLAP235
- Connect HDMI cable of the external display
- Connect USB cable of the RealSense D405 camera
- Connect USB cable of the M-Audio USB sound card
- Turn on the laptop

#### IITICUBLAP232
- Connect external GPU and switch it on
- Make sure the laptop is connected to the robot network using the proper cable
- Turn on the laptop
- Check that both the GPU are up and running by launching `nvidia-smi`

#### Both laptops
- Make sure docker is running, if not please `systemctl start docker.service`

#### Start the robot w/ camera streaming
- Open `yarpmanager` on `iiticublap235`
  - `Cluster` section:
    - Run `yarpserver` on `iiticublap235`
    - Run `yarprun` on `iiticublap235` and `icub-head`
  - `Entities` section:
    - Open `iCubStartup (iCubGenova11)` and run, in order, (0), (1) and, after the robot has started succesfully, (2), (3) and (4)
    - Open `Realsense (ROFT)` and run all, connect all

#### Start the speech deployer (optional)
- Open `VirtualBox` on `iiticublap235`
- Run the only available appliance
- Once inside, run the `bat` script on the Desktop

#### Deploy the stack
```console
cd $ROBOT_CODE/icub-contrib-iit/roft-samples/dockercompose
./set_swarm_up.sh
docker stack deploy -c docker-compose.yml stack
```

A `yarpmanager` will open. There, select `ROFT_Handover_with_iCub (embedded)` and run all, connect all. Please check that everything is green.

#### Switch object between YCB `006_mustard_bottle` and `004_sugar_box`

#### Via RPC
```console
yarp rpc /roft-samples-handover/rpc:i
> select_object <id>
```
where `<id>` is `o006` or `o004`.

#### Via microphone
Unmute the microphone and say `Let's play with mustard` or `Let's play with sugar` for the two objects respectively.

#### Shutdown the stack
Stop all in the `yarpmanager` and wait for everything to be red. Then:

```console
cd $ROBOT_CODE/icub-contrib-iit/roft-samples/dockercompose
docker stack rm stack
./set_swarm_down.sh
```

#### Final shutdown
- Stop all applications in `Realsense (ROFT)` and `iCubStartup (iCubGenova11)` in reversed order
- Power off the head of the robot `ssh icub-head; sudo poweroff`
- Turn off the two laptops, the microphone, the Sennheiser receiver, the external GPU and the external display
- Reconnect back `iiticublap232` to the external network
- Please, disconnect all the cables and leave the desk tidy and clean as it was before!
- Please, move back the YCB objects in their usual place!
