# [POSTECH CompSec Lab]
## A Framework for Generating Datasets for Detecting Swarm Drone Attacks

### Simulation Environment

|||
|---|---|
|**Simulation Mode**|SITL(Software-In-The-Loop|
|**Autopilot Software**|PX4|
|**Simulator**|jMAVsim|
|**Drone Communication Library**|PyMAVLink|
|**Ground Control Station**|QGroundControl(QGC)|

### Build

In `~/PX4-Autopilot` directory, run the following command.
```
make px4_sitl_default
```
In `~/PX4-Autopilot/Tools/simulation` directory, run the following command.
```
./sitl_multiple_run.sh 2
```
In `~/PX4-Autopilot/Tools/simulation/jmavsim` directory, run the following commands in separate terminals.
```
./jmavsim_run.sh -l
./jmavsim_run.sh -p 4561 -l
```
In `~/CSED499I_COMMAND` directory, run the following command in new terminal.
```
python3 leader-follower.py
```
