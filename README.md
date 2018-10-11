# multi_gazebo_sim


## Installazione
    apt install ros-melodic ros-melodic-ros-controllers ros-melodic-ros-control
    pip3 install numpy deap asyncio websockets
    pip2 install numpy pybrain tornado websocket-client

## Plugin Build
    source /opt/ros/melodic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    extract extra/hexapod_pkg.tar.xz to catkin_ws/src/
    catkin_make
    source devel/setup.bash

## Esecuzione
    source /usr/share/gazebo/setup.sh
    source /opt/ros/melodic/setup.sh
    source <path-to>/catkin_ws/devel/setup.bash
### Evolution
shell-1:
    ```python2 gazebo_manager.py <n_process>```

shell-2:
    ```python3 deap_runner.py <n_process>```
  
### Testing
shell-1:
    ```python2 gazebo_manager.py 1```

shell-2:
    ```python3 simulation_master.py```

shell-3:
    ```GAZEBO_MASTER_URI=http://localhost:11000 gzclient```
  
## Server Monitoring Simulazioni
    sh visualization/run.sh

    *http://localhost:5000/sim* : stato simulazioni
    *http://localhost:5000/sim/info* : parametri simulazioni
    *http://localhost:5000/sim/summary* : grafico comparativo simulazioni
