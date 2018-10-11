# multi_gazebo_sim


## Installazione
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    apt update
    apt install ros-melodic-desktop-full ros-melodic-ros-controllers ros-melodic-ros-control
    sudo apt install python-pip python3-pip
    pip3 install numpy deap asyncio websockets flask matplotlib plotly
    pip2 install numpy pybrain tornado websocket-client scipy

## Plugin Build
    source /opt/ros/melodic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    cd consegna
    extract extra/hexapod_pkg.tar.xz to catkin_ws/src/
    catkin_make
    mkdir -p ~/.gazebo/models/hexapod 
    cp src/multi_gazebo_sim/model_plugin/sdf/model.* ~/.gazebo/models/hexapod
    sudo cp src/multi_gazebo_sim/worlds/*.world /usr/share/gazebo-9/worlds

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
