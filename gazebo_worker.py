import numpy as np
import multiprocessing
import rospy
import time
import os
import subprocess
import os
import json
import websocket as ws
from os import environ as env
from subprocess import Popen
from gazebo_proxy import GazeboProxy
from std_msgs.msg import Float32MultiArray

from pybrain.structure.modules.relulayer import ReluLayer
from pybrain.structure import SoftmaxLayer
from pybrain.tools.shortcuts import buildNetwork

class GazeboWorker(multiprocessing.Process):
    def __init__(self, queue, 
            gz_master=('127.0.0.1', 11346), 
            ros_master=('127.0.0.1', 11350), 
            world="custom_empty.world", 
            quiet=False,
            audit_ws_uri="ws://127.0.0.1:9090/simulation-audit"
        ):
        super(GazeboWorker, self).__init__()
        self.gz_master_uri = "http://%s:%d" % gz_master
        self.ros_master_uri = "http://%s:%d" % ros_master
        self.ros_port = ros_master[1]
        self.gz_port = gz_master[1]
        self.world = world
        self.verbose = not quiet
        self.queue = queue
 
        self.sim_seq = 0
        self.gz_proxy = None
        self.ros_node = None
        self.joints_publisher = None
        self.joints_subscriber = None
        self.shutdown = False
        self.rate = None

        self.send_velocity_thresh = 1.0
        self.audit_ws_uri = audit_ws_uri
        self.audit_ws = None 

        self.brain = None
        self.running_sim = False

    def spawn_process(self, args):
        if not self.verbose:
            Popen(args, stdin=None, stdout=open(os.devnull, 'wb'), stderr=open(os.devnull, 'wb'))
        else:
            Popen(args)
 
    def _get_sim_time(self):
        return self.gz_proxy.get_world_properties().sim_time
 
    def _start_ros_master(self):
        args = ['roscore', '-p', str(self.ros_port)]
        self.spawn_process(args)
 
    def _start_gz_master(self):
        world_path = os.path.join(env["GAZEBO_RESOURCE_PATH"].replace(":", ""), "worlds", self.world)
 
        args = [
            'rosrun',
            'gazebo_ros',
            'gzserver',
            '--verbose' if self.verbose else '',
            '__name:=gz_server_%d' % self.gz_port,
            'paused:=true',
            world_path]
 
        self.spawn_process(args)
       
    def _start_ros_node(self):
        self.ros_node = rospy.init_node("gz_worker_%d" % self.gz_port, anonymous=True)
        self.joints_subscriber = rospy.Subscriber('/pexod/joint_positions', Float32MultiArray, self.move_model)
        self.joints_publisher = rospy.Publisher('/pexod/vel_cmd', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(1)
 
        print("instancing gazebo_server proxy @ %s" % self.gz_master_uri)
        #instance gazebo proxy -- blocks until gazebo services are available
        self.gz_proxy = GazeboProxy("gz_server_%d" % self.gz_port)

    def _start_neural_net(self):
        self.brain = buildNetwork(12, 12, 12, hiddenclass=ReluLayer, outclass=SoftmaxLayer)

    def _feed_brain(self, joint_values):
        return self.brain.activate(joint_values.data)
         
    def _move_joint(self, values, pub):
        msg = Float32MultiArray(data=values)
        pub.publish(msg)

    def move_model(self, joint_values):
        if self.running_sim:
            res = self._feed_brain(joint_values).tolist()
            self._move_joint(res, self.joints_publisher)

    def _init_net(self, weights):
        self.brain._setParameters(np.array(weights))

    def _run_simulation(self, sim_data):
        client = ws.create_connection(sim_data["client_ws"])
        print(client)
        if self.audit_ws is None:
            self.audit_ws = ws.create_connection(self.audit_ws_uri)

        sim_id = "{0}_{1}".format(self.gz_port, self.sim_seq)
        print("starting simulation_run[{0} -> {1}]".format(sim_id, sim_data["client_ws"]))

        try:
            self.gz_proxy.reset_world()
            self.gz_proxy.unpause()    
            self._init_net(sim_data['weights'])

            sim_start = sim_now = self._get_sim_time()
            elapsed_time = 0
            sim_duration = float(sim_data['duration'])

            first = True
            last_pos = (0,0)

            sim_status = "completed"
            self.running_sim = True
            while(sim_now - sim_start <= sim_duration):
                model_state = self.gz_proxy.get_model_state('pexod', '')
                model_pos = model_state.pose.position
                dx = abs(model_pos.x) - abs(last_pos[0])
                dy = abs(model_pos.y) - abs(last_pos[1])

                if first or (dx >= self.send_velocity_thresh or dy >= self.send_velocity_thresh):    
                    msg = {
                        "sim_id": sim_id,
                        "type": "sim_data",
                        "x": model_pos.x,
                        "y": model_pos.y,
                        "z": model_pos.z
                    }
                    self.audit_ws.send(json.dumps(msg))
                    first = False

                last_pos = (model_pos.x, model_pos.y)
                sim_now = self._get_sim_time()
            
        except Exception as ex:
            sim_status = "error"
            print("an error occurred: {0}".format(str(ex)))
        finally:
            self.gz_proxy.pause()
            client.send(json.dumps({"type": "sim_end","sim_id": sim_id, "result":last_pos,"status":sim_status}))
            client.close()
            self.running_sim = False
            self.sim_seq += 1    
            print("simulation_run[{0}] terminated".format(sim_id))
 
    def run(self):
        # setup environment for this process
        env['ROS_MASTER_URI'] = self.ros_master_uri
        env['GAZEBO_MASTER_URI'] = self.gz_master_uri
 
        #start rosmaster for this gazebo instance
        print("launching ros master @ %s" % self.ros_master_uri)
        self._start_ros_master()
        #start gazebo master
        print("launching gazebo master @ %s" % self.gz_master_uri)
        self._start_gz_master()
        #create ros node
        print("launching ros node")
        self._start_ros_node()

        print("creating neaural_network model")
        self._start_neural_net()
 
        print("gazebo_worker %s is now online!" % self.gz_master_uri)

        while not self.shutdown:
            job = self.queue.get()
            print("{0}|{1}|{2}".format(self.ros_master_uri, self.gz_master_uri, job["client_ws"]))
            if job == 'exit':
                break
            self._run_simulation(job)

        if self.audit_ws is not None:
            self.audit_ws.close()
    
 
if __name__ == '__main__':
    gzw = GazeboWorker(quiet=True)
    gzw.run()
