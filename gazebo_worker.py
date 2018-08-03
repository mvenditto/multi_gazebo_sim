import multiprocessing
import rospy
import time
import os
import subprocess
import os
from os import environ as env
from subprocess import Popen
from gazebo_proxy import GazeboProxy
from std_msgs.msg import Float32MultiArray
import websocket as ws

class GazeboWorker(multiprocessing.Process):
    def __init__(self, queue, gz_master=('127.0.0.1', 11346), ros_master=('127.0.0.1', 11350), world="custom_empty_0.world",quiet=True):
        super(GazeboWorker, self).__init__()
        self.gz_master_uri = "http://%s:%d" % gz_master
        self.ros_master_uri = "http://%s:%d" % ros_master
        self.ros_port = ros_master[1]
        self.gz_port = gz_master[1]
        self.world = world
        self.verbose = not quiet
        self.queue = queue
 
        self.gz_proxy = None
        self.ros_node = None
        self.joints_publisher = None
        self.joints_subscriber = None
        self.shutdown = False
        self.rate = None
 
 
    @staticmethod
    def joints_callback(self, data):
        pass
 
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
        self.joints_subscriber = rospy.Subscriber('/pexod/joint_positions', Float32MultiArray, self.joints_callback)
        self.joints_publisher = rospy.Publisher('/pexod/vel_cmd', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(1)
 
        print("instancing gazebo_server proxy @ %s" % self.gz_master_uri)
        #instance gazebo proxy -- blocks until gazebo services are available
        self.gz_proxy = GazeboProxy("gz_server_%d" % self.gz_port)
        print("all services online, done!")

    def _run_simulation(self, sim_data):
        client = ws.create_connection(sim_data["client_ws"])
        
        sim_start = sim_now = self._get_sim_time()
        elapsed_time = 0
        sim_duration = float(sim_data['duration'])
        
        while(sim_now - sim_start <= sim_duration):
            # do stuff
            sim_now = self._get_sim_time()
            client.send("current sim time: %s, gz_server_uri: %s, ros_master_uri" % (str(sim_now), self.gz_master_uri, self.ros_master_uri))

        client.close()    
        
 
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
 
        print("gazebo_worker %s is now online!" % self.gz_master_uri)

        while not self.shutdown:
            job = self.queue.get()
            self._run_simulation(job)
    
 
if __name__ == '__main__':
    gzw = GazeboWorker(quiet=True)
    gzw.run()
