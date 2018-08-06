from std_srvs.srv import Empty
from gazebo_msgs.srv import GetWorldProperties, GetModelState
import rospy
import os
 
class GazeboProxy():
        def __init__(self, namespace):
                self.gazebo_services = {}
                self.ns = namespace
 
                gazebo_services_ = [
                        ("/%s/pause_physics" % self.ns, Empty),
                        ("/%s/unpause_physics" % self.ns, Empty),
                        ("/%s/reset_simulation" % self.ns, Empty),
                        ("/%s/reset_world" % self.ns, Empty),
                        ("/%s/get_world_properties" % self.ns, GetWorldProperties),
                        ("/%s/get_model_state" % self.ns, GetModelState)
                ]
 
 
                for service in gazebo_services_:
                        name, type_ = service
                        rospy.wait_for_service(name)
                        self.gazebo_services[name] = rospy.ServiceProxy(name, type_)
 
        def pause(self):
                self.gazebo_services["/%s/pause_physics" % self.ns]()
 
        def unpause(self):
                self.gazebo_services["/%s/unpause_physics" % self.ns]()
 
        def reset_sim(self):
                self.gazebo_services["/%s/reset_simulation" % self.ns]()
 
        def reset_world(self):
                self.gazebo_services["/%s/reset_world" % self.ns]()  
 
        def get_world_properties(self):
                return self.gazebo_services["/%s/get_world_properties" % self.ns]()      

        def get_model_state(self, model_name, rel_entity_name):
                return self.gazebo_services["/%s/get_model_state" % self.ns](model_name, rel_entity_name)        
 