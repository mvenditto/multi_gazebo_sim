import tornado
import tornado.ioloop
import tornado.web
import tornado.template
from tornado.websocket import WebSocketHandler

import json

import multiprocessing
from gazebo_worker import GazeboWorker

job_queue = multiprocessing.Queue()

class WsGazeboManager(WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        print("open")

    def on_message(self, message):
        global job_queue
        try:
            json_data = json.loads(message)
            job_queue.put(json_data)
        except Exception as ex:
            print(ex)

    def on_close(self):
        job_queue.close()
 
if __name__ == '__main__':
   
    gazebo_ports = (11345, 11346, 11347, 11348)
    ros_ports = (11350, 11351, 11352, 11353)
    addr = "127.0.0.1"
    
    for gz_worker in zip(gazebo_ports, ros_ports):
        gz_port, ros_port = gz_worker
        gzw = GazeboWorker(job_queue, quiet=True, gz_master=(addr, gz_port), ros_master=(addr, ros_port))
        gzw.start()

    print("listening on port: 9090")

    application = tornado.web.Application([
      (r'/simulation-manager', WsGazeboManager)
    ])
 
    application.listen(9090)
    tornado.ioloop.IOLoop.instance().start()
