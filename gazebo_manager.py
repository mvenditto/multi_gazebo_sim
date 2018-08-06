import tornado
import tornado.ioloop
import tornado.web
import tornado.template
import atexit
import sys
from tornado.websocket import WebSocketHandler, WebSocketClosedError

import json

import multiprocessing
from gazebo_worker import GazeboWorker

job_queue = multiprocessing.Queue()
audit_clients = [ ]

class WsGazebomManagerAudit(WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        if self not in audit_clients:
            audit_clients.append(self)

    def on_message(self, message):
        try:
            json_data = json.loads(message)
            if json_data["type"] == "sim_data":
                for c in audit_clients:
                    c.write_message(message)
        except WebSocketClosedError:
            pass
        except Exception as ex:
            print("msg_err:", ex)

    def on_close(self):
        audit_clients.remove(self)

class WsGazeboManager(WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        print("open")

    def on_message(self, message):
        try:
            json_data = json.loads(message)
            job_queue.put(json_data)
        except Exception as ex:
            print("msg_err:", ex)

    def on_close(self):
        print("closed")
 
if __name__ == '__main__':

    gazebo_ports = (11345, 11346, 11347, 11348)
    ros_ports = (11350, 11351, 11352, 11353)
    addr = "127.0.0.1"

    if len(sys.argv) > 1 and sys.argv[1] == "--debug":
        gazebo_ports = (11345,)
        ros_ports = (11350,)

    gz_procs = []
    
    for gz_worker in zip(gazebo_ports, ros_ports):
        gz_port, ros_port = gz_worker
        gzw = GazeboWorker(job_queue, quiet=True, gz_master=(addr, gz_port), ros_master=(addr, ros_port))
        gzw.start()
        gz_procs.append(gzw)

    def cleanup():
        print("gracefully shutdown...")
        for proc in gz_procs:
            job_queue.put("exit") 
            proc.shutdown = True
            proc.terminate()
        for proc in gz_procs:
            proc.join()
            print("shutting down gazebo_worker @ %s..." % proc.gz_master_uri)
        job_queue.close()


    atexit.register(cleanup)

    print("listening on port: 9090")

    application = tornado.web.Application([
      (r'/simulation-manager', WsGazeboManager),
      (r'/simulation-audit', WsGazebomManagerAudit)
    ])
 
    application.listen(9090)
    tornado.ioloop.IOLoop.instance().start()
