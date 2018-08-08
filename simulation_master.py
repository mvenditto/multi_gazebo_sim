import asyncio
import websockets
import multiprocessing
import json
import numpy as np
 
ports = range(11360, 11368)
m = multiprocessing.Manager()
ports_queue = m.Queue()
for p in ports:
    ports_queue.put(p)

res = {}

def fitness(sim_result):
    try:
        x,y = sim_result["result"]
        return x + y
    except:
        return float('-inf')

def listen_sim_data(port, job):
    print("listen_sim_data 1")
    async def receive(websocket, path):
        try:
            
            data = await websocket.recv()
            print("before data: {0}".format(data))
            res[port]=json.loads(data)
        except Exception as ex:
            print(ex)
        finally:
            websocket.close()
            print("ws close {0}".format(port))
            asyncio.get_event_loop().stop()

    print("binding to {0}".format(port))
    start_server = websockets.serve(receive, 'localhost', port)

    asyncio.get_event_loop().run_until_complete(start_server)
    run_sim(job, "ws://127.0.0.1:9090/simulation-manager")
    asyncio.get_event_loop().run_forever()
    fitness_value = fitness(res[port])
    print(fitness_value, port)
    del res[port]
    print("listen_sim_data 2")
    ports_queue.put(port)
    print("port[{0}] released!".format(port))
    return fitness_value

def run_sim(job, sim_manager_uri):
    job_str = json.dumps(job)
    async def submit_job():
        async with websockets.connect(sim_manager_uri) as websocket:
            await websocket.send(job_str)
        
    asyncio.get_event_loop().run_until_complete(submit_job())

def evaluate(indiv, ports_queue=None):
    port = ports_queue.get()
    weights = indiv.tolist()
    print(weights[0:10])
    job = {
        "duration":1,
        "client_ws":"ws://localhost:{0}".format(port),
        "weights":weights
    }
    listen_sim_data(port, job)


