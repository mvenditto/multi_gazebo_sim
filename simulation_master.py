import asyncio
import websockets
import multiprocessing
import json
import numpy as np
 
ports = range(14000, 14050)
m = multiprocessing.Manager()
ports_queue = m.Queue()
for p in ports:
    ports_queue.put(p)

SIM_MANAGER_URI = "ws://127.0.0.1:9090/simulation-manager"
res = {}

Z_THRESH = 0.20
DEF_SIM_DURATION_S = 20
INVALID_FITNESS = (float('-inf'),)

def fitness(sim_result):
    try:
        x,y,max_z = sim_result["result"]
        if max_z >= Z_THRESH:
            return INVALID_FITNESS
        return (x,)
    except Exception as ex:
        print(ex)
        return INVALID_FITNESS

def send_job_and_wait(port, job):
    
    async def receive_sim_data(websocket, path):
        try:
            data = await websocket.recv()
            res[port] = json.loads(data)
        except Exception as ex:
            print(ex)
        finally:
            asyncio.get_event_loop().stop()

    loop = asyncio.get_event_loop()
    start_server = websockets.serve(receive_sim_data, 'localhost', port)
    job_server = loop.run_until_complete(start_server)
    send_job(job, SIM_MANAGER_URI)
    loop.run_forever()

    # trigger and wait server shutdown
    job_server.close()
    job_server.wait_closed()
    
    # release used port
    ports_queue.put(port)
    print("port[{0}] released!".format(port))

    #compute fitness value
    fitness_value = fitness(res[port])
    del res[port]
    return fitness_value

def send_job(job, sim_manager_uri):
    job_str = json.dumps(job)
    async def submit_job():
        async with websockets.connect(sim_manager_uri) as websocket:
            await websocket.send(job_str)
        
    asyncio.get_event_loop().run_until_complete(submit_job())

def evaluate(indiv, d=DEF_SIM_DURATION_S, ports_queue=None):
    port = ports_queue.get()
    print("port[{0}] locked!".format(port))
    weights = indiv.tolist()
    job = {
        "duration":d,
        "client_ws":"ws://localhost:{0}".format(port),
        "weights":weights
    }
    return send_job_and_wait(port, job)


if __name__ == '__main__':
    import best_indiv
    from multiprocessing import Queue
    import numpy as np
    queue = Queue()
    queue.put(11360)
    evaluate(np.array(best_indiv.weights), d=10000, ports_queue=queue)