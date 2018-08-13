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

SIM_MANAGER_URI = "ws://127.0.0.1:9090/simulation-manager"
res = {}

def fitness(sim_result):
    try:
        x,y = sim_result["result"]
        return (x,)
    except:
        return (float('-inf'),)

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
    #print("{0}:fitness={1}".format(port, fitness_value))
    del res[port]
    return fitness_value

def send_job(job, sim_manager_uri):
    job_str = json.dumps(job)
    async def submit_job():
        async with websockets.connect(sim_manager_uri) as websocket:
            await websocket.send(job_str)
        
    asyncio.get_event_loop().run_until_complete(submit_job())

def evaluate(indiv, ports_queue=None):
    port = ports_queue.get()
    weights = indiv.tolist()
    job = {
        "duration":10,
        "client_ws":"ws://localhost:{0}".format(port),
        "weights":weights
    }
    return send_job_and_wait(port, job)

if __name__ == '__main__':
    import best_elem
    from multiprocessing import Queue
    import numpy as np
    queue = Queue()
    queue.put(11360)
    evaluate(np.array(best_elem.weights), queue)
