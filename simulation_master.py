import asyncio
import websockets
import multiprocessing


def ws_server(port):
    res = None
    async def receive(websocket, path):
        data = await websocket.recv()
        res = data
        asyncio.get_event_loop().stop()
        websocket.close()

    start_server = websockets.serve(receive, 'localhost', port)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
    return res

"""
async def hello():
    async with websockets.connect(
            'ws://localhost:8765') as websocket:
        name = input("What's your name? ")

        await websocket.send(name)
        print(f"> {name}")

        greeting = await websocket.recv()
        print(f"< {greeting}")

asyncio.get_event_loop().run_until_complete(hello())
"""

if __name__ == '__main__':
    # res = ws_server((8767,))
    # print(res)

    ports = (8765, 8766, 8767, 8768)
    procs = [multiprocessing.Process(target=ws_server, args=(p,)) for p in ports]
    
    for p in procs:
        p.start()

    for p in procs:
        p.join()
