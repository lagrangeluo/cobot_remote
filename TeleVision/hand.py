from vuer import Vuer, VuerSession
from vuer.schemas import Hands
from asyncio import sleep

app = Vuer(host='0.0.0.0',port = 8012,cert="./cert.pem", key="./key.pem")

@app.add_handler("HAND_MOVE")
async def handler(event, session):
    print(f"Movement Event: key-{event.key}", event.value)

@app.spawn(start=True)
async def main(session: VuerSession):
    # Important: You need to set the `stream` option to `True` to start
    # streaming the hand movement.
    session.upsert @ Hands(fps=30, stream=True, key="hands")

    while True:
        print("111")
        await sleep(1)