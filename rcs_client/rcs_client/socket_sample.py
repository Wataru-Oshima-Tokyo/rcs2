import websocket
import json
import time
def on_open(ws):
    print("WebSocket connection opened.")
    # send a message to the server to request some data
    
    data = {"sender": "WAFFLE2", "timestamp": 1680232703.5191429, "msgtype": "INITIAL_CONNECTION", "destination": "RCS", "map_id": "sh"}
    message = json.dumps({"message": data})
    ws.send(message)

def on_message(ws, message):
    print("Received message:", message)

def on_error(ws, error):
    print("WebSocket error:", error)

def on_close(ws):
    print("WebSocket connection closed.")

# create a WebSocketApp instance and set the callbacks
ws_app = websocket.WebSocketApp("ws://100.92.208.93:5000/ws/dispatch/robot/WAFFLE/",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

# start the WebSocket client
while True:
    ws_app.run_forever()
    time.sleep(5)