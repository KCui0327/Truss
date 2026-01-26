"""
Truss Web Server
 - receives sensor data from Truss and emits updates to connected clients via WebSockets.
"""

from flask import Flask, request, jsonify
import requests
from flask_socketio import SocketIO, send
import os
import signal


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

THRESHOLD = 0.10  
registered_tokens = set()
sensor_data = {
    'temperature': None,
    'humidity': None,
    'light': None
}

# Determines if the change between two sensor measurements exceeds threshold
def delta_exceed_threshold(oldVal, newVal):
  if not oldVal:
      return True
  if oldVal == 0:
    return newVal == 0

  delta = abs(newVal - oldVal)
  limit = abs(oldVal) * THRESHOLD
  return delta <= limit


@app.route('/')
def root():
    return "This is the root endpoint of the Truss' web server."

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    if request.method == 'POST':
        if os.getenv('CONTROL_PROC_PID') is None:
            return jsonify({"error": "The PID for control process was not found on server end."}), 400
        
        try:
            control_proc_pid = int(os.getenv('CONTROL_PROC_PID'))
            os.kill(control_proc_pid, signal.SIGTERM)  # Send SIGTERM to the control process
        except ValueError:
            return jsonify({"error": "Invalid control process PID."}), 400
        except ProcessLookupError:
            return jsonify({"error": "Control process not found."}), 404
        
        return jsonify({"message": "Emergency stop signal processed."}), 200

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@app.route('/register_device', methods=['POST'])
def register_device():
    data = request.json
    if data and 'token' in data:
        token = data['token']
        registered_tokens.add(token)
        print(f"Device registered: {token}")
        return jsonify({"message": "Token registered successfully"}), 200
    return jsonify({"error": "Invalid token data"}), 400

def send_push_notification(title, message):
    url = "https://exp.host/--/api/v2/push/send"
    for token in registered_tokens:
        payload = {
            "to": token,
            "title": title,
            "body": message,
            "sound": "default",
        }
        try:
            requests.post(url, json=payload)
        except Exception as e:
            print(f"Failed to send notification: {e}")
            
@app.route('/temperature', methods=['POST'])
def temperature():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'temperature' in data:
            if not (-50 <= data['temperature'] <= 100):
                return jsonify({"error": "Invalid temperature value. It must be between -50 and 100."}), 400
            
            newValue = data['temperature']
            oldValue = sensor_data['temperature']

            sensor_data['temperature'] = newValue
            socketio.emit('temperature sensor data update', {'temperature': sensor_data['temperature']})
            if delta_exceed_threshold(oldValue, newValue) or (oldValue >= 15 and oldValue <= 18):
                if (newValue > 18):
                    send_push_notification(f"High Temperature Alert", f"Current temperature is {newValue}° Celsius")
                elif (newValue < 15):
                    send_push_notification(f"Low Temperature Alert", f"Current temperature is {newValue}° Celsius")

            return jsonify({"message": f"Received temperature data: {sensor_data['temperature']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide temperature data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@app.route('/humidity', methods=['POST'])
def humidity():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'humidity' in data:
            newValue = data['humidity']
            oldValue = sensor_data['humidity']

            sensor_data['humidity'] = newValue
            socketio.emit('humidity sensor data update', {'humidity': sensor_data['humidity']})

            if delta_exceed_threshold(oldValue, newValue) or (oldValue >= 65 and oldValue <= 75):
                if (newValue > 75):
                    send_push_notification(f"High Humidity Alert", f"Current humidity is {newValue}%")
                elif (newValue < 65):
                    send_push_notification(f"Low Humidity Alert", f"Current humidity is {newValue}%")
            return jsonify({"message": f"Received humidity data: {sensor_data['humidity']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide humidity data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@app.route('/light', methods=['POST'])
def light():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'light' in data:
            if not (0 <= data['light'] <= 14):
                return jsonify({"error": "Invalid light value. It must be between 0 and 14."}), 400
            
            sensor_data['light'] = data['light']
            socketio.emit('light sensor data update', {'light': sensor_data['light']})
            return jsonify({"message": f"Received light data: {sensor_data['light']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide light data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@socketio.on('connect')
def webhook_stolon():
    print("Client connected")
    send({'message': 'Welcome! You are now connected to the Truss Web Server.'})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=12345, debug=False)
