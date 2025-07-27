"""
Truss Web Server
 - receives sensor data from Truss and emits updates to connected clients via WebSockets.
"""

from flask import Flask, request, jsonify
from flask_socketio import SocketIO, send

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

sensor_data = {
    'temperature': None,
    'humidity': None,
    'pH': None
}

@app.route('/')
def root():
    return "This is the root endpoint of the Truss' web server."

@app.route('/temperature', methods=['POST'])
def temperature():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'temperature' in data:
            sensor_data['temperature'] = data['temperature']
            socketio.emit('temperature sensor data update', {'temperature': sensor_data['temperature']})
            return jsonify({"message": f"Received temperature data: {sensor_data['temperature']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide temperature data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@app.route('/humidity', methods=['POST'])
def humidity():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'humidity' in data:
            sensor_data['humidity'] = data['humidity']
            socketio.emit('humidity sensor data update', {'humidity': sensor_data['humidity']})
            return jsonify({"message": f"Received humidity data: {sensor_data['humidity']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide humidity data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@app.route('/pH', methods=['POST'])
def pH():
    if request.method == 'POST':
        data = request.json

        if data and type(data) is dict and 'pH' in data:
            sensor_data['pH'] = data['pH']
            socketio.emit('pH sensor data update', {'pH': sensor_data['pH']})
            return jsonify({"message": f"Received pH data: {sensor_data['pH']}"}), 200
        else:
            return jsonify({"error": "Invalid data format. Please provide pH data in JSON format."}), 400

    return jsonify({"message": "Method not allowed. Please use POST."}), 405

@socketio.on('connect')
def webhook_stolon():
    print("Client connected")
    send({'message': 'Welcome! You are now connected to the Truss Web Server.'})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=54321, debug=True)
