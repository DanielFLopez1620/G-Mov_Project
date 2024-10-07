import paho.mqtt.client as mqtt
import json
from flask import Flask, render_template, jsonify
from flask_cors import CORS
import datetime

app = Flask(__name__)
CORS(app)

# Store sensor data for graphing
sensor_data = []

# MQTT settings
BROKER = 'mqtt.thingspeak.com'
PORT = 1883
TOPIC = 'channels/CHANNEL_ID/subscribe/fields/field1/YOUR_MQTT_API_KEY'

# Callback when connecting to the broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)

# Callback when receiving a message
def on_message(client, userdata, msg):
    global sensor_data
    payload = msg.payload.decode()
    data = json.loads(payload)

    # Filter data for sensorID = 2 and variableID = 1
    if data['sensorID'] == 2 and data['variableID'] == 1:
        sensor_data.append({
            'timestamp': data['timestamp'],
            'sensorValue': data['sensorValue']
        })
        # Limit the data stored
        if len(sensor_data) > 100:
            sensor_data = sensor_data[-100:]

# MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)

# Run the MQTT client in the background
client.loop_start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/sensor_data')
def get_sensor_data():
    # Convert timestamps to readable format
    formatted_data = [
        {
            'timestamp': datetime.datetime.fromisoformat(entry['timestamp']).strftime('%Y-%m-%d %H:%M:%S'),
            'sensorValue': entry['sensorValue']
        }
        for entry in sensor_data
    ]
    return jsonify(formatted_data)

if __name__ == '__main__':
    app.run(debug=True)
