import base64
from flask import Flask, render_template, Response
import paho.mqtt.client as mqtt
from threading import Thread

app = Flask(__name__)

# MQTT Configuration
BROKER = "mqtt.thingspeak.com"
PORT = 1883
TOPIC = "channels/<channel_id>/subscribe/fields/field1/<mqtt_api_key>"
CLIENT_ID = "flask_mqtt_client"
username = "<mqtt_username>"
password = "<mqtt_password>"

# Variable to store the received image
image_data = None

# MQTT Callback when a message is received
def on_message(client, userdata, message):
    global image_data
    try:
        # Assuming image is sent as base64 text
        image_data = base64.b64decode(message.payload.decode("utf-8"))
        print("Image received and decoded.")
    except Exception as e:
        print(f"Failed to decode image: {e}")

# MQTT Setup
def setup_mqtt():
    client = mqtt.Client(CLIENT_ID)
    client.username_pw_set(username, password)
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.subscribe(TOPIC)
    client.loop_forever()

# Flask Route to Display ImagFine
@app.route('/')
def index():
    if image_data:
        # Convert image to be displayed in HTML
        encoded_image = base64.b64encode(image_data).decode("utf-8")
        return render_template('index.html', img_data=encoded_image)
    else:
        return "No image received yet"

# Start the MQTT Subscriber in a separate thread
mqtt_thread = Thread(target=setup_mqtt)
mqtt_thread.daemon = True
mqtt_thread.start()

if __name__ == '__main__':
    app.run(debug=True)
