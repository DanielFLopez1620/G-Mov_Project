from flask import Flask, render_template, redirect, url_for
from flask_cors import CORS

import base64
import paho.mqtt.client as mqtt
from threading import Thread
import json

app = Flask(__name__)
CORS(app)

mqtt_channel_id = "2667301"
mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
mqtt_topic = "channels/" + mqtt_channel_id + "/subscribe/fields/field4"
mqtt_port = 1883
mqtt_host = "mqtt3.thingspeak.com"

image_data = None

# ------------------------- PYTHON FUNCTIONS ---------------------------------
def on_message(client, userdata, message):
    global image_data
    try:
        print("Image_received... Processing...")
        # Check if the payload starts with the base64 encoding of a JPEG image ('/9j/')
        payload = message.payload.decode("utf-8")
        if payload.startswith("/9j/"):
            image_data = base64.b64decode(payload)
            print("JPEG image received and decoded.")
        else:
            print("Payload is not a JPEG image. Ignoring message.")
    except Exception as e:
        print(f"Failed to process image: {e}")

def setup_mqtt():
    client = mqtt.Client(client_id=mqtt_client_ID)
    client.username_pw_set(mqtt_username, mqtt_password)
    client.on_message = on_message
    client.connect(mqtt_host, mqtt_port, 60)
    client.subscribe(mqtt_topic)
    client.loop_forever()

# Thread configuration
mqtt_thread = Thread(target=setup_mqtt)
mqtt_thread.daemon = True
mqtt_thread.start()

# ------------------------ FLASK ROUTE APPLICATIONS ---------------------------
@app.route('/')
def index():
    return redirect(url_for('dashboard_cam_local'))

@app.route('/dashboard_cam_local')
def dashboard_cam_local():
    return render_template('dashboard_cam_local.html')

@app.route('/dashboard_cam_mqtt')
def dashboard_cam_mqtt():
    if image_data:
        # Convert image to be displayed in HTML
        encoded_image = base64.b64encode(image_data).decode("utf-8")
        return render_template('dashboard_cam_mqtt.html', img_data=encoded_image)
        # return render_template('dashboard_cam_mqtt.html')
    else:
        return "No image received yet"
    


if __name__ == '__main__':
    app.run(debug=True)