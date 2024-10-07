import paho.mqtt.client as mqtt
import base64

# MQTT settings
mqtt_channel_id = "2667301"
mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
mqtt_read_key = "4M5ALN2ZG22M8C0L"
mqtt_host = "mqtt3.thingspeak.com"
mqtt_port = 1883

# Topic including the Read API Key for field4
mqtt_topic = f"channels/{mqtt_channel_id}/subscribe/fields/field4/{mqtt_read_key}"

# Callback when a message is received
def on_message(client, userdata, message):
    payload = message.payload.decode("utf-8")
    if payload.startswith('/9j/'):
        print("Received a base64 encoded JPEG image.")
        print(f"Message payload: {payload[:100]}...")  # Display a portion of the payload
    else:
        print(f"Received non-image data: {payload}")
# Callback when connected to the broker
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected successfully to MQTT broker!")
        client.subscribe(mqtt_topic, 2)
        print(f"Subscribed to topic: {mqtt_topic}")
    else:
        print(f"Connection failed with code {reason_code}")

# MQTT Client setup
client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=mqtt_client_ID, protocol=mqtt.MQTTv31)
client.username_pw_set(mqtt_username, mqtt_password)
client.on_message = on_message
client.on_connect = on_connect


def on_log(client, userdata, level, buf):
    print(f"Log: {buf}")

# client.on_log = on_log



# Connect to the MQTT broker
client.connect(mqtt_host, mqtt_port, 60)

# Loop to keep the client connected and listening for messages
client.loop_forever()
