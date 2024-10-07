import paho.mqtt.client as mqtt

# ThingSpeak Credentials
mqtt_channel_id = "2667301"
mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
mqtt_topic = "channels/2667301/subscribe/fields/field4/4M5ALN2ZG22M8C0L"
mqtt_port = 1883
mqtt_host = "mqtt3.thingspeak.com"

# Callback function when the client receives a message
def on_message(client, userdata, message):
    print(f"Received message on {message.topic}: {message.payload.decode('utf-8')}")

# Initialize MQTT client and set up authentication
client = mqtt.Client(client_id=mqtt_client_ID, protocol=mqtt.MQTTv311)
client.username_pw_set(mqtt_username, mqtt_password)
client.on_message = on_message

# Connect to the MQTT broker
client.connect(mqtt_host, mqtt_port, 60)

# Subscribe to the correct topic
client.subscribe(mqtt_topic)

# Start the loop
client.loop_forever()
