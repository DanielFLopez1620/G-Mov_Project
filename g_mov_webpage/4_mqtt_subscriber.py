import paho.mqtt.client as mqtt

# Define your connection parameters
CHANNEL_ID = "2667301"  # Your Channel ID
CLIENT_ID = "ER0GJDQkIQsBNR0hGjcqIyw"  # Your Client ID
USERNAME = "ER0GJDQkIQsBNR0hGjcqIyw"  # Username (same as client ID)
PASSWORD = "Fu4VIaTN2ZtaoqTuMUnGYv5D"  # Your MQTT password (use your write API key if needed)
mqtt_read_key = "4M5ALN2ZG22M8C0L"
TOPIC = f"channels/{CHANNEL_ID}/subscribe/fields/+"  # Topic to subscribe (field1)
HOST = "mqtt3.thingspeak.com"
PORT = 1883  # Non-secure port (or use 8883 for SSL)

# Callback when the client receives a message
def on_message(client, userdata, message):
    print(f"Received message: {message.payload.decode()} from topic: {message.topic}")

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker successfully!")
        # Subscribe to the topic
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected with return code {rc}")


# Create MQTT client instance
client = mqtt.Client(client_id=CLIENT_ID)

# Set username and password for authentication
client.username_pw_set(username=USERNAME, password=PASSWORD)

# Assign callback functions
client.on_message = on_message
client.on_connect = on_connect
client.on_disconnect = on_disconnect

# Connect to the ThingSpeak MQTT broker
client.connect(HOST, PORT, keepalive=60)

# Start the loop to process network traffic and dispatch callbacks
client.loop_forever()
