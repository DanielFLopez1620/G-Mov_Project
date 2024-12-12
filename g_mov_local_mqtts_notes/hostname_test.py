import paho.mqtt.client as mqtt
import ssl

# Define broker and port
broker = "mqtt.local"  # Hostname for your PC
port = 8883  # TLS/SSL port

# Define topic
topic = "test/topic"

# MQTT client ID
client_id = "mqtt_pub_client"

# Path to CA certificate on Raspberry Pi
ca_cert = "/etc/mosquitto/ca_certificates/ca_host.crt"

# Callback function on connect
def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connected to broker successfully")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback function on publish
def on_publish(client, userdata, mid, rc, properties):
    print(f"Message published with mid: {mid}")

# Create a new MQTT client instance with callback API version
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)

# Set TLS settings
client.tls_set(ca_certs=ca_cert,
               tls_version=ssl.PROTOCOL_TLSv1_2)

client.username_pw_set("Dan1620", "h1d4n16")
# client.tls_insecure_set(True)

# Set callbacks
client.on_connect = on_connect
client.on_publish = on_publish

# Connect to the broker
client.connect(broker, port)

# Start the network loop
client.loop_start()

# Publish a message to the topic
message = "Hello, MQTT with TLS!"
result = client.publish(topic, message)

# Wait for the message to be sent
result.wait_for_publish()

# Stop the network loop and disconnect
client.loop_stop()
client.disconnect()

print("Disconnected from broker")