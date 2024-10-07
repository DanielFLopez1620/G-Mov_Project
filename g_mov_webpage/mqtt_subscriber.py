import paho.mqtt.subscribe as subscribe

channel_id = "2667301"
mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
mqtt_topic = "channels/" + channel_id + "/subscribe/fields/"
mqtt_host = "mqtt3.thingspeak.com"

def callback(client, userdate, message):
    topic = message.topic
    payload = message.payload.decode('utf-8')
    print(f"Received message on topic {topic}: {payload}")

subscribe.callback(callback=callback, topics=mqtt_topic + "field1",
                hostname= mqtt_host,
                transport= "tcp",
                port= 1883,
                client_id= mqtt_client_ID,
                auth = {
                        'username':mqtt_username,
                        'password': mqtt_password
                        }
                )