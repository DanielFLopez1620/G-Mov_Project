# MQTT with TLS on local network and localhost setup

After some days of experimentation, reading about Mosquitto Broker, exploring Paho Python Library and having existencial crisis with the keys for TLS by using OpenSSL. I want to include here a brief guide on how to set up the everything in order to have your MQTT broker with TLS for experiments in your local network without having to buy a online service for this if you are not sure if the implementation will work.

## Steps for configuring the keys on your PC

The PC in our case will act as our MQTT broker, for this consider the next steps:

1. Install all the required dependencies

~~~bash
# Mosquito dependencies
sudo apt install mosquitto mosquitto-client

# SSL/TLS dependencies


# Python required
pip3 install paho-mqtt
~~~

2. Create a Certificate Authority Key (CA) key

~~~bash
openssl genpkey 
-algorithm RSA -out /etc/mosquitto/ca_certicates/ca.key
~~~

3. Create a self-signed CA certificate, make sure to add a **CN** name like "LocalCA" or "test" or related:

~~~bash
openssl req -new -x509 -days 365 -key /etc/mosquitto/ca_certificate/ca.key -out /etc/mosquitto/certs/ca.crt
~~~

4. Create folder for server certificates:

~~~bash
sudo mkdir -p /etc/mosquitto/certs
~~~

5. Generate a server key

~~~bash
openssl genpkey -algorithm RSA -out /etc/mosquitto/certs/server.key
~~~

6. Create a certificate signing request (CSR), make sure to add in the **CN** field the IP adress or hostname or FQDN of your server (your PC). Also do not use the same info for the company or city field, as if they are the same, the verification of the keys can be omitted, which can lead to future errors.

~~~bash
openssl req -new -key /etc/mosquitto/certs/server.key -out /etc/mosquitto/certs/server.csr
~~~

7. Sign the server certificate using the CA certificate:

~~~bash
openssl x509 -req -in /etc/mosquitto/certs/server.csr -CA /etc/mosquitto/ca_certificates/ca.crt -CAkey /etc/mosquitto/certs/ca.key -CAcreateserial -out /etc/mosquitto/certs/server.crt -days 365
~~~

8. Configure permissions and ownership

~~~bash
sudo chmod 600 /etc/mosquitto/certs/server.key
sudo chmod 644 /etc/mosquitto/certs/server.crt
sudo chmod 644 /etc/mosquitto/certs/ca.crt

sudo chown mosquitto:mosquitto /etc/mosquitto/certs/server.crt

sudo chown mosquitto:mosquitto /etc/mosquitto/certs/server.key

sudo chown mosquitto:mosquitto /etc/mosquitto/ca_certificates/ca.crt
~~~

9. Update mosquitto configuration, open with your editor (by having sudo access) the mosquitto.conf file:

~~~bash
sudo nano /etc/mosquitto/mosquitto.conf
~~~

Then edit it to consider the next:

~~~bash
pid_file /run/mosquitto/mosquitto.pid

# persistence true
# persistence_location /var/lib/mosquitto/

log_type all
log_dest file /var/log/mosquitto/mosquitto.log

# include_dir /etc/mosquitto/conf.d

listener 8883
protocol mqtt

cafile /etc/mosquitto/ca_certificates/ca.crt
keyfile /etc/mosquitto/certs/server.key
certfile /etc/mosquitto/certs/server.crt

tls_version tlsv1.2

# Allow the next only for test, it is recommended to set user and password)
allow_anonymous true
~~~

10. Kill previous instances of mosquitto to prevent issues

~~~bash
sudo systemctl stop mosquitto
sudo killall mosquitto
~~~

11. Restart mosquitto

~~~bash
sudo systemctl start mosquitto
~~~

12. Test that everything is going right, by publishing and subscribing on different terminals

~~~bash
# Terminal 1
mosquitto_sub -h <hostname/ip> -t "test/topic" --cafile /etc/mosquitto/ca_certificates/ca.crt -p 8883
# Terminal 2
mosquitto_pub -h <hostname/ip> -t "test/topic" -m "Hello World" --cafile /etc/mosquitto/ca_certificates/ca.crt -p 8883
~~~

13. You can now run a Python code with Paho to test the publishing and subscribing, in this case if you are running from local, you should set insecure tls to avoid certain validations of paho that may block the connection.

~~~python
import paho.mqtt.client as mqtt
import ssl

broker = "<broker/hostname/ip>"  # Your PC's IP address (broker)
port = 8883  # Port for TLS/SSL

# Define topic
topic = "test/topic"

client_id = "mqtt_pub_client"

ca_cert = "/etc/mosquitto/ca_certificates/ca.crt"  # CA certificate (on Raspberry Pi)

def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connected to broker successfully")
    else:
        print(f"Failed to connect, return code {rc}")

def on_publish(client, userdata, mid, rc, properties):
    print(f"Message published with mid: {mid}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)

client.tls_set(ca_certs=ca_cert,
               tls_version=ssl.PROTOCOL_TLSv1_2)
client.tls_insecure_set(True)

client.on_connect = on_connect
client.on_publish = on_publish

client.connect(broker, port)

client.loop_start()

message = "Hello, MQTT with TLS!"
result = client.publish(topic, message)

result.wait_for_publish()

client.loop_stop()
client.disconnect()

print("Disconnected from broker")
~~~

## Additional configurations for user and password

To prevent anonymous connections, you can set up a user and a password, for this you can follow the next steps:

1. Set up the user and password within the proper file with the command below, make sure to remember the password linked to the username

~~~bash
sudo mosquitto_passwd  -c /etc/mosquitto/passwd <username>
~~~

2. Modify your mosquitto.conf file in order to block anonymous connections and ask for user/password by checking a given file:

~~~bash
sudo nano /etc/mosquitto/mosquitto.conf
~~~

~~~bash
allow_anonymous false
password_file /etc/mosquitto/passwd
~~~

3. Test with a publication and a subscription by passing the proper username and password, along with the mosquitto_pub and mosquitto_sub command

~~~bash
# Terminal 1
mosquitto_sub -h 1<hostname/ip> -t "test/topic" --cafile /etc/mosquitto/certs/ca.crt -p 8883 -u <your_username> -P <your_password>

# Terminal 2
mosquitto_pub -h <hostname/ip> -t "test/topic" -m "Hello MQTT over TLS" --cafile /etc/mosquitto/certs/ca.crt -p 8883 -u <your_username> -P <your_password>

~~~



## Troubleshooting:

- Make sure your host IP is unique, for example, if you have others IP set up like for NGrok, ZeroTier, Docker. It is better to disable them to prevent issues. For example, you can use in the case of Docker:

~~~
sudo ip link set docker<id> down
~~~

- If you are not sure about the IP or hostname you set up, it is better to add new keys.

- Remember, do not use the same city, location, company and unit info for CA and server keys, as it will lead to ignore a validation of the keys. Of just one character change, it will be enough

## Sources and References

- [Mosquitto MQTT Broker SSL Configuration Using Own Certificates | Steves Internet Guide](http://www.steves-internet-guide.com/mosquitto-tls/)

- [Creating and Using Client Certificates with MQTT and Mosquitto | Steves Internet Guide](http://www.steves-internet-guide.com/creating-and-using-client-certificates-with-mqtt-and-mosquitto/)

- [Mosquitto MQTT Broker SSL Configuration Using Own Certificates | Steves Internet Guide](http://www.steves-internet-guide.com/mosquitto-tls/)

- [paho-mqtt 2.1.0 | Pypi.org](https://pypi.org/project/paho-mqtt/#tls-set)

- [Paho Mqtt Python | Github by eclipse](https://github.com/eclipse/paho.mqtt.python)

- [Mosquitto.conf Man Page | Eclipse foundation & Mosquitt.org](https://mosquitto.org/man/mosquitto-conf-5.html)

- [Mosquitto Simple Config Example | Github by timWaizenegger](https://mosquitto.org/man/mosquitto-conf-5.html)

- [Migration Guide for Paho](https://eclipse.dev/paho/files/paho.mqtt.python/html/migrations.html)

- [Mosquitto_sub output format options | AsciiCinema by Raylight](https://asciinema.org/a/82233)