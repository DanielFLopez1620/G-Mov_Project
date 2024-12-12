# Notes about MQTT with Mosquitto/Thingsboard

This file aims to keep constancy of the steps made when setting up Mosquitto/Thingsboard broker in a local network with or without TLS/SSL for the IoT communication. For additional information refer to the links provided.

## Thingsboard MQTT and Dashboard set up:

For this project, it was used the Thingsboard Community Edition for Ubuntu. The instruction installations are provided in the next link: [Installing Thingsboard CE on Ubuntu Server](https://thingsboard.io/docs/user-guide/install/ubuntu/). However, some modifications were made to the installation:

- PostgresSQL was selected as the ThingsBoard database.

- In the steps were the IP 127.0.0.1 was selected (localhost), it was replaced with the IP of the machine in the local network desired for the application. It was also achieved by adding a custom host to the */etc/hosts* file (Check below in the Mosquitto section for more information), for example in the step:

~~~bash
# psql -U postgres -d postgres -h 127.0.0.1 -W
psql -U postgres -d postgres -h <machine_ip> -W
~~~

- In the thingboard file, the *localhost* was replaced with the machine ip or hostname:

~~~conf
export DATABASE_TS_TYPE=sql
# export SPRING_DATASOURCE_URL=jdbc:postgresql://localhost:5432/thingsboard
export SPRING_DATASOURCE_URL=jdbc:postgresql://<machine_ip>:5432/thingsboard
export SPRING_DATASOURCE_USERNAME=postgres
export SPRING_DATASOURCE_PASSWORD=PUT_YOUR_POSTGRESQL_PASSWORD_HERE
export SQL_POSTGRES_TS_KV_PARTITIONING=MONTHS
~~~

- When asking about the queue service, *In Memory* was selected.

- For checking the installation (after waiting 2 minutes for the restartup) use the machine ip, isntaed of localhost, to connect to the Dashboard online. It should also work with other devices over the network, just make sure that no firewall isn't blocking the connection:

~~~link
http://<maiche_ip>:8080/
~~~

## Thinsboard MQTT with TLS

Although it was implemented at this point of the project, you can find more information on:

- [MQTT over TLS | Thingsboard IO](https://thingsboard.io/docs/user-guide/mqtt-over-ssl/)

## MQTT with TLS on local network with IP setup

After some days of experimentation, reading about Mosquitto Broker, exploring Paho Python Library and having existencial crisis with the keys for TLS by using OpenSSL. I want to include here a brief guide on how to set up the everything in order to have your MQTT broker with TLS for experiments in your local network without having to buy a online service for this if you are not sure if the implementation will work.

### Steps for configuring the keys on your PC

The PC in our case will act as our MQTT broker, for this consider the next steps:

1. Install all the required dependencies

~~~bash
# Mosquito dependencies
sudo apt install mosquitto mosquitto-client

# SSL/TLS dependencies


# Python required
pip3 install paho-mqtt
~~~
https://thingsboard.io/docs/user-guide/install/ubuntu/
2. Create a Certificate Authority Key (CA) key

~~~bash
sudo openssl genpkey -algorithm RSA -out /etc/mosquitto/ca_certicates/ca.key
~~~

3. Create a self-signed CA certificate, make sure to add a **CN** name like "LocalCA" or "test" or related:

~~~bash
sudo openssl req -new -x509 -days 365 -key /etc/mosquitto/ca_certificate/ca.key -out /etc/mosquitto/certs/ca.crt
~~~

4. Create folder for server certificates:

~~~bash
sudo mkdir -p /etc/mosquitto/certs
~~~

5. Generate a server key

~~~bash
sudo openssl genpkey -algorithm RSA -out /etc/mosquitto/certs/server.key
~~~

6. Create a certificate signing request (CSR), make sure to add in the **CN** field the IP adress or hostname or FQDN of your server (your PC). Also do not use the same info for the company or city field, as if they are the same, the verification of the keys can be omitted, which can lead to future errors.

~~~bash
sudo openssl req -new -key /etc/mosquitto/certs/server.key -out /etc/mosquitto/certs/server.csr
~~~

7. Sign the server certificate using the CA certificate:

~~~bash
sudo openssl x509 -req -in /etc/mosquitto/certs/server.csr -CA /etc/mosquitto/ca_certificates/ca.crt -CAkey /etc/mosquitto/ca_certificates/ca.key -CAcreateserial -out /etc/mosquitto/certs/server.crt -days 365
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

client.on_connect = on_connectFor Mosquitto issues
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

### Additional configurations for user and password

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

4. In the case you want to test with python, after the mqtt.Client connection, you should set the password and user:

~~~Python
client.username_pw_set("<user>", "<password>")
~~~

### Setting up raspberry pi

1. Make sure you have installed mosquitto on the Raspberry Pi

~~~bash
# Raspberry Terminal
sudo apt install mosquitto mosquitto-clients
~~~

2. You will need to pass the Certificate Authority to the pi, you can use:

~~~bash
# PC terminal
scp /etc/mosquitto/ca_certificates/ca.crt <pi_user>@<pi_ip>:/home/<pi_user>/
~~~


3. Move the **ca.crt** file to the proper location:

~~~bash
# Raspberry terminal
sudo cp ca.crt /etc/mosquitto/ca_certificates/
~~~

4. Test that everything is going OK with **mosquitto_pub** and **mosquitto_sub**

~~~bash
# Terminal 1
mosquitto_sub -h 1<hostname/ip> -t "test/topic" --cafile /etc/mosquitto/certs/ca.crt -p 8883 -u <your_username> -P <your_password>

# Terminal 2
mosquitto_pub -h <hostname/ip> -t "test/topic" -m "Hello MQTT over TLS" --cafile /etc/mosquitto/certs/ca.crt -p 8883 -u <your_username> -P <your_password>

~~~

5. You can now test with Paho in Python, by running the file [localhost_usps_tls.py](/g_mov_local_mqtts_notes/localhost_usps_tls.py)

But you should still need to use the **tls_insecure_set**. That is why we need to configure a hostname that will be covered in the next title.

## MQTT with TLS on local network with hostname setup

### MQTT set up for PC with hostname

1. In your local PC add a new host with the IP of your broker, in this case, your own IP adress.

~~~bash
sudo nano /etc/hosts
~~~

~~~bash
# By default, systemd-resolved or libnss-myhostname will resolve
# localhost and the system hostname if they're not specified here.
127.0.0.1       localhost
::1             localhost

# Add the next host for example, replace x with your IP
192.168.x.x   mqtt.local
~~~

2. Install avahi to resolve the hostname using mDNS:

~~~bash
sudo apt update
sudo apt install avahi-daemon
~~~

3. Enable and start the service

~~~bash
sudo systemctl start avahi-daemon
sudo systemctl enable avahi-daemon
~~~

4. Make sure that the hostname is connected 

~~~bash
ping mqtt.local
~~~

5. After this, you will need to generate new keys, you can generate them with a new name so you can use the previous one, in case you want to have the last configuration as a backup. Follow the next steps to configure the keys again.

6. Create a Certificate Authority Key (CA) key

~~~
sudo openssl genpkey -algorithm RSA -out ca_certificates/ca_host.key
~~~

7. Create a self-signed CA certificate, make sure to add a **CN** name like "LocalCA" or "test" or related:

~~~bash
sudo openssl req -new -x509 -days 365 -key ca_certificates/ca_host.key -out ca_certificates/ca_host.crt
~~~

8. Generate a server key

~~~bash
sudo openssl genpkey -algorithm RSA -out /etc/mosquitto/certs/server_host.key
~~~

9. Create a certificate signing request (CSR), make sure to add in the **CN** field the IP adress or hostname or FQDN of your server (your PC). Also do not use the same info for the company or city field, as if they are the same, the verification of the keys can be omitted, which can lead to future errors.

~~~bash
sudo openssl req -new -key /etc/mosquitto/certs/server_host.key -out /etc/mosquitto/certs/server_host.csr
~~~

10. Sign the server certificate using the CA certificate:

~~~bash
sudo openssl x509 -req -in /etc/mosquitto/certs/server_host.csr -CA /etc/mosquitto/ca_certificates/ca_host.crt -CAkey /etc/mosquitto/ca_certificates/ca_host.key -CAcreateserial -out /etc/mosquitto/certs/server_host.crt -days 365
~~~

11. Edit the mosquitto configuration

~~~bash
sudo nano /etc/mosquitto/mosquitto.conf
~~~

~~~bash
# A full description of the configuration file is at
# /usr/share/doc/mosquitto/examples/mosquitto.conf.example

pid_file /run/mosquitto/mosquitto.pid

log_type all
log_dest file /var/log/mosquitto/mosquitto.log

listener 8883 0.0.0.0
protocol mqtt

cafile /etc/mosquitto/ca_certificates/ca_host.crt
keyfile /etc/mosquitto/certs/server_host.key
certfile /etc/mosquitto/certs/server_host.crt 

tls_version tlsv1.2

allow_anonymous false
require_certificate false

password_file /etc/mosquitto/passwd
~~~

12. Restart mosquito services

~~~
sudo systemctl restart mosquitto
~~~

### MQTT set up for raspberry with hostname

1. In your local PC add a new host with the IP of your broker, in this case, your own IP adress.

~~~bash
sudo nano /etc/hosts
~~~

~~~bash
# By default, systemd-resolved or libnss-myhostname will resolve
# localhost and the system hostname if they're not specified here.
127.0.0.1       localhost
::1             localhost

# Add the next host for example, replace x with your IP
192.168.x.x   mqtt.local
~~~

2. Install avahi to resolve the hostname using DNS:

~~~bash
sudo apt update
sudo apt install avahi-daemon
~~~

3. Enable and start the service

~~~bash
sudo systemctl start avahi-daemon
sudo systemctl enable avahi-daemon
~~~

4. Make sure that the hostname is connected 

~~~bash
ping mqtt.local
~~~

## Troubleshooting:

- Make sure your host IP is unique, for example, if you have others IP set up like for NGrok, ZeroTier, Docker. It is better to disable them to prevent issues. For example, you can use in the case of Docker:

~~~
sudo ip link set docker<id> down
~~~

- If you are not sure about the IP or hostname you set up, it is better to add new keys.

- Remember, do not use the same city, location, company and unit info for CA and server keys, as it will lead to ignore a validation of the keys. Of just one character change, it will be enough

- Keep in mind that Thingsboard service may take 2 minutes to restart if you made some changes to the Thignsboard config or yaml file, this also applies for start up. In case, the startup doesn't apply the proper IP configuration, you should restart postgres and thingsboard services:

~~~bash
sudo systemctl restart postgres # It can also be postgres.service
sudo systemctl restart thingsboard.service
~~~

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