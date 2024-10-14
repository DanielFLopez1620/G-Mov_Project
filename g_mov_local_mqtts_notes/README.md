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