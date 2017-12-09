# Feather-Wifi-AP
SoftAP webserver and Mqtt Client solution using Adafruit Feather M0 Wifi Module 

# Maintain Connection to Broker
Manage the connection to the broker at all times and automatically attempt to reconnect to a Broker if the
Feather becomes disconnected for any reason.

# Embedded Web Server
- Create an embedded web interface for the Feather. 
    The OLED will be used to display the IP address even
    if it is 169.x.x.x so someone can launch the web page using the IP.
- The web page should scan and list the available Wi-Fi networks (I saw some sample code for this also).
- Web page should allow the user to select an available SSID. 
    It should have text fields to allow the user to
    type in connection info if required for the network they want to connect to. The web page should allow
    user to select connect to the network using DHCP and should provide the option to manual type in static
    IP address, subnet, and gateway.
- Web page should provide a textbox to type in the URL or IP of the MQTT Broker. There should be a
connect button available to connect to the Broker. If a connection to the Broker is successful there
should be a message that says it was successful on the web page.
- Web page should have a section to set the current date and time.
- All settings should be saved locally to the Feather. Once the Feather successfully connects to the Broker,
all settings made on the Feather should also be published to the Broker via MQTT message so it can be
stored there as well.
# Power management
Manage the power consumption of the Feather as discussed here. The idea here is to preserve battery life when
the Feather is running off the battery.
https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/power-management
