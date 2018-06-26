PD956 Readme
======================
This demo project combines a number of web-based applications. 
The applications are:


* An MQTT client
* A web server which can be used to display sensor readings but also to
  configure MQTT functionality

The sensor starts up and initialize the sensors. The sensor will then connect with the server and trigger a measurement on each sensor.
When each sensor has reported back that they are done, a MQTT message will bee formed and sendt to the server. When the server 
acknowledged the MQTT frame the sensor will sleep. This i a very low power sleep resulting in no communication with the device.
This feature works well with sensors like temperature, movement and botton.
