"""
Author : Vaibhav
Description : Servo will publish data on "path_tx" topic and subscribed to "feedback" topic.
"""


import paho.mqtt.client as AgvClient
import time


IP = "192.168.43.241"
name = "Server_tx"
port = 1883
Publish_Topic = "path_tx"
Subscribe_Topic = "feedback"
path = str(input("Enter Path: "))
Connected = False

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection
    else:
        print("Connection failed",rc)

def on_message(client, userdata, message):
	msg = str(message.payload)
        #print ("Current Cordinates" + "(" + msg[0: )
	print("X Coordinate: {}, Y Coordinate: {}, Orientation: {}".format(msg[2],msg[3],msg[4]))


AGV = AgvClient.Client(name)		#create a client name
AGV.on_connect= on_connect                      #attach function to callback
AGV.on_message= on_message                      #attach function to callback
AGV.connect(IP,port)			#connect to broker
AGV.loop_start()

while Connected != True:
	time.sleep(0.1)

AGV.subscribe(Subscribe_Topic)
print("Subscribed...")
AGV.publish(Publish_Topic,path)
time.sleep(2)


try:
	while True:
		time.sleep(1)
except KeyboardInterrupt:
	print("Exiting")
	AGV.disconnect()
	AGV.loop_stop()







