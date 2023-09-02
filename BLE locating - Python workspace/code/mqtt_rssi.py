# python 3.x
import json
import random
import time
import numpy as np
import matplotlib.pyplot as plt 
# import字型管理套件
from matplotlib.font_manager import FontProperties
ax = []                     
ay = []  


count = 1

from paho.mqtt import client as mqtt_client
BROKER = 'ec2-18-118-19-223.us-east-2.compute.amazonaws.com'
PORT = 8083
TOPIC = "/C6E67C4F099B/connect_packet/adv_publish"
# generate client ID with pub prefix randomly
CLIENT_ID = "python-mqtt-ws-pub-{id}".format(id=random.randint(0, 1000))
USERNAME = 'admin'
PASSWORD = 'public'
FLAG_CONNECTED = 0
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print("Failed to connect, return code {rc}".format(rc=rc), )
def on_message(client, userdata, msg):
    #Modified
    global count
    count = count+1
    fo = open("foo.txt" , "a")
    print ("Open file", fo.name, "for the ", count-1, " time\n")
    fo.write("\n")
    fo.write("Set index:")
    fo.write(str(count-1))
    fo.write("\n")
    fo.write( msg.payload.decode())
    
    fo.close()

    # modify the txt to have a nice list
    lines = open('foo.txt').readlines()
    fp = open('foo.txt','w')
    for s in lines:
        fp.write( s.replace('},{','\n'))
    fp.close()

#write the line to specific.txt if a line contains certain ID
    f = open('foo.txt','r')
    fi = open('specific.txt', 'w' )
    
    #######################################################################################
    #######################################################################################
    ####################   here to modify the mac of ble you want   #######################
    substring ="E73439667E0D"##############################################################
    #######################################################################################
    #######################################################################################
    #######################################################################################
    stringTime = ""
    stringRSSI = ""

    for line in f.readlines():
        if "time" in line:
            stringTime=line
        if substring in line:
            stringRSSI=line
            fi.write(stringTime)
            fi.write(stringRSSI)

    # only write when the gateway can read RSSI from the mac address
    f.close()
    fi.close()

    #now try to simplify the file
    ky = open('specific.txt', 'r' )
    kyl= open('rssi.txt', 'w' )
    oe=1 #odd =1, even=0, start from line 1

    #############################
    ##### the title of file ##### 
    kyl.write(substring)
    kyl.write('\n')
    #############################

    listTime = []  # they can be used to draw the graph
    listRSSI = []

    for line in ky.readlines():
        if oe==1:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            kyl.write(line[start:end])
            listTime.append(line[start:end])
            kyl.write('\n')
            oe=0
        else:
            start = line.find("-") 
            end = line.find('-') + 3
            kyl.write(line[start:end])
            listRSSI.append(int(line[start:end]))
            kyl.write('\n')
            oe=1
    ky.close()
    kyl.close()


    global ax
    global ay
    plt.ion()                      
    ax.append(listTime[len(listTime)-1])     
    ay.append(listRSSI[len(listTime)-1])        
    plt.clf()         
    plt.plot(ax,ay)        
    plt.pause(0.1)     
    plt.ioff()      





    #print(listTime)
    #Modified
 #  print("Received `{payload}` from `{topic}` topic".format(
   #     payload=msg.payload.decode(), topic=msg.topic))

def connect_mqtt():
    client = mqtt_client.Client(CLIENT_ID, transport='websockets')
    client.username_pw_set(USERNAME, PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT)
    return client


def run():
    client = connect_mqtt()
    client.loop_forever()
if __name__ == '__main__':
    run()