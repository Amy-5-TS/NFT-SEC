




# python 3.x
import json
import random
import time
import matplotlib.pyplot as plt 
import numpy

ax = []                     
ay = []  
useA=[]
useB=[]
rat=0
count = 1

from paho.mqtt import client as mqtt_client
BROKER = 'ec2-18-118-19-223.us-east-2.compute.amazonaws.com'
PORT = 8083
TOPIC1 = "/C6E67C4F099B/connect_packet/adv_publish"
TOPIC2 = "/D892A794427E/connect_packet/adv_publish"
TOPIC3 = "/ECA55EBEEF1A/connect_packet/adv_publish"
# generate client ID with pub prefix randomly
CLIENT_ID = "python-mqtt-ws-pub-{id}".format(id=random.randint(0, 1000))
USERNAME = 'admin'
PASSWORD = 'public'
FLAG_CONNECTED = 0



def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC1)
        client.subscribe(TOPIC2)
        client.subscribe(TOPIC3)
    else:
        print("Failed to connect, return code {rc}".format(rc=rc), )
def on_message(client, userdata, msg):
    #Modified
    global count
    count = count+1
    fo = open("foo.txt" , "a")
    print ("gateway returned data ",  count-1, "\n")
    fo.write("\n")
    fo.write("Set index:")
    fo.write(str(count-1))
    fo.write("\n")
    fo.write( msg.payload.decode() )
    
    fo.close()

    ##########---###########`````````` here update ID ``````````###########---##########
    ID1="C6E67C4F099B"
    ID2="D892A794427E"
    ID3="ECA55EBEEF1A"
    ##########---###########`````````` here update ID ``````````###########---##########
    # data will be dumped after this iteration
    lines = open('foo.txt').readlines()
    fp = open('foo.txt','w')
    for s in lines:
        fp.write( s.replace('},{','\n'))
    if ID1 in lines:
        fp.write( s.replace(ID1,"point1"))   # which to which? marked out
    elif ID2 in lines:
        fp.write( s.replace(ID2,"point2"))
    elif ID3 in lines:
        fp.write( s.replace(ID3,"point3"))
    fp.close()


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