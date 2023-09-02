# python 3.x
import json
import random
import time
import matplotlib.pyplot as plt 
import math
ax = []                     
ay = []  
RLx = []  
RLy = []  

rat = 0
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


######################################class Kalman filter######################################
class KalmanFilter:

    cov = float('nan')
    x = float('nan')

    def __init__(self, R, Q):
        """
        Constructor
        :param R: Process Noise
        :param Q: Measurement Noise
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.R = R
        self.Q = Q

    def filter(self, measurement):
        """
        Filters a measurement
        :param measurement: The measurement value to be filtered
        :return: The filtered value
        """
        u = 0
        if math.isnan(self.x):
            self.x = (1 / self.C) * measurement
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        else:
            predX = (self.A * self.x) + (self.B * u)
            predCov = ((self.A * self.cov) * self.A) + self.R

            # Kalman Gain
            K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q))

            # Correction
            self.x = predX + K * (measurement - (self.C * predX))
            self.cov = predCov - (K * self.C * predCov)

        return self.x

    def last_measurement(self):
        """
        Returns the last measurement fed into the filter
        :return: The last measurement fed into the filter
        """
        return self.x

    def set_measurement_noise(self, noise):
        """
        Sets measurement noise
        :param noise: The new measurement noise
        """
        self.Q = noise

    def set_process_noise(self, noise):
        """
        Sets process noise
        :param noise: The new process noise
        """
        self.R = noise




def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print("Failed to connect, return code {rc}".format(rc=rc), )
def on_message(client, userdata, msg):

    
    print("count", count)

    #Modified
    fo = open("foo.txt" , "w")
    print ("gateway returned data ",  count-1, "\n")
    fo.write("\n")
    fo.write( msg.payload.decode())
    
    fo.close()
    # data will be dumped after this iteration

# modify the txt to have a nice list
    lines = open('foo.txt').readlines()
    fp = open('foo.txt','w')
    for s in lines:
        fp.write( s.replace('},{','\n'))
    fp.close()

    #######################################################################################
    #######################################################################################
    ####################   here to modify the mac of ble you want   #######################
    substring ="66EFB77BF19C"##############################################################
    #######################################################################################
    #######################################################################################
    #######################################################################################


    #####************TASK: can IO be removed? Try to apply Kalman filter*************######


    global ax
    global ay
    global RLx
    global RLy

    stringTime=""
    RSSI= -120  ############################** CUATION **##################################
    #####################################***initial value***###############################
    f = open("foo.txt" , "r")
    for line in f.readlines():
        if "time" in line:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            stringTime=line[start:end]
            ax.append(stringTime)
        if substring in line:
            rat+1
            # to prevent dimension mismatch, only update graph with RSSI of mac found
            start = line.find("-") 
            end = line.find('-') + 3
            RSSI=int(line[start:end])
            #print(int(line[start:end]))
            ay.append(RSSI)       #rat is len[ay]

    ####################################Apply Kalman Filter###############################
    test = KalmanFilter(0.008, 0.1)
    #      #      #      #  seperate list into smaller ones of size n  #     #     #     #
    ########################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    ########################~~~~~~~     modify n here     ~~~~~~~~########################
    n=5#####################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    ########################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    #append one point every n iteration
    c=[]
    d=[]
    total=0

    if rat % n == 0 & rat != 0:
        c= [ax[i:i+n] for i in range(0, len(ax), n)]
        d= [ay[i:i+n] for i in range(0, len(ay), n)]
        total = int(rat/n)
        for x in d[total-1]:
             afterKalman=test.filter(x)

    
    #every number in the set will go through Kalman filter once
    #append point to the graph
    ##############
    if total!=0:
         newTime=c[total-1][rat-total*n-1]
         print(newTime)
         plt.ion()                      
         RLx.append(newTime)     #append, instead of doing all over again
         RLy.append(afterKalman)        
         plt.clf()         
         plt.plot(RLx,RLy)        
         plt.pause(0.5)     #referesh graph every one second
         plt.ioff()      

   


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