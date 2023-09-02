# python 3.x
# this version uses list of dictionary instead of File I/O
# PROBLEM: 
import json
import random
import time
import matplotlib.pyplot as plt 
import numpy
import datetime
import time

ax1 = []                     
ay1 = []  
useA1=[]
useB1=[]
rat1=0
ax2 = []                     
ay2 = []  
useA2=[]
useB2=[]
rat2=0
ax3 = []                     
ay3 = []  
useA3=[]
useB3=[]
rat3=0
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


##########################--------------least square--------------##########################
import math

# Assume those points are:
a = numpy.matrix('-82 135; 150 91; 118 -52')
# Assume those distance are:
b = numpy.array([106.6,197.52,181.34])

x1 = a.item((0, 0))
x2 = a.item((1, 0))
x3 = a.item((2, 0))


y1 = a.item((0, 1))
y2 = a.item((1, 1))
y3 = a.item((2, 1))


def B_row_cal (xa,ya,xn,yn,da,dn):
  result = numpy.square(xa)+numpy.square(ya)-numpy.square(xn)-numpy.square(yn)-numpy.square(da)+numpy.square(dn)
  return result

def A_row_cal (xa,ya,xn,yn):
  result = [2*(xa-xn),2*(ya-yn)]
  return result 


##########################--------------least square--------------##########################



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
        client.subscribe(TOPIC1)
        client.subscribe(TOPIC2)
        client.subscribe(TOPIC3)
        
    else:
        print("Failed to connect, return code {rc}".format(rc=rc), )
def on_message(client, userdata, msg):
    #Modified
    ID1="C6E67C4F099B"
    ID2="D892A794427E"
    ID3="ECA55EBEEF1A"
    test = KalmanFilter(0.008, 0.1)
    
    global ax1
    global ay1
    global ax2
    global ay2
    global ax3
    global ay3
    c1=[]
    d1=[]
    c2=[]
    d2=[]
    c3=[]
    d3=[]
    global useA1
    global useB1
    global useA2
    global useB2
    global useA3
    global useB3
    global rat1
    global rat2
    global rat3

    ########################~~~~~~~     modify n here     ~~~~~~~~########################
    n=5#####################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    total1=0
    total2=0
    total3=0
    stringTime=""


    #######################################################################################
    #######################################################################################
    ####################   here to modify the mac of ble you want   #######################
    substring ="E73439667E0D"##############################################################
    #######################################################################################
    #######################################################################################
    #######################################################################################

    if json.loads(msg.payload.decode())[0]['id']==ID1:
        stringTime=json.loads(msg.payload.decode())[0]['time']
        for data in json.loads(msg.payload.decode())[1:]:
            if data['mac']==substring:
                ay1.append(int(data['rssi']))
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                ax1.append(timeStamp)
                rat1=rat1+1
    elif json.loads(msg.payload.decode())[0]['id']==ID2:
        stringTime=json.loads(msg.payload.decode())[0]['time']
        for data in json.loads(msg.payload.decode())[1:]:
            if data['mac']==substring:
                ay2.append(int(data['rssi']))
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                ax2.append(timeStamp)
                rat2=rat2+1
    elif json.loads(msg.payload.decode())[0]['id']==ID3:
        stringTime=json.loads(msg.payload.decode())[0]['time']
        for data in json.loads(msg.payload.decode())[1:]:
            if data['mac']==substring:
                ay3.append(int(data['rssi']))
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                ax3.append(timeStamp)
                rat3=rat3+1
               ############################** HERE **##########################
    ################################***  Kalman Filter  ***###############################

    afterKalman=0
    if rat1!=0:
        if rat1 % n == 0:
            c1= [ax1[i:i+n] for i in range(0, len(ax1), n)]    #time
            d1= [ay1[i:i+n] for i in range(0, len(ay1), n)]    #RSSI strength
            total1 = int(rat1/n)
            for x in d1[total1-1]:
                 afterKalman=test.filter(x)
            useA1.append(c1[total1-1][rat1-n*total1-1]) # the latest time in 5 numbers (timestamp)
            useB1.append(afterKalman)

    if rat2!=0:
        if rat2 % n == 0:
            c2= [ax2[i:i+n] for i in range(0, len(ax2), n)]    #time
            d2= [ay2[i:i+n] for i in range(0, len(ay2), n)]    #RSSI strength
            total2 = int(rat2/n)
            for x in d2[total2-1]:
                 afterKalman=test.filter(x)
            useA2.append(c2[total2-1][rat2-n*total2-1])
            useB2.append(afterKalman)

    if rat3!=0:
        if rat3 % n == 0:
            c3= [ax3[i:i+n] for i in range(0, len(ax3), n)]    #time
            d3= [ay3[i:i+n] for i in range(0, len(ay3), n)]    #RSSI strength
            total3 = int(rat3/n)
            for x in d3[total3-1]:
                 afterKalman=test.filter(x)
            useA3.append(c3[total3-1][rat3-n*total3-1])
            useB3.append(afterKalman)
    ######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%######
    ######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   timeline   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####

    final1=[]
    final2=[]
    final3=[]
    finalTime=[]

    for x in useA1: #check in three, about value of timestamp
        for y in useA2:
            for z in useA3:
                if x==y:
                    if y==z:
                        final1.append(useA1.index(x))
                        final2.append(useA2.index(y))
                        final3.append(useA3.index(z))
                        finalTime.append(x)

                        




    ######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%######
    ##############---------------  $$$  generate graph  $$$ -----------------###############
    #####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#######


    plt.ion()                            
    plt.clf()         
    plt.plot(finalTime,final1,color='r',marker='o')
    plt.plot(finalTime,final2,color='y',marker='s')
    plt.plot(finalTime,final3,color='b',marker='o')
    plt.pause(0.1)     #referesh graph every one second
    plt.ioff()      


##########################--------------least square--------------##########################

 #   B = numpy.array([B_row_cal(x1,y1,x3,y3,b[0],b[2]),B_row_cal(x2,y2,x3,y3,b[1],b[2])])
  #  A = numpy.matrix([A_row_cal(x1,y1,x3,y3),A_row_cal(x2,y2,x3,y3)])
   # A_bar = A.transpose()
   # result=numpy.dot(numpy.linalg.inv(numpy.dot(A_bar,A)),(numpy.dot(A_bar,B)).transpose())
    #print(result)
##########################--------------least square--------------##########################

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