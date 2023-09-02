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
 
useA1=[]
useB1=[]
useA2=[]
useB2=[]
useA3=[]
useB3=[]
count = 1
preA1=[]
preB1=[]
preA2=[]
preB2=[]
preA3=[]
preB3=[]

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

def sleeptime(hour, min, sec):
    return hour * 3600 + min * 60 + sec



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


    global useA1
    global useB1
    global useA2
    global useB2
    global useA3
    global useB3
    global preA1
    global preB1
    global preA2
    global preB2
    global preA3
    global preB3

    ########################~~~~~~~     modify n here     ~~~~~~~~########################
    n=5#####################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
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
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                preA1.append(int(data['rssi']))
                preB1.append(timeStamp)
    elif json.loads(msg.payload.decode())[0]['id']==ID2:
        stringTime=json.loads(msg.payload.decode())[0]['time']
        for data in json.loads(msg.payload.decode())[1:]:
            if data['mac']==substring:
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                preA2.append(int(data['rssi']))
                preB2.append(timeStamp)
    elif json.loads(msg.payload.decode())[0]['id']==ID3:
        stringTime=json.loads(msg.payload.decode())[0]['time']
        for data in json.loads(msg.payload.decode())[1:]:
            if data['mac']==substring:
                d = datetime.datetime.strptime(stringTime, "%Y/%m/%d %H:%M:%S.%f")
                t = d.timetuple()
                timeStamp = int(time.mktime(t))
                preA3.append(int(data['rssi']))
                preB3.append(timeStamp) # dump it later

               ###########################** HERE **##########################
    ################################***  Kalman Filter  ***###############################
    #####-------@------####&&&&&    do it every 5 seconds   &&&&&#####-------@-----#######
    #####^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    V.2   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#####
    second = sleeptime(0, 0, 5)
    while 1 == 1:
        time.sleep(second)

        afterKalman1=-120
        for x in preA1:
            afterKalman1=test.filter(x)
        
        afterKalman2=-120
        for x in preA2:
            afterKalman2=test.filter(x)
        
        afterKalman3=-120
        for x in preA3:
            afterKalman3=test.filter(x)
       
        ts = time.time()

        if afterKalman1 != -120:
            if afterKalman2 != -120:
                if afterKalman3 != -120:
                    useA1.append(afterKalman1)
                    useA2.append(afterKalman2)
                    useA3.append(afterKalman3)
                    useB1.append(int(ts))
                    useB2.append(int(ts))
                    useB3.append(int(ts))

    
        
        preA1=[]
        preA2=[]
        preA3=[]
        preB1=[]
        preB2=[]
        preB3=[]
        #in the end
        #kill all the time lists
        plt.ion()                            
        plt.clf()         
        plt.plot(useB1,useA1,color='r',marker='o')
        plt.plot(useB2,useA2,color='y',marker='s')
        plt.plot(useB3,useA3,color='b',marker='o')
        plt.pause(0.1)     #referesh graph every one second
        plt.ioff()      


    ######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%######
    ##############---------------  $$$  generate graph  $$$ -----------------###############
    #####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#######
    


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