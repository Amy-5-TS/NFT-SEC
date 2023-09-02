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
        
    else:
        print("Failed to connect, return code {rc}".format(rc=rc), )
def on_message(client, userdata, msg):
    #Modified
    global count
    count = count+1
    fo = open("foo.txt" , "w")
    print ("gateway returned data ",  count-1, "\n")
    fo.write("\n")
    fo.write("Set index:")
    fo.write(str(count-1))
    fo.write("\n")
    fo.write( msg.payload.decode())
    
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

    #######################################################################################
    #######################################################################################
    ####################   here to modify the mac of ble you want   #######################
    substring ="E08B98F92C33"##############################################################
    #######################################################################################
    #######################################################################################
    #######################################################################################
    test = KalmanFilter(0.008, 0.1)
    global ax
    global ay
    c=[]
    d=[]
    global useA
    global useB
    ########################~~~~~~~     modify n here     ~~~~~~~~########################
    n=5#####################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    total=0

    global rat
    stringTime=""
    RSSI= -120  ############################** CUATION **##################################
    #####################################***initial value***###############################
    f = open("foo.txt" , "r")
    for line in f.readlines():
        if "time" in line:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            stringTime=line[start:end]
        if substring in line:
            # to prevent dimension mismatch, only update graph with RSSI of mac found
            start = line.find("-") 
            end = line.find('-') + 3
            ay.append(int(line[start:end]))
            ax.append(stringTime)
            rat=rat+1
            
            #print(int(line[start:end]))


    if rat!=0:
        if rat % n == 0:
            c= [ax[i:i+n] for i in range(0, len(ax), n)]    #time
            d= [ay[i:i+n] for i in range(0, len(ay), n)]    #RSSI strength
            total = int(rat/n)
            for x in d[total-1]:
                 afterKalman=test.filter(x)
            print("////////////////////////////////////////////////////////////////////////")
            print("              /\      ")
            print("             /  \     ")
            print("            /    \    ")
            print("            \    /    ")
            print("             \  /     ")
            print("              \/      ")
            print("Got something new: one more Kalman filtered value: ", afterKalman,"\n")
            print(total, " Kalman filtered RSSIs in total")
            useA.append(c[total-1][rat-n*total-1])
            useB.append(afterKalman)
    print(useB)
    print("////////////////////////////////////////////////////////////////////////")
    
    
    plt.ion()                            
    plt.clf()         
    plt.plot(useA,useB)        
    plt.pause(0.5)     #referesh graph every one second
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