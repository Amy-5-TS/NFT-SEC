# python 3.x
import json
import random
import time
import matplotlib.pyplot as plt 
import numpy

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
dis1=[]
dis2=[]
dis3=[]
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
    global count
    count = count+1
    fo1 = open("foo1.txt" , "a")
    fo2 = open("foo2.txt" , "a")
    fo3 = open("foo3.txt" , "a")
    print ("gateway returned data ",  count-1, "\n")
    if ID1 in msg.payload.decode():
        fo1.write( msg.payload.decode())
    if ID2 in msg.payload.decode():
        print(ID2)
        fo2.write( msg.payload.decode())
    if ID3 in msg.payload.decode():
        fo3.write( msg.payload.decode())
    
    fo1.close()
    fo2.close()
    fo3.close()


    ##########---###########`````````` here update ID ``````````###########---##########
   
    ##########---###########`````````` here update ID ``````````###########---##########
    # data will be dumped after this iteration
    lines1 = open('foo1.txt').readlines()
    fp = open('foo1.txt','w')
    for s in lines1:
        fp.write( s.replace('},{','\n'))
    fp.close()

    lines2 = open('foo2.txt').readlines()
    fp = open('foo2.txt','w')
    for s in lines2:
        fp.write( s.replace('},{','\n'))
    fp.close()

    lines3 = open('foo3.txt').readlines()
    fp = open('foo3.txt','w')
    for s in lines3:
        fp.write( s.replace('},{','\n'))
    fp.close()

    #######################################################################################
    #######################################################################################
    ####################   here to modify the mac of ble you want   #######################
    substring ="E73439667E0D"##############################################################
    #######################################################################################
    #######################################################################################
    #######################################################################################
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
    ########################~~~~~~~     modify n here     ~~~~~~~~########################
    n=5#####################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~########################
    total1=0
    total2=0
    total3=0

    global dis1
    global dis2
    global dis3

    global rat1
    global rat2
    global rat3
    stringTime=""
    RSSI= -120  ############################** CUATION **##################################
    #####################################***initial value***###############################

    pd0 = -60.073810363075786
    n = 3.591969



    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    ##############&&&&&&&&&&&&&&           ID1            &&&&&&&&&&&&&&&&&&########
    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    f = open("foo1.txt" , "r")
    for line in f.readlines():
        if "time" in line:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            stringTime=line[start:end]
        if substring in line:
            # to prevent dimension mismatch, only update graph with RSSI of mac found
            start = line.find("-") 
            end = line.find('-') + 3
            ay1.append(int(line[start:end]))
            ax1.append(stringTime)
            rat1=rat1+1
            
            #print(int(line[start:end]))

    u1=open("kalman1.txt","a")

    if rat1!=0:
        if rat1 % n == 0:
            c1= [ax1[i:i+n] for i in range(0, len(ax1), n)]    #time
            d1= [ay1[i:i+n] for i in range(0, len(ay1), n)]    #RSSI strength
            total1 = int(rat1/n)
            for x in d1[total1-1]:
                 afterKalman=test.filter(x)
                 dis1.append(math.pow((afterKalman-pd0)/(10*n), 10))
            print("////////////////////////////////////////////////////////////////////////")
            print("        /\      ")
            print("       /  \     ","Got something new: one more Kalman filtered value: ", afterKalman,"\n")
            print("      / 1  \    ",total1, " Kalman filtered RSSIs in total")
            print("      \    /    ")
            print("       \  /     ")
            print("        \/      ")
            useA1.append(c1[total1-1][rat1-n*total1-1])
            u1.write(c1[total1-1][rat1-n*total1-1])
            u1.write("\n")
            useB1.append(afterKalman)
            u1.write(str(afterKalman))
            u1.write("\n")
    print(useB1)
    u1.close()
    
    f.close()


    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    ##############&&&&&&&&&&&&&&           ID2            &&&&&&&&&&&&&&&&&&########
    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##

    f = open("foo2.txt" , "r")
    for line in f.readlines():
        if "time" in line:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            stringTime=line[start:end]
        if substring in line:
            # to prevent dimension mismatch, only update graph with RSSI of mac found
            start = line.find("-") 
            end = line.find('-') + 3
            ay2.append(int(line[start:end]))
            ax2.append(stringTime)
            rat2=rat2+1
            
            #print(int(line[start:end]))

    u2=open("kalman2.txt","a")
    if rat2!=0:
        if rat2 % n == 0:
            c2= [ax2[i:i+n] for i in range(0, len(ax2), n)]    #time
            d2= [ay2[i:i+n] for i in range(0, len(ay2), n)]    #RSSI strength
            total2 = int(rat2/n)
            for x in d2[total2-1]:
                 afterKalman=test.filter(x)
                 dis2.append(math.pow((afterKalman-pd0)/(10*n), 10))
            print("        /\      ")
            print("       /  \     ","Got something new: one more Kalman filtered value: ", afterKalman,"\n")
            print("      / 2  \    ",total2, " Kalman filtered RSSIs in total")
            print("      \    /    ")
            print("       \  /     ")
            print("        \/      ")
            
            useA2.append(c2[total2-1][rat2-n*total2-1])
            u2.write(c2[total2-1][rat2-n*total2-1])
            u2.write("\n")
            useB2.append(afterKalman)
            u2.write(str(afterKalman))
            u2.write("\n")
    print(useB2)
    u2.close()



    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    ##############&&&&&&&&&&&&&&           ID3            &&&&&&&&&&&&&&&&&&########
    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    


    f = open("foo3.txt" , "r")
    for line in f.readlines():
        if "time" in line:
            start = line.find(" ") + 1
            end = line.find('id') - 4
            stringTime=line[start:end]
        if substring in line:
            # to prevent dimension mismatch, only update graph with RSSI of mac found
            start = line.find("-") 
            end = line.find('-') + 3
            ay3.append(int(line[start:end]))
            ax3.append(stringTime)
            rat3=rat3+1
            
            #print(int(line[start:end]))
    f.close()

    u3 =open("kalman2.txt","a")
     
    if rat3!=0:
        if rat3 % n == 0:
            c3= [ax3[i:i+n] for i in range(0, len(ax3), n)]    #time
            d3= [ay3[i:i+n] for i in range(0, len(ay3), n)]    #RSSI strength
            total3 = int(rat3/n)
            for x in d3[total3-1]:
                 afterKalman=test.filter(x)
                 dis3.append(math.pow((afterKalman-pd0)/(10*n), 10))
            print("        /\      ")
            print("       /  \     ","Got something new: one more Kalman filtered value: ", afterKalman,"\n")
            print("      / 3  \    ",total3, " Kalman filtered RSSIs in total")
            print("      \    /    ")
            print("       \  /     ")
            print("        \/      ")
            useA3.append(c3[total3-1][rat3-n*total3-1])
            u3.write(c3[total3-1][rat3-n*total3-1])
            u3.write("\n")
            useB3.append(afterKalman)
            u3.write(str(afterKalman))
            u3.write("\n")
    print(useB3)
    u3.close()
    print("////////////////////////////////////////////////////////////////////////")
    


     ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
    ##############---------------       generate graph      -----------------########
    ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###


    plt.ion()                            
    plt.clf()         
    plt.plot(useA1,useB1,color='r',marker='o')
    plt.plot(useA2,useB2,color='y',marker='s')
    plt.plot(useA3,useB3,color='b',marker='o')
    plt.pause(0.5)     #referesh graph every one second
    plt.ioff()      




##########################--------------least square--------------##########################
# careful : dimensions
    
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