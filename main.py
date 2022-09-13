import RPi.GPIO as GPIO
#from gpiozero import AngularServo
#from RPLCD import CharLCD
import time
import serial
from time import sleep
GPIO.setwarnings(False)
ser = serial.Serial ("/dev/ttyS0", 9600, timeout=0.1) #Open port with baud rate
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
#set GPIO Pins
GPIO_TRIGGER = 40
GPIO_ECHO = 38
GPIO_TRIGGER2 = 32
GPIO_ECHO2 = 36
GPIO_TRIGGER3 = 18
GPIO_ECHO3 = 16
M1A = 7
M1B = 11
M2A = 13
M2B = 15
GPIO.setup(M1A, GPIO.OUT)
GPIO.setup(M1B, GPIO.OUT)
GPIO.setup(M2A, GPIO.OUT)
GPIO.setup(M2B, GPIO.OUT)
######################
item1="item 1"
item1_price=100
item2="item 2"
item2_price=200
total_amt=0
cont=0
card_amt=1000
item1_cont=0
item2_cont=0
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER3, GPIO.OUT)
GPIO.setup(GPIO_ECHO3, GPIO.IN)
from RPLCD.gpio import CharLCD
lcd = CharLCD(cols=16, rows=4, pin_rs=37, pin_e=35, pins_data=[33, 31, 29, 23],numbering_mode=GPIO.BOARD)
button = 5
#GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
p = GPIO.PWM(3, 50)
p.start(7.5)
#lcd.cursor_pos = (0, 3)
#lcd.write_string(u'Hello worasCADld!')
tpre = time.time()
lcd.clear()
def distance():
# set Trigger to HIGH
GPIO.output(GPIO_TRIGGER, True)
# set Trigger after 0.01ms to LOW
time.sleep(0.00001)
GPIO.output(GPIO_TRIGGER, False)
StartTime = time.time()
StopTime = time.time()
# save StartTime
while GPIO.input(GPIO_ECHO) == 0:
StartTime = time.time()
# save time of arrival
while GPIO.input(GPIO_ECHO) == 1:
StopTime = time.time()
# time difference between start and arrival
TimeElapsed = StopTime - StartTime
# multiply with the sonic speed (34300 cm/s)
# and divide by 2, because there and back
distance = (TimeElapsed * 34300) / 2
return distance
def distance2():
# set Trigger to HIGH
GPIO.output(GPIO_TRIGGER2, True)
# set Trigger after 0.01ms to LOW
time.sleep(0.00001)
GPIO.output(GPIO_TRIGGER2, False)
StartTime = time.time()
StopTime = time.time()
# save StartTime
while GPIO.input(GPIO_ECHO2) == 0:
StartTime = time.time()
# save time of arrival
while GPIO.input(GPIO_ECHO2) == 1:
StopTime = time.time()
# time difference between start and arrival
TimeElapsed = StopTime - StartTime
# multiply with the sonic speed (34300 cm/s)
# and divide by 2, because there and back
distance2 = (TimeElapsed * 34300) / 2
return distance2
def distance3():
# set Trigger to HIGH
GPIO.output(GPIO_TRIGGER3, True)
# set Trigger after 0.01ms to LOW
time.sleep(0.00001)
GPIO.output(GPIO_TRIGGER3, False)
StartTime = time.time()
StopTime = time.time()
# save StartTime
while GPIO.input(GPIO_ECHO3) == 0:
StartTime = time.time()
# save time of arrival
while GPIO.input(GPIO_ECHO3) == 1:
StopTime = time.time()
# time difference between start and arrival
TimeElapsed = StopTime - StartTime
# multiply with the sonic speed (34300 cm/s)
# and divide by 2, because there and back
distance3 = (TimeElapsed * 34300) / 2
return distance3
def sms(amt):
ser.write('AT'+'\r\n')
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
write('AT+CMGF=1'+'\r\n') # Select Message format as Text mode
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
ser.write('AT+CNMI=2,1,0,0,0'+'\r\n') # New SMS Message Indications
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
# Sending a message to a particular Number
ser.write('AT+CMGS="7899274516"'+'\r\n')
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
ser.write('Amount paid: ') # Message
ser.write(amt) # Message
rcv = ser.read(10)
#print (rcv)
ser.write("\x1A") # Enable to send SMS
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
ser.write("\x1A")
rcv = ser.read(10)
#print (rcv)
time.sleep(1)
ser.write("\x1A")
def farword():
GPIO.output(M1A, False)
GPIO.output(M1B, True)
GPIO.output(M2A, False)
GPIO.output(M2B, True)
def left():
GPIO.output(M1A, False)
GPIO.output(M1B, True)
GPIO.output(M2A, False)
GPIO.output(M2B, False)
def right():
GPIO.output(M1A, False)
GPIO.output(M1B, False)
GPIO.output(M2A, False)
GPIO.output(M2B, True)
def stop():
GPIO.output(M1A, False)
GPIO.output(M1B, False)
GPIO.output(M2A, False)
GPIO.output(M2B, False)
#print("Sl No. Item Name qty amount ")
def servo():
button_state = GPIO.input(button)
if (button_state == False):
#GPIO.output(led, True)
#print('Button Pressed...')
p.ChangeDutyCycle(7.5)
time.sleep(1)
else:
p.ChangeDutyCycle(2.5)
time.sleep(1)
while True:
#servo()
received_data = ser.read() #read serial port
sleep(0.03)
data_left = ser.inWaiting() #check for remaining byte
received_data += ser.read(data_left)
if(received_data):
#print (received_data) #print received data
button_state = GPIO.input(button)
if (button_state == False and received_data=="27001715B89D"):
if(item1_cont>0):
item1_cont=item1_cont-1
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("item1 removed ")
cont=cont+1
total_amt=total_amt-item1_price
print(str(cont) + " " + str(item1)+" 1"+" "+str(item1_price)+" removed")
#sms()
else:
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("no itm added yet ")
elif (button_state == False and received_data=="270016B043C2"):
if(item2_cont>0):
item2_cont=item2_cont-1
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("item2 removed ")
cont=cont+1
total_amt=total_amt-item2_price
print(str(cont) + " " + str(item2)+" 1"+" "+str(item2_price)+" removed")
#sms()
else:
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("no itm added yet ")
elif (received_data=="27001715B89D"):
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("item1 added ")
if(cont<1):
print("")
print("")
print("-------------------------------------------------------")
print("Sl No. Item Name qty amount ")
print("-------------------------------------------------------")
cont=cont+1
item1_cont=item1_cont+1
total_amt=item1_price+total_amt
print(str(cont) + " " + str(item1)+" 1"+" "+str(item1_price))
#sms()
elif (received_data=="270016B043C2"):
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("item2 added ")
if(cont<1):
print("")
print("")
print("-------------------------------------------------------")
print("Sl No. Item Name qty amount ")
print("-------------------------------------------------------")
cont=cont+1
item2_cont=item2_cont+1
total_amt=item2_price+total_amt
print(str(cont) + " " + str(item2)+" 1"+" "+str(item2_price))
#sms()
elif (received_data=="270017317677"):
#cont=cont+1
print("-------------------------------------------------------")
print("Total "+str(total_amt))
print("-------------------------------------------------------")
if(cont<1):
print("Cart is empty")
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("Cart is empty ")
if(card_amt>total_amt):
card_amt=card_amt-total_amt
print("Amount paid:"+str(total_amt)+" Balnce:" +str(card_amt))
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("Amt paid:"+str(total_amt)+" ")
print("Sending SMS please wait")
sms(str(total_amt))
total_amt=0
cont=0
item1_cont=0
item2_cont=0
else:
  print("Insufficient fund")
lcd.cursor_pos = (0, 0)
#lcd.write_string(u'dist !')
lcd.write_string("Insufficient Amt ")
#sms()
#ser.write(received_data)
tCurrent = time.time()
if(tCurrent-tpre>0.3):
#print("delay")
tpre=time.time()
dist = distance()
#print ("Measured Distance = %.1f cm" % dist)
lcd.cursor_pos = (1, 0)
#lcd.write_string(u'dist !')
lcd.write_string("D1:%.1f " % dist)
#time.sleep(1)
dist2 = distance2()
#print ("Measured Distance = %.1f cm" % dist)
lcd.cursor_pos = (1, 8)
#lcd.write_string(u'dist !')
lcd.write_string("D2:%.1f " % dist2)
dist3 = distance3()
#print(dist3)
if(dist3<30):
stop()
#print("stop")
elif(dist >70 and dist2>70):
# print("stop")
stop()
elif(dist<70 and dist2<70):
# print("farword")
farword()
elif(dist<40):
# print("left")
left()
elif(dist2<40):
# print("right")
right()
