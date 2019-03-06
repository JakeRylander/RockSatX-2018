#Rocksat X Payload Code for August 2018 Flight
#UPRRP Team
#Coded by: John G. Wilson Negroni
#jgwilson1997@gmail.com
#---------------------------------------------

#Imports
import serial
import picamera
import time
import math
import RPi.GPIO as GPIO
import os.path
from Stepper import Stepper
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

#Serial Setup

serialOUT = serial.Serial(port = '/dev/ttyAMA0', baudrate = 19200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

#Board Pin Mode Setup

GPIO.setmode(GPIO.BCM)

#Variable Definitions
#Components
Inhibit = 19.0
GSE_Test = 27
Leica = 20
Leica2 = 21
Proximity_Sensor = 5
Plasma = 18
UV = 4

#PiCamera Setup

camera = picamera.PiCamera() #Initialization
camera.framerate = 24 #Framerate
camera.shutter_speed = 41666 #Shutter Speed
camera.resolution = '1080p' #Resolution
camera.iso = 100 #ISO

filename = ''

#Stepper Motor
Step_Ena = 12
Step_Dir = 13
Step_Step = 16

#Flags
Launch = 22
Skirt = 23
PowerOffin30 = 24

#Misc
time_to_launch = 225 #225 full sequence(-16 is the marging between actual time and boot up) IE 240 is 225

Count = 0
check = 1
GSE_Test_Active = 0

#IO Setup
#Inhibit
GPIO.setup(Inhibit, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#GSE Test
GPIO.setup(GSE_Test, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#CameraLeica 
GPIO.setup(Leica, GPIO.OUT)
GPIO.setup(Leica2, GPIO.OUT)

#Stepper Motor
GPIO.setup(Step_Ena, GPIO.OUT)
GPIO.setup(Step_Dir, GPIO.OUT)
GPIO.setup(Step_Step, GPIO.OUT)

#Flags
GPIO.setup(Launch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Skirt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PowerOffin30, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Proximity Sensor
GPIO.setup(Proximity_Sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Plasma
GPIO.setup(Plasma, GPIO.OUT)

#UV
GPIO.setup(UV, GPIO.OUT)

#Stepper Setup

DoorStepper = Stepper([Step_Ena,Step_Dir,Step_Step])

#Clamp Shell Setup

MH = Adafruit_MotorHAT(addr=0x60)
ClampShell = MH.getMotor(1)

#Function Definitions
#File Exist Check
def FileIsThere():

        IsNotThere = 1
        num = 0

        while (IsNotThere):
                
                filename = ''.join(['PayloadRecording',str(num),'.h264'])
        
                if(os.path.isfile(filename)):
                        num += 1
                else:
				
	serialOUT.write('\n'.encode())
	time.sleep(.20)

#Flag Check

def FlagCheck(flag):

        InputToCheck = 1
        PrintOut = ''
        check = 1
        Count = 0

        if (flag == 19):
                InputToCheck = 1
                PrintOut = 'Payload Inhibited at T(sec)= '
        if (flag == 22):
                InputToCheck = 1
                PrintOut = 'Time to Launch T(sec)= '
        if (flag == 23):
                InputToCheck = 1
                PrintOut = 'Flying Time T(sec)= '
        if (flag == 5):
                InputToCheck = 0
                PrintOut = 'Skirt is not off at T(sec)= '
        if (flag == 24):
                InputToCheck = 1
                PrintOut = 'Door still open at T(sec)= '
        while(check):
                for x in range(0,13):
                        if(GPIO.input(flag) == InputToCheck):
                                Count += 1
                if (Count > 10):
                        check = 0       
                else:
                        check = 1
                        serialOUT.write(PrintOut.encode())
                        GetTime()
                Count = 0
	
#Turn on Leica
def TurnOnLeica(GSE_Test):

        if (GSE_Test):
                serialOUT.write('Camera Leica not turning on, GSE Test Active at T(sec)= '.encode())
                GetTime()
        else:          
                GPIO.output(Leica, GPIO.HIGH)
                GPIO.output(Leica2, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(Leica, GPIO.LOW)
                GPIO.output(Leica2, GPIO.LOW)
                time.sleep(3)
                GPIO.output(Leica, GPIO.HIGH)
                GPIO.output(Leica2, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(Leica, GPIO.LOW)
                GPIO.output(Leica2, GPIO.LOW)
                serialOUT.write('Leica Camera on T(sec)= '.encode())
                GetTime()
	
#Open Door
def OpenDoor(GSE_Test):

        if(GSE_Test):
                serialOUT.write('Door not Opening. GSE Test Active at T(sec)= '.encode())
                GetTime()
        else:  
                #Open ClampShell
                ClampShell.run(Adafruit_MotorHAT.FORWARD)
                ClampShell.setSpeed(255)
                time.sleep(2.3)
                ClampShell.setSpeed(0)
                ClampShell.run(Adafruit_MotorHAT.RELEASE)
                
                #Open Door
                DoorStepper.step(122500, dir = 'right') #right == open

                serialOUT.write('Door Opened at T(sec)= '.encode())
                GetTime()
	
#Close Door
def CloseDoor(GSE_Test):

        if(GSE_Test):
                serialOUT.write('Door not Closing. GSE Test Active at T(sec)= '.encode())
                GetTime()
        else:  
                #Close Door
                DoorStepper.step(125000, dir = 'left') #left == close 
                
                #Close Clamp Shell
                ClampShell.run(Adafruit_MotorHAT.BACKWARD)
                ClampShell.setSpeed(255)
                time.sleep(2.4)
                ClampShell.setSpeed(0)
                ClampShell.run(Adafruit_MotorHAT.RELEASE)

                serialOUT.write('Door Closed at T(sec)= '.encode())
                GetTime()
	
#--------------------------------------------------------------------------
#Begin Program
#Starting Print

serialOUT.write('Software RockSat X 2018 Revision 7/31/17 \n'.encode())
serialOUT.write('This software is for August Flight \n'.encode())
serialOUT.write('UPR Payload Alive T(sec)= '.encode())
GetTime()

#Check for GSE Test and set Flag if it is

serialOUT.write('Checking for GSE Test at T(sec)= '.encode())
GetTime()

for x in range(0,13):
		if(GPIO.input(27) == 1):
                Count += 1
if (Count > 10):
		check = 0
		serialOUT.write('GSE Test is NOT active at T(sec)= '.encode())
		GetTime()
else:
		check = 1
		serialOUT.write('GSE Test is Active at T(sec)= '.encode())
		GetTime()
		GSE_Test_Active = 1
Count = 0

#Inhibit
FlagCheck(Inhibit)

#Create Name for Video file for Raspicam

filename = FileIsThere()

#Activate UV

GPIO.output(UV, GPIO.HIGH)
	
#Cameras turn on 15 seconds before launch

fifteen_to_launch = (time.perf_counter() - 9) + (time_to_launch - 15)

while (time.perf_counter() < fifteen_to_launch):
        serialOUT.write('Time to Launch T(sec)= '.encode())
        GetTime()

#Turn on Leica Camera

TurnOnLeica(GSE_Test_Active)

#Launch Flag Wait

FlagCheck(Launch)

#Rocket Launched

serialOUT.write('Launch T(sec)= '.encode())
GetTime()


#RaspiCam on at 60 seconds

while (time.perf_counter() < (60 + time_to_launch)):
	serialOUT.write('Flying Time T(sec)= '.encode())
	GetTime()

if (GSE_Test_Active):
        serialOUT.write('Pi camera not turned on. GSE Test Active at T(sec)= '.encode())
        GetTime()
else:
        camera.start_recording(filename)
        serialOUT.write('Pi camera on at: T(sec)= '.encode())
        GetTime()

#Skirt Flag Wait
FlagCheck(Skirt)

#Activate Plasma
GPIO.output(Plasma, GPIO.HIGH)

serialOUT.write('Plasma is ON at T(sec)= '.encode())
GetTime()

#Wait for Skirt Clear 
FlagCheck(Proximity_Sensor)

#Skirt Off
serialOUT.write('Skirt Off T(sec)= '.encode())
GetTime()

while (time.perf_counter() < (200 + time_to_launch)):
	serialOUT.write('Plasma is ON at T(sec)= '.encode())
	GetTime()

#Deactivate Plasma and UV
GPIO.output(UV, GPIO.LOW)
GPIO.output(Plasma, GPIO.LOW)

serialOUT.write('Plasma and UV are off at T(sec)= '.encode())
GetTime()

#Door Open

serialOUT.write('Door Opening Started at T(sec)= '.encode())
GetTime()

#Door Opening

OpenDoor(GSE_Test_Active)

#30 Seconds before Power Off Flag Wait

FlagCheck(PowerOffin30)

#Door Closing
	
serialOUT.write('Door Closing T(sec)= '.encode())
GetTime()

CloseDoor(GSE_Test_Active)

#Door Closed

#Wait for 15 seconds before power off to turn off Pi Camera

while (time.perf_counter() < (325 + time_to_launch)):
	serialOUT.write('Waiting to turn off Picamera T(sec)= '.encode())
	GetTime()

#Stop Pi Camera Recording

if (GSE_Test_Active):
        serialOUT.write('Pi camera not turned off. GSE Test Active at T(sec)= '.encode())
        GetTime()
else:
        camera.stop_recording
        serialOUT.write('Picamera of at T(sec)= '.encode())
        GetTime()

#Sequence End

serialOUT.write('Going back Home T(sec)= '.encode())
GetTime()
