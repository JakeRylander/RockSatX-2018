#Rocksat X Payload Code for August 2018 Flight (Pi 0)
#UPRRP Team
#Coded by: John G. Wilson Negroni
#jgwilson1997@gmail.com
#---------------------------------------------

#Imports
import picamera
import time
import os.path
import math

#Board Pin Mode Stup
GPIO.setmode(GPIO.BCM)

#Variable Definitions
Start_Recording = 20
Stop_Recording = 19

#PiCamera Setup

camera = picamera.PiCamera() #Initialization
camera.framerate = 24 #Framerate
camera.shutter_speed = 41666 #Shutter Speed
camera.resolution = '1080p' #Resolution
camera.iso = 100 #ISO

filename = ''

#Misc
time_to_launch = 34 #225 full sequence(-16 is the marging between actual time and boot up) IE 240 is 225

#IO Setup

GPIO.setup(Start_Recording, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(Stop_Recording, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#Functions
#File Existing Check
def FileIsThere():

        IsNotThere = 1
        num = 0

        while (IsNotThere):
                
                filename = ''.join(['PayloadRecording',str(num),'.h264'])
        
                if(os.path.isfile(filename)):
                        num += 1
                else:
				
	time.sleep(.20)
	
#Flag Check
def InputCheck(pin):

        InputToCheck = 1
        PrintOut = ''
        check = 1
        Count = 0

        if (pin == 19):
                InputToCheck = 1
                PrintOut = 'Waiting to Start Recording at T(sec)= '
        if (pin == 20):
                InputToCheck = 1
                PrintOut = 'Waiting to Stop Recording at T(sec)= '
        while(check):
                for x in range(0,13):
                        if(GPIO.input(pin) == InputToCheck):
                                Count += 1
                if (Count > 10):
                        check = 0       
                else:
                        check = 1
                        print(PrintOut)
                        GetTime()
                Count = 0
			
#Get Time
def truncate(number, digits) -> float:
        a = pow(10.0, digits)
        return math.trunc(a * number) / a
        
def GetTime():
	current_time = time.perf_counter() - time_to_launch
	current_time = truncate(current_time, 2)
	print(str(current_time))
	time.sleep(.20)


#----------------------------------------------------------------
print('Software RockSat X 2018 Revision 7/18/18 (Pi 0)')
print('This software is for August Flight')
print('Pi 0 Alive at T(sec)= ')
GetTime()

#Create Filename for Camera
filename = FileIsThere()

InputCheck(Start_Recording)

camera.start_recording(filename)
print('Camera Started Recording at T(sec)= ')
GetTime()

InputCheck(Stop_Recording)

camera.stop_recording
print('Camera Stopped Recording at T(sec)= ')
GetTime()