#from SerialCommand import Com
import RPi.GPIO as GPIO
from SerialCommand_copy import Com
import jetson.inference
import jetson.utils
import cv2
import argparse
import sys
from threading import Thread
import time

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]


income_img = None

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)
oldtarL = 0
oldtarR = 0
oldtarB = 0

tarL = 0
tarR = 0
tarB = 0

dirL = 0
dirR = 0
dirB = 0 

olddirL = 0
olddirR = 0
olddirB = 0

ebashit = 0
oldGoal=0 # last seen:  0 - left, 1 - right
oldBall=0 # last seen:  0 - left, 1 - right

deltaCenter = 115

speed = 10

fullspeed = 28

goal = None    # target:  2 - yellow, 3 - blue

koooooficent = 1.0

stop = 1

# create video sources & outputs
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)




def kuda (nap):
    
    print(nap)
    global oldtarL
    global oldtarR
    global oldtarB

    global tarL
    global tarR
    global tarB

    global dirL
    global dirR
    global dirB

    global olddirL
    global olddirR
    global olddirB
    
    global speed
    global fullspeed
    
    global koooooficent
    
    if koooooficent>1:
        koooooficent = 1
    elif koooooficent<0.25:
        koooooficent = 0.25
    
    if (nap == "forward"):
        dirL=1
        dirR=0
        dirB=0
        
        tarL=speed
        tarR=speed
        tarB=0
        
    elif (nap == "fullforward"):
        dirL=1
        dirR=0
        dirB=0
        
        tarL=fullspeed
        tarR=fullspeed
        tarB=0
        

    elif (nap == "backward"):
        dirL=0
        dirR=1
        dirB=0
        
        tarL=speed
        tarR=speed
        tarB=0
        

    elif (nap == "stop"):
        dirL=1
        dirR=0
        dirB=0
        
        tarL=0
        tarR=0
        tarB=0
        

    elif (nap == "left"):
        dirL=0
        dirR=1
        dirB=1
        
        tarL = int(speed*0.5)
        tarR = int(speed*0.5)
        tarB = int(speed*1)    
    
    elif (nap == "right"):
        dirL=1
        dirR=0
        dirB=0
        
        tarL = int(speed*0.5)
        tarR = int(speed*0.5)
        tarB = int(speed*1)
    
    elif (nap == "curveleft"):
        dirL=0
        dirR=0
        dirB=1
        
        tarL = int(speed*0.6)
        tarR = int(speed*1)
        tarB = int(speed*1)    
        


    elif (nap == "curveright"):
        dirL=1
        dirR=1
        dirB=0
        
        tarL = int(speed*1)
        tarR = int(speed*0.6)
        tarB = int(speed*1)
        


    elif (nap == "rotleft"):
        dirL=0
        dirR=0
        dirB=0
        
        tarL=int(speed*0.5*koooooficent)
        tarR=int(speed*0.5*koooooficent)
        tarB=int(speed*0.5*koooooficent)
        


    elif (nap == "rotright"):
        dirL=1
        dirR=1
        dirB=1
        
        tarL=int(speed*0.5*koooooficent)
        tarR=int(speed*0.5*koooooficent)
        tarB=int(speed*0.5*koooooficent)
    print("koooooficent = ",koooooficent)
    '''    
    print("\n\n\n\n\ndirL = ", dirL ,"dirR = ", dirR ,"dirB = ", dirB)
    print("oldL = ", olddirL ,"oldR = ", olddirR ,"oldB = ", olddirB,"\n\n")
    
    print("tarL = ", tarL ,"tarR = ", tarR ,"tarB = ", tarB)
    print("oldL = ", oldtarL ,"oldR = ", oldtarR ,"oldB = ", oldtarB,"\n\n")
    '''
    
    





def toArduino():
    s = Com(ARDPath = "/dev/ttyUSB0", portSpeed = 115200)
    global oldtarL
    global oldtarR
    global oldtarB

    global tarL
    global tarR
    global tarB

    global dirL
    global dirR
    global dirB

    global olddirL
    global olddirR
    global olddirB
    global stop
    global goal
    global ebashit
    
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(40, GPIO.IN)  # button pin set as input
    GPIO.setup(38, GPIO.IN)
    GPIO.setup(37, GPIO.IN)
    GPIO.setup(36, GPIO.IN)
    '''
    while(1):
        if ((tarL != oldtarL)|(dirL != olddirL)):
            s.writeCmd(0|(0<<6)|(dirL<<5)|(tarL))
            olddirL=dirL
            oldtarL=tarL
        if ((tarR != oldtarR)|(dirR != olddirR)):
            s.writeCmd(0|(1<<6)|(dirR<<5)|(tarR))
            olddirR=dirR
            oldtarR=tarR
        if ((tarB != oldtarB)|(dirB != olddirB)):
            s.writeCmd(0|(2<<6)|(dirB<<5)|(tarB))
            olddirB=dirB
            oldtarB=tarB
    '''
    helpstop = 0
    while(1):
        if(GPIO.input(40)):
            while (GPIO.input(40)):
                continue
            if (stop):
                stop = 0
            else:
                stop = 1 
        if(GPIO.input(38)):
            goal = 3
        if(GPIO.input(37)):
            stop = 0
            ebashit = 1
        if(GPIO.input(36)):
            goal = 2
        if stop and (not helpstop):
            s.writeCmd_(0|(0<<6)|(dirL<<5)|(0),0|(1<<6)|(dirR<<5)|(0),0|(2<<6)|(dirB<<5)|(0))
            time.sleep(0.005)
            helpstop = 1
        elif (not stop) and ((tarL != oldtarL)|(dirL != olddirL)|(tarR != oldtarR)|(dirR != olddirR)|(tarB != oldtarB)|(dirB != olddirB)):
            helpstop = 0
            s.writeCmd_(0|(0<<6)|(dirL<<5)|(tarL),0|(1<<6)|(dirR<<5)|(tarR),0|(2<<6)|(dirB<<5)|(tarB))
            #print("sent to arduino",tarL, tarR, tarB)
            
            time.sleep(0.005)

        


def stream():
    cap = cv2.VideoCapture(0)
    cap.set(5, 30)
    global income_img
    while(1):
        global income_img
        ret, income_img = cap.read()	
        #cv2.imshow("stream", income_img)
        #if (cv2.waitKey(1) & (0xFF==ord('q'))):
        #    break
        
        
        
        
def main():
    time.sleep(25)
    print("exit sleep")
    global oldBall
    global oldGoal
    global goal
    global koooooficent
    global deltaCenter
    global ebashit
    t = None
    tl = None
    tr = None
    check = 0
    while True:
        global income_img
        img = jetson.utils.cudaFromNumpy(income_img)
        detections = net.Detect(img, overlay=opt.overlay)
        

        #print("detected {:d} objects in image".format(len(detections)))
        ball = None
        ball_con = 0
        target_goal = None
        target_goal_con = 0
        koooooficent = 1.0
        if(goal == 2):
            print("target = yellow")
        if(goal == 3):
            print("target = blue")
        for i in detections:
            #print(i.ClassID,"\n", i.Center[0],"\n\n\n")
            if(i.Confidence>=0.6):
                cv2.rectangle(income_img, (int(i.Left),int(i.Top)),(int(i.Right),int(i.Bottom)),(0,255,0),2)
            if(i.ClassID==1): #53
                if(i.Confidence>ball_con):
                    ball = i
                    ball_con = i.Confidence
            if(i.ClassID==goal):
                if(i.Confidence>target_goal_con):
                    target_goal = i
                    target_goal_con = i.Confidence


        if(not ebashit):
            if((ball == None)|(ball_con < 0.5)):
                if (t == None):
                    t = time.monotonic()
                if (time.monotonic()-t>3):
                    if check == 0:
                        kuda("backward")
                    if check == 1:
                        kuda("left")
                    if check == 2:
                        kuda("right")
                    if check == 3:
                        kuda("forward")
                    time.sleep(0.5)
                    check += 1
                    if (check > 3):
                        check = 0
                    t = None
                                        
                if(oldBall == 0):
                    koooooficent = 1.0
                    kuda("rotleft")
                else:
                    koooooficent = 1.0 
                    kuda("rotright")
            else:
                t = None
                check = 0
                #print(ball.Center[0])
                #print("\n\n\n\n\n\n\n\n\n\n aaaaaaaaaaaaaareaaaaaaaaaaaaaaaaa = ", ball.Area,"\n\n\n\n\n\n")
                if(ball.Center[0]>320+deltaCenter):
                    koooooficent = (ball.Center[0]-320)/500
                    kuda("rotright")
                    oldBall = 1
                else:
                    if(ball.Center[0]<320-deltaCenter):
                        koooooficent = (320 - ball.Center[0])/500
                        kuda("rotleft")
                        oldBall = 0
                    else:
                        if(ball.Area < 30000):
                            kuda("forward")
                        else:
                            if ((target_goal == None)|(target_goal_con < 0.6)):
                                if(oldGoal == 0):
                                    #koooooficent = 1.0
                                    kuda("curveright")
                                else:
                                    #koooooficent = 1.0 
                                    kuda("curveleft")
                            else:
                                if(target_goal.Center[0]>320+deltaCenter):
                                    #koooooficent = (target_goal.Center[0]-320)/500
                                    kuda("curveleft")
                                    oldGoal = 1
                                else:
                                    if(target_goal.Center[0]<320-deltaCenter):
                                        #koooooficent = (320 - target_goal.Center[0])/500
                                        kuda("curveright")
                                        oldGoal = 0
                                    else:
                                        kuda("fullforward")
        else:
            if((ball == None)|(ball_con < 0.5)):
                ebashit = 0
            else:
                if(ball.Center[0]>320+deltaCenter*2)|(ball.Center[0]<320-deltaCenter*2):
                    ebashit = 0
                else:
                    kuda("forward")
            
                       
                    #koooooficent = 1.0
                    #kuda("forward")

        cv2.imshow("custom",income_img)
        if (cv2.waitKey(1) & (0xFF==ord('q'))):
            break

stream_ = Thread(target = stream)
main_ = Thread(target = main)
toArduino_ = Thread(target = toArduino)

stream_.start()
toArduino_.start()
main_.start()

print("all threads have been stared")
#stream_ = Thread(target = stream)
#stream_.start()
    

