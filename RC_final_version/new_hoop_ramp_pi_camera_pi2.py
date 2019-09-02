# import the necessary packages
import numpy as np
import argparse
import cv2
import time
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
ser = serial.Serial('/dev/serial0', baudrate=9600,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1)

arduino_start=11
arduino_stop=12
arduino_comand=0
import RPi.GPIO as GPIO
from smbus import SMBus
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

GPIO.setup(arduino_stop,GPIO.OUT)
GPIO.output(arduino_stop, GPIO.HIGH)
GPIO.setup(arduino_start,GPIO.OUT)
GPIO.output(arduino_start, GPIO.HIGH)

# The above step is to set the Resolution of the Video. The default is 640x480.
# This example works with a Resolution of 640x480.
next_step=0

    # Capture frame-by-frame       
    

def track_hoop(captuer):
        # load the image, clone it for output, and then convert it to grayscale
        global v1, v2,counter_comunication_on,counter_comunication_off
        
        output = captuer
        output = output[0:int(v1/2),0:v2]
                        
        blur = cv2.GaussianBlur(output, (5,5),0)



                # Convert BGR to HSV

        #hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)



                # Threshold the HSV image for only green colors
                
        lower_green = np.array([0, 71, 0]) #working fine      
        upper_green = np.array([121, 255, 9]) #working fine
        lower_green3 = np.array([0, 20,0])
        upper_green3 = np.array([0, 24, 12])
        lower_green2 = np.array([0, 0, 0]) #working fine      
        upper_green2 = np.array([102, 74, 24]) #working fine



                # Threshold the HSV image to get only green colors

        mask1 = cv2.inRange(blur, lower_green, upper_green)
        mask2 = cv2.inRange(blur, lower_green2, upper_green2)
        mask3 = cv2.inRange(blur, lower_green3, upper_green3)
        mask=mask1+mask3+mask2

                

                # Blur the mask
        bmask = cv2.GaussianBlur(mask, (5,5),0)
        canny = cv2.Canny(bmask,80,240,3)
        
        
                
                # detect circles in the image
        circles = cv2.HoughCircles(canny, cv2.HOUGH_GRADIENT, 1,v2, param1=50, param2=25, minRadius=50, maxRadius=0)

        crt=(-1) # if no circles found return (-1,-1)
                # ensure at least some circles were found
        if circles is not None:
                # when the curcle is ditacted
                
                        counter_comunication_off=0
                        counter_comunication_on+=1
                        if (counter_comunication_on==2 or counter_comunication_on==10 ):
                                ser.write(b'3')
##                        GPIO.output(arduino_stop, GPIO.LOW)
##                        GPIO.output(arduino_start, GPIO.HIGH)

                        
                        #print(max(circles))
                        # convert the (x, y) coordinates and radius of the circles to integers
                        circles=max(circles)
                        circles = np.round(circles[0, :]).astype("int")
                        #print(circles)
                        (x,y,r)=circles
                        crt= (x)
                        #for (x,y,r)in (circles):
                                                

                        # draw the circle in the output image, then draw a rectangle in the image
                        # corresponding to the center of the circle
                        cv2.circle(output, (x, y), r, (255, 0, 255), 4)
                        cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                                
            
                # Display the resulting frame
##        cv2.imshow('bmask',bmask)
##        cv2.imshow('frame',captuer)
##        cv2.imshow('canny',canny)
        #cv2.imshow('i',iiii)
        if cv2.waitKey(1) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            return 0
        return crt
# When everything done, release the capture
def reamp(captuer2):
    global v1, v2,counter_comunication_on,counter_comunication_off


    output = captuer2
    
    #output = output[int(v1/2): v1 ,0:v2]
                    
    blur = cv2.GaussianBlur(output, (5,5),0)



            # Convert BGR to HSV

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)



            # Threshold the HSV image for only green colors
            
    lower_blue = np.array([101, 50, 38])


    upper_blue = np.array([110, 255,255])



            # Threshold the HSV image to get only green colors
    #lower_blue2 = np.array([87, 30, 170])
    #upper_blue2 = np.array([103, 255,255])

    mask1 = cv2.inRange(hsv, lower_blue, upper_blue)
    #mask2 = cv2.inRange(hsv, lower_blue2, upper_blue2)
    mask=mask1#+mask2

            

            # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)
    canny = cv2.Canny(bmask,80,240,3)



    #contours

    canny2, contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    ctr = -1
    #print(len(contours))
   
    for i in range(0,len(contours)):

        #approximate the contour with accuracy proportional to

        #the contour perimeter

        approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.1,True)
        #print(len(approx))


        #Skip small or non-convex objects

        if(abs(cv2.contourArea(contours[i]))<100): #or not(cv2.isContourConvex(approx))):

            continue
        

        if len(contours) != 0:
            
            if(2<len(approx)<=4):
                counter_comunication_off=0
                counter_comunication_on+=1
                if (counter_comunication_on==1 or counter_comunication_on==10 ):
                        ser.write(b'3')

                #nb vertices of a polygonal curve

                #vtc = len(approx)


                #Use the degrees obtained above and the number of vertices

                #to determine the shape of the contour
                c = max(contours, key = cv2.contourArea)

                x,y,w,h = cv2.boundingRect(c)
                

                cv2.rectangle(output,(x,y),(x+w,y+h),(60,255,255),2)
                moments = cv2.moments(c)

                m00 = moments['m00']

                centroid_x, centroid_y = None, None

                if m00 != 0:

                        centroid_x = int(moments['m10']/m00)

                        centroid_y = int(moments['m01']/m00)



                    # Assume no centroid

                ctr = -1



                    # Use centroid if it exists

                if centroid_x != None and centroid_y != None:



                        ctr = centroid_x



                    # Put black circle in at centroid in image

                #cv2.circle(output, ctr, 20, (255,255,255),5)
                cv2.circle(output, (x, y), 20, (255, 0, 255), 4)


                cv2.putText(output,'RECT',(x,y),cv2.FONT_HERSHEY_SIMPLEX,2,(255,255,255),2,cv2.LINE_AA)

               
           



    #Display the resulting frame


    #cv2.imshow('frame2',captuer2)

    cv2.imshow('bmask2',bmask)
    #cv2.imshow('canny2',canny)

    if cv2.waitKey(1) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            return 0

    return ctr

def red_stantion(image):
    global v1, v2,counter_comunication_on,counter_comunication_off
    output = image
    #output = output[int(v1/2): v1 ,0:v2]
    # Blur the image to reduce noise

    blur = cv2.GaussianBlur(output, (5,5),0)



    # Convert BGR to HSV

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)



    # Threshold the HSV image for only red colors

    #lower_red = np.array([0,100,80])
    lower_red = np.array([0,70,50])

    upper_red = np.array([10,256,256])

    lower_red1 = np.array([170,100,100])

    upper_red1 = np.array([180,256,256])



    # Threshold the HSV image to get only red colors

    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask=mask1+mask2

    

    # Blur the mask

    bmask = cv2.GaussianBlur(mask, (5,5),0)
    canny2,contours, hierarchy = cv2.findContours(bmask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Assume no centroid
    ctr = (-1,-1)
    for i in range(0,len(contours)):
        approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.1,True)



            #Skip small or non-convex objects

        if(abs(cv2.contourArea(contours[i]))<100): #or not(cv2.isContourConvex(approx))):
            continue
        
        if len(contours) != 0:
            sides=len(approx)
            #print('sides',sides)
            #print('area',cv2.contourArea(contours[i]))
            if(2<sides<5):



                #Use the aspect ratio to identify the objecy
                x,y,w,h = cv2.boundingRect(contours[i])

                aspect_ratio = float((w)/h)
                print(aspect_ratio)
                #0.12396694
                if 0.1<aspect_ratio<1: # change the value according to the aspect_ratio
                    counter_comunication_off=0
                    counter_comunication_on+=1
                    if (counter_comunication_on==1 or counter_comunication_on==10 ):
                        ser.write(b'3')
                        
                    c = max(contours, key = cv2.contourArea)
                    
                    x,y,w,h = cv2.boundingRect(c)
    
                    cv2.rectangle(output,(x,y),(x+w,y+h),(0,0,255),2)
                    moments = cv2.moments(c)

                    m00 = moments['m00']

                    centroid_x, centroid_y = None, None

                    if m00 != 0:

                            centroid_x = int(moments['m10']/m00)

                            centroid_y = int(moments['m01']/m00)



                        # Assume no centroid

                    ctr = -1



                        # Use centroid if it exists

                    if centroid_x != None and centroid_y != None:



                            ctr = (centroid_x)



                        # Put black circle in at centroid in image

                    cv2.circle(output, (centroid_x,centroid_y), 10, (255,0,0),5)

                    cv2.putText(output,'stanchion',(x,y),cv2.FONT_HERSHEY_SIMPLEX,2,(255,0,255),2,cv2.LINE_AA)

               
           



    #cv2.imshow('Original',image)
    cv2.imshow('bmask_red',bmask)
    cv2.imshow('frame_red',output)
    if cv2.waitKey(1) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            return 0
    return ctr
def column_to_degree(y1,y2,x1,x2):
    #y1,y2 are max and min=90 degree of the servo
    m=(y2-y1)/(x2-x1)
    b=y1-(m*x1)
    return m,b

def track_math_comunication(image):
    global v2,v2mid,v2_p10,v2_n10,equation_left,equation_right
    column=image
    if column ==-1:
            return -1
    elif v2_n10 < column < v2_p10:
        y=int(90)
        print("fwd")
        return y
    elif column>v2_p10:
        y= int((equation_right[0]*column)+equation_right[1])
        
        print("right",y)
        return y
    elif column<v2_n10:
        y= int((equation_left[0]*column)+equation_left[1])
        
        print("left",y)
        return y
   # print(column)
def track_math_comunication_stantion(image):
    global v2,v2mid,v2_p10,v2_n10,equation_left,equation_right
    column=image
    if column ==-1:
            return -1
    elif v2_n10/2 < column < v2_p10/2:
        y=int(90)
        print("fwds_s",y)
        return y
    elif column>v2_p10/2:
        y= int((equation_right[0]*column)+equation_right[1])
        
        print("right_s",y)
        return y
    elif column<v2_n10/2:
        y= int((equation_left[0]*column)+equation_left[1])
        
        print("left_s",y)
        return y
    
#### main code
width=600
hight=260
camera = PiCamera()
camera.resolution = (width,hight)
camera.framerate =32
camera.awb_mode = 'auto' # off, auto, sunlight, cloudy, shade, tungsten, fluorescent, incandescent, flash, and horizon.
rawCapture = PiRGBArray(camera, size=(width,hight))
time.sleep(1)
while True:
        try:
                for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
                        image= frame.array
                        rawCapture.truncate(0)
                        while next_step==0:
                                v1,v2,v3 = image.shape
                                v1mid=int(v1/2)
                                v2mid=int(v2/2) # half of the column
                                v2_n10= v2mid-(v2*0.1)
                                v2_p10= v2mid+(v2*0.1)
                                equation_left= column_to_degree(90,115,v2mid,0)
                                equation_right= column_to_degree(90,80,v2mid,v2)
                                print(equation_right)
                                print(equation_left)
                                print(v2mid)
                                #GPIO.output(arduino_start, GPIO.HIGH)
                                #GPIO.output(arduino_stop, GPIO.HIGH)
                                counter_comunication_off=0
                                counter_comunication_on=0
                                next_step=1
                                
                                
                        

                        #track1=track_math_comunication(track_hoop(image))
                        track2=track_math_comunication(reamp(image))
                        track3=track_math_comunication_stantion(red_stantion(image))
                        arduino_comand=int(ser.readline())

                        if(arduino_comand==7):
                                ser.write(b'2')
                                time.sleep(0.01)
                        while(arduino_comand==7): 
                                arduino_comand=int(ser.readline())
                                
                        
                        if( track2 !=-1 or track3!=-1):
                                #GPIO.output(arduino_stop, GPIO.LOW)
                                #GPIO.output(arduino_start, GPIO.HIGH)
                                if (track2 !=-1):
                                        bus.write_byte(addr ,track_max)
                                        time.sleep(0.01)
                                else:
                                        track_max=max(track2,track3)
                                        bus.write_byte(addr ,track_max)
                                        time.sleep(0.01)
                        elif (track2 ==-1 and track3==-1 ):
                                #GPIO.output(arduino_stop, GPIO.HIGH)
                                #GPIO.output(arduino_start, GPIO.LOW)
                                counter_comunication_off+=1
                                counter_comunication_on=0
                                if (counter_comunication_off==10 or counter_comunication_off==20):
                                        time.sleep(1)
                                        ser.write(b'2')
                                time.sleep(0.01)
                                print(-1)
                        rawCapture.truncate(0)
        except OSError:
                print("Error check wire connection")
                #time.sleep(2)
                pass
        except TypeError:
                pass
        except ValueError:
                pass


        
  
image.release()
cv2.destroyAllWindows()
GPIO.cleanup()
