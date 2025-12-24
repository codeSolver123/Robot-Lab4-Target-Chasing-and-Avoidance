# python 3 code
# ALex Anderson
# Tracking and avoidanceS

import socket
from time import *
from pynput import keyboard
"""pynput: On Mac OSX, one of the following must be true:
* The process must run as root. OR
* Your application must be white listed under Enable access for assistive devices. Note that this might require that you package your application, since otherwise the entire Python installation must be white listed."""
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = "192.168.1.106"    # SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 2 # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = True   # enable to allow robot movement

# Minimum size (in pixels) to be considered a valid object
MIN_OBJECT_AREA = 2200

# =====================================
# STATES
# =====================================
class States(enum.Enum):
    SEARCHING = enum.auto()
    LISTEN = enum.auto()
    TURN_LEFT = enum.auto()
    TURN_RIGHT = enum.auto()
    GO_FORWARD = enum.auto()
    GO_BACKWARD = enum.auto()


class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10                   # If its unable to connect after 10 seconds, give up.
        self.STATE = States.SEARCHING       # Start in searching mode
        self.RUNNING = True
        self.DIST = False
        self.foundOnce = False
        self.video = ImageProc()
        # Start video
        self.video.start()
       
        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection((self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)
   
        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        if ENABLE_ROBOT_CONNECTION:
            with socketLock:
                self.sock.sendall("i /dev/ttyUSB0".encode())
                print("Sent command")
                result = self.sock.recv(128)
                print(result)
                if result.decode() != "i /dev/ttyUSB0":
                    self.RUNNING = False
       
        self.sensors = Sensing(self.sock)
        # Start getting data
        if ENABLE_ROBOT_CONNECTION:
            self.sensors.start()
       
        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
           
    def run(self):
        # BEGINNING OF THE CONTROL LOOP
        while(self.RUNNING):
            sleep(0.1)

            # SEARCHING MODE
            if self.STATE == States.SEARCHING:
                if self.video.targetFound:
                    print("Target found — transitioning to tracking mode")
                    self.STATE = States.LISTEN
                    self.foundOnce = True
                else:
                    # spin slowly to search for target
                    if ENABLE_ROBOT_CONNECTION:
                        with socketLock:
                            print("Searching... spinning slowly")
                            self.sock.sendall("a spin_right(12)".encode())
                            self.sock.recv(128)
                continue

            # TRACKING LOGIC
            if self.video.targetFound:
                if self.video.red_obstacle_found: self.STATE = States.TURN_LEFT
                if self.video.yellow_obstacle_found: self.STATE = States.TURN_RIGHT
                cx, cy, area = self.video.targetCenter
                frame_center_x = self.video.frameCenter[0]
                frame_center_y = self.video.frameCenter[1]

                tol_x = 40
                tol_y = 40

                if abs(cx - frame_center_x) < tol_x:
                    if cy < frame_center_y - tol_y:
                        self.STATE = States.GO_FORWARD
                    elif cy > frame_center_y + tol_y:
                        self.STATE = States.GO_BACKWARD
                    else:
                        self.STATE = States.GO_FORWARD
                elif cx < frame_center_x - tol_x:
                    self.STATE = States.TURN_LEFT
                elif cx > frame_center_x + tol_x:
                    self.STATE = States.TURN_RIGHT
            else:
                # Lost target after finding it once
                if self.foundOnce:
                    print("Lost target — resuming search")
                    self.STATE = States.SEARCHING
                else:
                    self.STATE = States.SEARCHING

            # EXECUTE STATE ACTION
            if ENABLE_ROBOT_CONNECTION:
                self.execute_state()
       
        # END OF CONTROL LOOP
       
        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.video.RUNNING = False
       
        sleep(1)    # Wait for threads to wrap up
       
        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

    def execute_state(self):
        with socketLock:
            if self.STATE == States.TURN_LEFT:
                print("Turning left")
                self.sock.sendall("a spin_left(15)".encode())
                self.sock.recv(128)

            elif self.STATE == States.TURN_RIGHT:
                print("Turning right")
                self.sock.sendall("a spin_right(15)".encode())
                self.sock.recv(128)

            elif self.STATE == States.GO_FORWARD:
                print("Going forward")
                self.sock.sendall("a drive_straight(30)".encode())
                self.sock.recv(128)

            elif self.STATE == States.GO_BACKWARD:
                print("Going backward")
                self.sock.sendall("a drive_straight(-15)".encode())
                self.sock.recv(128)

            elif self.STATE == States.LISTEN:
                print("Holding position")
                self.sock.sendall("a drive_straight(0)".encode())
                self.sock.recv(128)

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'q':
                # Stop listener
                self.RUNNING = False
                self.sensors.RUNNING = False
                self.video.RUNNING = False
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
            return False

# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        self.RUNNING = True
        self.sock = socket
   
    def run(self):
        while self.RUNNING:
            sleep(1)
            # This is where I would get a sensor update
            with socketLock:
                self.sock.sendall("a distance".encode())
                print(self.sock.recv(128))


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.feedback = []
        self.feedback2 = []

        self.targetFound = False
        self.red_obstacle_found = False
        self.yellow_obstacle_found = False
        self.targetCenter = (0, 0, 0)
        self.targetCenterRED = (0, 0, 0)
        self.targetCenterYellow = (0, 0, 0)
        self.frameCenter = (160, 120)  # assuming 320x240

        self.thresholds = {'low_red':70,'high_red':255,'low_green':45,'high_green':255,'low_blue':30,'high_blue':100}#this is for the green
        
        self.yellowThresholds = {'low_red':119,'high_red':186,'low_green':95,'high_green':255,'low_blue':0,'high_blue':41}
        
        self.redThresholds = {'low_red':120,'high_red':255,'low_green':180,'high_green':255,'low_blue':0,'high_blue':20}

    def run(self):
        url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
        stream = urllib.request.urlopen(url)
        while(self.RUNNING):
            sleep(0.1)
            bytes = b''
            while self.RUNNING:
                bytes += stream.read(8192)  # image size is about 40k bytes
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a > b:
                    bytes = bytes[b+2:]
                    continue
                if a != -1 and b != -1:
                    jpg = bytes[a:b+2]
                    break
            img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8), cv2.IMREAD_COLOR)
            img = cv2.resize(img, ((int)(len(img[0]) / RESIZE_SCALE), (int)(len(img) / RESIZE_SCALE)))
           
            with imageLock:
                self.latestImg = copy.deepcopy(img)

            masked, masked2 = self.doImgProc()

            with imageLock:
                self.feedback = copy.deepcopy(masked)
                self.feedback2 = copy.deepcopy(masked2)

    def setThresh(self, name, value):
        self.thresholds[name] = value

    def doImgProc(self):
        # Use adjustable thresholds with HSV filtering
        low = (self.thresholds['low_blue'], self.thresholds['low_green'], self.thresholds['low_red'])
        high = (self.thresholds['high_blue'], self.thresholds['high_green'], self.thresholds['high_red'])

        lowYellow = (self.yellowThresholds['low_blue'], self.yellowThresholds['low_green'], self.yellowThresholds['low_red'])
        highYellow = (self.yellowThresholds['high_blue'], self.yellowThresholds['high_green'], self.yellowThresholds['high_red'])

        lowRed = (self.redThresholds['low_blue'], self.redThresholds['low_green'], self.redThresholds['low_red'])
        highRed = (self.redThresholds['high_blue'], self.redThresholds['high_green'], self.redThresholds['high_red'])
       
        hsvImg = cv2.cvtColor(self.latestImg, cv2.COLOR_BGR2HSV)

        theGreenMask = cv2.inRange(hsvImg, low, high)
        theYellowMask = cv2.inRange(hsvImg, lowYellow, highYellow)
        theRedMask = cv2.inRange(hsvImg, lowRed, highRed)
       
        # Create a kernel (structuring element)
        kernel = numpy.ones((5, 5), numpy.uint8)

        # Dilate the binary image
        theGreenMask = cv2.erode(theGreenMask, kernel, iterations=2)
        theGreenMask = cv2.dilate(theGreenMask, kernel, iterations=2)

        theYellowMask = cv2.erode(theYellowMask, kernel, iterations=2)
        theYellowMask = cv2.dilate(theYellowMask, kernel, iterations=2)

        theRedMask = cv2.erode(theRedMask, kernel, iterations=2)
        theRedMask = cv2.dilate(theRedMask, kernel, iterations=2)

        # Apply connected component analysis
        analysis = cv2.connectedComponentsWithStats(theGreenMask, 4, cv2.CV_32S)
        red_obstacle = cv2.connectedComponentsWithStats(theRedMask, 4, cv2.CV_32S)
        yellow_obstacle = cv2.connectedComponentsWithStats(theYellowMask, 4, cv2.CV_32S)
        (totalLabels, label_ids, stats, centroids) = analysis
        (totalLabelsRED, label_idsRED, statsRED, centroidsRED) = red_obstacle
        (totalLabelsYELLOW, labels_idYELLOW, statsYELLOW, centroidsYELLOW) = yellow_obstacle
        if totalLabelsRED > 1: #this is the red one
            maxAreaRED = 0
            bestIndexRED = -1
            for i in range(1, totalLabelsRED):
                areaRED = statsRED[i, cv2.CC_STAT_AREA]
                # Ignore objects smaller than minimum size
                if areaRED > maxAreaRED and areaRED >= MIN_OBJECT_AREA:
                    maxAreaRED = areaRED
                    bestIndexRED = i
            if bestIndexRED != -1:
                cxRED = int(centroidsRED[bestIndexRED][0])
                cyRED = int(centroidsRED[bestIndexRED][1])
                self.red_obstacle_found = True
                self.targetCenterRED = (cxRED, cyRED, maxAreaRED)
            else: self.red_obstacle_found = False
        else: self.red_obstacle_found = False
                #cv2.circle(theGreenMask, (cx, cy), 10, (255, 255, 255), 2)


        if totalLabelsYELLOW > 1: # this is the yellow one
            maxAreaYELLOW = 0
            bestIndexYELLOW = -1
            for i in range(1, totalLabelsYELLOW):
                areaYELLOW = statsYELLOW[i, cv2.CC_STAT_AREA]
                # Ignore objects smaller than minimum size
                if areaYELLOW > maxAreaYELLOW and areaYELLOW >= MIN_OBJECT_AREA:
                    maxAreaYELLOW = areaYELLOW
                    bestIndexYELLOW = i
            if bestIndexYELLOW != -1:
                cxYELLOW = int(centroidsYELLOW[bestIndexYELLOW][0])
                cyYELLOW = int(centroidsYELLOW[bestIndexYELLOW][1])
                self.yellow_obstacle_found = True
                self.targetCenterYELLOW = (cxYELLOW, cyYELLOW, maxAreaYELLOW)
            else: self.yellow_obstacle_found = False
        else: self.yellow_obstacle_found = False
                #cv2.circle(theGreenMask, (cx, cy), 10, (255, 255, 255), 2)

        if totalLabels > 1:
            maxArea = 0
            bestIndex = -1
            for i in range(1, totalLabels):
                area = stats[i, cv2.CC_STAT_AREA]
                # Ignore objects smaller than minimum size
                if area > maxArea and area >= MIN_OBJECT_AREA:
                    maxArea = area
                    bestIndex = i
            if bestIndex != -1:
                cx = int(centroids[bestIndex][0])
                cy = int(centroids[bestIndex][1])
                self.targetFound = True
                self.targetCenter = (cx, cy, maxArea)
                cv2.circle(theGreenMask, (cx, cy), 10, (255, 255, 255), 2)
            else:
                self.targetFound = False
        else:
            self.targetFound = False

        return cv2.bitwise_and(self.latestImg, self.latestImg, mask=theRedMask), cv2.bitwise_and(self.latestImg, self.latestImg, mask=theRedMask)

# END OF IMAGEPROC


if __name__ == "__main__":
   
    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)
   
    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)

    cv2.namedWindow('w2')
    cv2.moveWindow('w2', 100, 310)
   
    sm = StateMachine()
    sm.start()
   
    # Probably safer to do this on the main thread rather than in ImgProc init
    cv2.createTrackbar('low_red', 'sliders', sm.video.thresholds['low_red'], 255,
                      lambda x: sm.video.setThresh('low_red', x) )
    cv2.createTrackbar('high_red', 'sliders', sm.video.thresholds['high_red'], 255,
                     lambda x: sm.video.setThresh('high_red', x) )
   
    cv2.createTrackbar('low_green', 'sliders', sm.video.thresholds['low_green'], 255,
                      lambda x: sm.video.setThresh('low_green', x) )
    cv2.createTrackbar('high_green', 'sliders', sm.video.thresholds['high_green'], 255,
                     lambda x: sm.video.setThresh('high_green', x) )
   
    cv2.createTrackbar('low_blue', 'sliders', sm.video.thresholds['low_blue'], 255,
                      lambda x: sm.video.setThresh('low_blue', x) )
    cv2.createTrackbar('high_blue', 'sliders', sm.video.thresholds['high_blue'], 255,
                     lambda x: sm.video.setThresh('high_blue', x) )

    while len(sm.video.latestImg) == 0 or len(sm.video.feedback) == 0:
        sleep(1)

    while(sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View", sm.video.latestImg)
            cv2.imshow("sliders", sm.video.feedback)
            cv2.imshow("w2", sm.video.feedback2)
        cv2.waitKey(5)

    cv2.destroyAllWindows()
    sleep(1)
