CAMERA_DEVICE_NAME = "camera"
DISPLAY_DEVICE_NAME = "display"
import cv2
import numpy as np

class ObjectDetector:
    def __init__(self, robot):
        # Receive Robot instance.
        self.robot = robot

        # get the time step of the current world.
        self.timestep = int(robot.getBasicTimeStep())

        # setup camera 
        self.camera = self.robot.getDevice(CAMERA_DEVICE_NAME)
        self.camera.enable(self.timestep)

        # setup display
        self.display = self.robot.getDevice(DISPLAY_DEVICE_NAME)
        self.display.attachCamera(self.camera)

    def detect(self):
        self.display.setAlpha(0.0)
        
        # Get the image from the camera
        img = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(),4))
        #using HSV color space to segment the image with mask
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        #[90,200,225]   [87,204,80]   [88,200,70]   [90,237,225]
        #x = [print(i) for i in img]

        #* definition of range of yellow
        lower = np.array([70,100,0])
        upper= np.array([90,255,255])
        mask = cv2.inRange(img, lower, upper)
        #x = [print(i) for i in mask]
        
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_center = cv2.moments(largest_contour)
            if largest_contour_center['m00'] > 0 :
                cx = int(largest_contour_center['m10']/largest_contour_center['m00'])
                cy = int(largest_contour_center['m01']/largest_contour_center['m00'])
            else: pass
            #bounding box - draw in the display 
            x, y, w, h = cv2.boundingRect(largest_contour)
            self.display.setAlpha(1.0)
            self.display.drawRectangle(x,y,w,h)
            
            d_left = x
            d_right = self.camera.width - (x + w)
                    
            if d_left == d_right :
                return True
        