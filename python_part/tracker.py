import cv2
import time
import imutils
from PIL import Image, ImageTk
import numpy as np


class Tracker:

    def __init__(self, ui):
        self.arduino = ui.arduino
        self.ui = ui
        self.pid_controller = PID(self.ui)

        # ---------- Camera setup ---------
        self.camHeight = 480
        self.camWidth = 640
        self.cam = cv2.VideoCapture(1)
        self.cam.set(3, self.camWidth)
        self.cam.set(4, self.camHeight)

        self.refY = 240
        self.refX = 240
        self.prevX, self.prevY = 0, 0
        self.alpha, self.beta, self.prevAlpha, self.prevBeta = 0, 0, 0, 0
        self.x, self.y = 0, 0

        self.start_time = 0  # useless
        self.init_angels()

    def track(self):
        # ---- RGB range of green color -----
        lowerblue = np.array([45, 100, 50])
        appergreen = np.array([75, 255, 255])

        # ---- read frame from camera ------
        _, img = self.cam.read()
        img = img[0:int(self.camHeight), int((self.camWidth - self.camHeight) / 2):int(
            self.camWidth - ((self.camWidth - self.camHeight) / 2))]  # [Y1:Y2,X1:X2]

        # ---- converting frame into HSV ------
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ---- creating mask of previous frame by taking green colors only -----
        mask = cv2.inRange(imgHSV, lowerblue, appergreen)
        mask = cv2.blur(mask, (6, 6))  # add Blur
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # ---- Finding contours of the ball ----------------
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[0]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            (self.x, self.y), radius = cv2.minEnclosingCircle(c)

            if radius > 10:
                self.draw_bounding_circle(img, radius)
                self.pid_controller.pid_controller(int(self.x), int(self.y), self.prevX, self.prevY, self.refX,
                                                   self.refY)
                self.start_time = time.time()

        # ---- Draw the reference (center) point ---------------
        cv2.circle(img, (int(self.refX), int(self.refY)), int(4), (255, 0, 0), 2)

        # ---- Displaying the frame ----------------------------
        if self.ui.is_showing_video_window:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            self.ui.lmain.imgtk = imgtk
            self.ui.lmain.configure(image=imgtk)

        # ---- read the next frame -----
        self.ui.lmain.after(5, self.track)

        # ---- store values ------------
        self.prevX, self.prevY = int(self.x), int(self.y)
        self.prevRefX, self.prevRefY = self.refX, self.refY
        self.prevAlpha = self.alpha
        self.prevBeta = self.beta

    def draw_bounding_circle(self, img, radius):
        # put a coordination as a text above the circle
        cv2.putText(img, str(int(self.x)) + ";" + str(int(self.y)).format(0, 0), (int(self.x) - 50, int(self.y) - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # draw a yellow circle around the ball
        cv2.circle(img, (int(self.x), int(self.y)), int(radius), (0, 255, 255), 2)

    def init_angels(self) -> dict:
        lines = open("data.txt").read().splitlines()
        self.max_alpha, self.max_theta = lines[1].split("|")
        self.max_alpha = - float(self.max_alpha)
        self.max_theta = float(self.max_theta)

        self.dataDict = {}

        for i in range(1, len(lines)):
            alpha, theta = lines[i].split("|")
            self.dataDict[float(alpha)] = float(theta)

        return self.dataDict


class PID:

    def __init__(self, ui):
        self.ui = ui
        self.arduino = ui.arduino
        self.N = 20
        self.Ts = 0
        self.prevDerivX = 0
        self.prevDerivY = 0
        self.prevIntegX = 0
        self.prevIntegY = 0
        self.delivery_time = 0
        self.prevErrorX = 0
        self.prevErrorY = 0

        self.Ki = 0.022
        self.Kp = 0
        self.Kd = 0.012

        self.init_angels()

    # -------------- PID controller ------------------

    def pid_controller(self, ballPosX, ballPosY, prevBallPosX, prevBallPosY, refX, refY):

        self.Ts = time.time() - self.delivery_time  # sampling time
        self.delivery_time = time.time()

        # update PID values from UI sliders
        self.update_pid_values()

        # calculate the error from the reference point
        errorX = refX - ballPosX
        errorY = refY - ballPosY

        deriveX = self.get_derive(prevBallPosX, ballPosX)
        deriveY = self.get_derive(prevBallPosY, ballPosY)

        Cix = 0
        Ciy = 0

        Cdx = self.Ts / (1 + self.N * self.Ts) * (self.N * self.Kd * deriveX + self.prevDerivX / self.Ts)
        Cdy = self.Ts / (1 + self.N * self.Ts) * (self.N * self.Kd * deriveY + self.prevDerivY / self.Ts)
        Ix = self.Kp * errorX + Cix + Cdx
        Iy = self.Kp * errorY + Ciy + Cdy

        Ix, Iy = round(Ix, 1), round(Iy, 1)
        Ix, Iy = self.validate(Ix, Iy)

        # ---------------- send the calculated angel to Arduino ----------
        if self.arduino.arduinoIsConnected and self.ui.is_balincing_ball:
            self.arduino.ser.write((str(self.dataDict[Ix]) + "," + str(self.dataDict[-Iy]) + "\n").encode())

        # ---------------- Store current values as previous----------
        if self.ui.is_balincing_ball:
            self.prevDerivX = Cdx
            self.prevDerivY = Cdy
            self.prevIntegX = Cix
            self.prevIntegY = Ciy
            self.prevErrorX = errorX
            self.prevErrorY = errorY

    def validate(self, Ix, Iy):
        """
         A method to ensure the angle is not more than the accepted value
        """
        if Ix > self.max_alpha:
            Ix = self.max_alpha
        elif Ix < - self.max_alpha:
            Ix = - self.max_alpha
        else:
            Ix = Ix

        if Iy > self.max_alpha:
            Iy = self.max_alpha
        elif Iy < - self.max_alpha:
            Iy = - self.max_alpha
        else:
            Iy = Iy

        return Ix, Iy

    def get_derive(self, prev_pos, ball_pos):
        try:
            derive = (prev_pos - ball_pos) / self.Ts
        except ZeroDivisionError:
            derive = 0

        return derive

    def update_pid_values(self):
        self.Kp = self.ui.sliderCoefP.get()
        self.Ki = self.ui.sliderCoefI.get()
        self.Kd = self.ui.sliderCoefD.get()

    # -------------- Reading data --------------------

    def init_angels(self) -> dict:
        lines = open("data.txt").read().splitlines()
        self.max_alpha, self.max_theta = lines[1].split("|")
        self.max_alpha = - float(self.max_alpha)
        self.max_theta = float(self.max_theta)

        self.dataDict = {}

        for i in range(1, len(lines)):
            alpha, theta = lines[i].split("|")
            self.dataDict[float(alpha)] = float(theta)

        return self.dataDict
