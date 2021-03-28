import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *

lines = open("data.txt").read().splitlines()
max_alpha, max_theta = lines[1].split("|")
max_alpha = - float(max_alpha)
max_theta = float(max_theta)

dataDict = {}

camHeight = 480
camWidth = 640
cam = cv2.VideoCapture(1)
cam.set(3, camWidth)
cam.set(4, camHeight)

for i in range(1, len(lines)):
    alpha, theta = lines[i].split("|")
    dataDict[float(alpha)] = float(theta)

controllerWindow = tk.Tk()
controllerWindow.title("Ball and Plate System -__-")
controllerWindow.geometry("430x400")
controllerWindow["bg"] = "gray15"
controllerWindow.resizable(0, 0)

videoWindow = tk.Toplevel(controllerWindow)
videoWindow.title("Camera")
videoWindow.resizable(0, 0)
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

showVideoWindow = False


def showCameraFrameWindow():
    global showVideoWindow
    if showVideoWindow == False:
        videoWindow.deiconify()
        showVideoWindow = True
        BShowVideo["text"] = "Hide Camera"
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BShowVideo["text"] = "Open Camera"


# t = 500
refY = 240
refX = 240
Ts = 0


def endProgam():
    controllerWindow.destroy()


sliderCoefPDefault = 0.022
sliderCoefIDefault = 0.0
sliderCoefDDefault = 0.012


def donothing():
    pass


arduinoIsConnected = False


def connectArduino():
    global ser
    global label
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" or "CH340" in p.description:
            print(p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1)
            label.configure(text="Arduino connecte", fg="#36db8b")
            arduinoIsConnected = True


startBalanceBall = False


def startBalance():
    global startBalanceBall
    if arduinoIsConnected == True:
        if startBalanceBall == False:
            startBalanceBall = True
            BStartBalance["text"] = "Stop"
        else:
            startBalanceBall = False
            BStartBalance["text"] = "Start"
    else:
        if tkinter.messagebox.askokcancel("Warning", "Hey you didnt connect with ardunio"):
            donothing()


totalErrorX = 0
totalErrorY = 0
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0

N = 20
prevDerivX = 0
prevDerivY = 0
prevIntegX = 0
prevIntegY = 0
delivery_time = 0
prevErrorX = 0
prevErrorY = 0
x, y = 0, 0


def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, refX, refY):  # PID controller
    global totalErrorX, totalErrorY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected
    global Ts, delivery_time, N
    global prevDerivX, prevDerivY, prevIntegX, prevIntegY
    global prevErrorX, prevErrorY

    Ts = time.time() - delivery_time  # sampling time
    delivery_time = time.time()
    print("Ts", Ts)

    errorX = refX - ballPosX
    errorY = refY - ballPosY

    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    try:
        derivX = (prevBallPosX - ballPosX) / Ts
    except ZeroDivisionError:
        derivX = 0

    try:
        derivY = (prevBallPosY - ballPosY) / Ts
    except ZeroDivisionError:
        derivY = 0
    print("erorrY", totalErrorY)
    Cix = Ki * totalErrorX  # prevIntegX + errorX*Ki*Ts                    #Ki * totalErrorX
    Ciy = Ki * totalErrorY  # prevIntegY + errorY*Ki*Ts                    #Ki * totalErrorX

    Cdx = Ts / (1 + N * Ts) * (
                N * Kd * derivX + prevDerivX / Ts)  # (Kd*N*(errorX-prevErrorX)+prevDerivX)/(1+N*Ts)# #Kd * ((errorX - prevErrorX)/Ts)
    Cdy = Ts / (1 + N * Ts) * (
                N * Kd * derivY + prevDerivY / Ts)  # (Kd*N*(errorY-prevErrorY)+prevDerivY)/(1+N*Ts) # #Kd * ((errorY - prevErrorY)/Ts)
    print("Cdx: ", Cdx)
    print("Cix: ", Cix)
    print("derivX: ", derivX)
    print("prevDerivX: ", prevDerivX)
    Ix = Kp * errorX + Cix + Cdx
    Iy = Kp * errorY + Ciy + Cdy
    print("Kp: ",Kp)
    print("Ix: ",Ix)
    # Ix = Kp * (refX - ballPosX)
    # Iy = Kp * (refX - ballPosY)

    Ix = round(Ix, 1)
    Iy = round(Iy, 1)
    print("Ix round: " , Ix)

    if Ix > max_alpha:
        Ix = max_alpha
    elif Ix < - max_alpha:
        Ix = - max_alpha
    if Iy > max_alpha:
        Iy = max_alpha
    elif Iy < - max_alpha:
        Iy = - max_alpha
    print("Ix after IF: ", Ix)
    print("totalErorrX", totalErrorX)

    if arduinoIsConnected == True and startBalanceBall == True:
        ser.write((str(dataDict[Ix]) + "," + str(dataDict[-Iy]) + "\n").encode())

    if startBalanceBall == True:
        prevDerivX = Cdx
        prevDerivY = Cdy
        prevIntegX = Cix
        prevIntegY = Ciy
        prevErrorX = errorX
        prevErrorY = errorY


prevX, prevY = 0, 0
prevRefX, prevRefY = 0, 0
start_time = 0


def main():
    start_timeFPS = time.time()
    global x, y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevRefX, prevRefY
    global refX, refY, totalErrorX, totalErrorY
    global camWidth, camHeight
    global timeInterval, start_time
    global showVideoWindow

    _, img = cam.read()
    img = img[0:int(camHeight),
          int((camWidth - camHeight) / 2):int(camWidth - ((camWidth - camHeight) / 2))]  # [Y1:Y2,X1:X2]
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lowerblue = np.array([45, 100, 50])
    appergreen = np.array([75, 255, 255])


    mask = cv2.inRange(imgHSV, lowerblue, appergreen)
    mask = cv2.blur(mask, (6, 6))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[0]
    center = None

    cv2.circle(img, (int(refX), int(refY)), int(4), (255, 0, 0), 2)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        timeInterval = time.time() - start_time
        (x, y), radius = cv2.minEnclosingCircle(c)
        if radius > 10:
            cv2.putText(img, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            PIDcontrol(int(x), int(y), prevX, prevY, refX, refY)
            start_time = time.time()
    else:
        totalErrorX, totalErrorY = 0, 0

    if showVideoWindow == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    lmain.after(5, main)

    prevX, prevY = int(x), int(y)
    prevRefX, prevRefY = refX, refY
    prevAlpha = alpha
    prevBeta = beta


FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID Variabless")
FramePIDCoef.place(x=0, y=20, width=430)

sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="P", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="I", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="D", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()

label = tk.Label(controllerWindow, text="Arduino disconnected  ", fg="red", anchor="ne")
label.pack(fill="both")

BConnect = tk.Button(controllerWindow, text="Connect With Ardunio", command=connectArduino, background="white")
BConnect.place(x=20, y=300)

BShowVideo = tk.Button(controllerWindow, text="Open Camera", command=showCameraFrameWindow)
BShowVideo.place(x=170, y=300)

BStartBalance = tk.Button(controllerWindow, text="Start", command=startBalance, highlightbackground="#36db8b")
BStartBalance.place(x=270, y=300)

BQuit = tk.Button(controllerWindow, text="Quit", command=endProgam)
BQuit.place(x=320, y=300)

T = tk.Text(controllerWindow, height=4, width=80)
T.pack()
T.place(x=0, y=350)
T.insert(tk.END, "                  Created By\n Moaaz Alnouri , Nazir AlKadi & Wassem Balloul,\n                Ghaith AlKhole")

videoWindow.protocol("WM_DELETE_WINDOW", donothing)

main()
tk.mainloop()
