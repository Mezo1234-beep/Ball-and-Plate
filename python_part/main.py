import tkinter as tk
from PID import PID
from arduino import Arduino
from tracker import Tracker


class UI:

    def __init__(self):

        self.arduino = Arduino()
        self.pid = PID()
        self.tracker = Tracker(self)

        # --------- Setting Flags -------------------
        self.is_showing_video_window = False
        self.is_balincing_ball = False

        # ---------- Default values for PID ---------
        self.sliderCoefPDefault = 0.022
        self.sliderCoefIDefault = 0.0
        self.sliderCoefDDefault = 0.012

        # ---------- Init main window ----------------
        self.main_window = tk.Tk()
        self.main_window.title("Ball and Plate System -__-")
        self.main_window.geometry("430x400")
        self.main_window["bg"] = "gray15"
        self.main_window.resizable(0, 0)

        # ---------- Init video window ----------------

        self.video_window = tk.Toplevel(self.main_window)
        self.video_window.title("Camera")
        self.video_window.resizable(0, 0)

        self.lmain = tk.Label(self.video_window)
        self.lmain.pack()
        self.video_window.withdraw()

        self.init_gui()
        self.arduino.set_label(self.arduino_satatus_label)

        self.tracker.track()
        tk.mainloop()

    def init_gui(self):

        self.FramePIDCoef = tk.LabelFrame(self.main_window, text="PID Variables")
        self.FramePIDCoef.place(x=0, y=20, width=430)

        # -------- P Slider --------------------
        self.sliderCoefP = tk.Scale(self.FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="P", length=350,
                                    tickinterval=0.01,
                                    resolution=0.001)
        self.sliderCoefP.set(self.sliderCoefPDefault)
        self.sliderCoefP.pack()

        # -------- I Slider --------------------
        self.sliderCoefI = tk.Scale(self.FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="I", length=350,
                                    tickinterval=0.01,
                                    resolution=0.001)
        self.sliderCoefI.set(self.sliderCoefIDefault)
        self.sliderCoefI.pack()

        # -------- D Slider --------------------
        self.sliderCoefD = tk.Scale(self.FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="D", length=350,
                                    tickinterval=0.01,
                                    resolution=0.001)
        self.sliderCoefD.set(self.sliderCoefDDefault)
        self.sliderCoefD.pack()

        # -------- arduino status label shows the current state connected/unconnected --------------------
        self.arduino_satatus_label = tk.Label(self.main_window, text="Arduino disconnected  ", fg="red", anchor="ne")
        self.arduino_satatus_label.pack(fill="both")

        # -------- A button to connect with Arduoin board --------------------
        self.BConnect = tk.Button(self.main_window, text="Connect With Arduino", command=self.arduino.connectArduino,
                                  background="white")
        self.BConnect.place(x=20, y=300)

        # -------- A button to show the capture video --------------------
        self.BShowVideo = tk.Button(self.main_window, text="Open Camera", command=self.showCameraFrameWindow)
        self.BShowVideo.place(x=170, y=300)

        # -------- A button to start the balancing --------------------
        self.BStartBalance = tk.Button(self.main_window, text="Start", command=self.startBalance,
                                       highlightbackground="#36db8b")
        self.BStartBalance.place(x=270, y=300)

        # -------- A button to exit the pogram --------------------
        BQuit = tk.Button(self.main_window, text="Quit", command=self.end_program)
        BQuit.place(x=320, y=300)

        # -------- show developers names --------------------
        T = tk.Text(self.main_window, height=4, width=80)
        T.pack()
        T.place(x=0, y=350)
        T.insert(tk.END, " Created By: \n Moaaz-Alnouri, Ghaith-Alkholy and Nazir-Alkadi")

        # override protocol to have our customs behave which is --> closing video window not allowed.
        self.video_window.protocol("WM_DELETE_WINDOW", self.do_nothing)

    def showCameraFrameWindow(self):
        # if is_showing_video_window == False
        if not self.is_showing_video_window:
            self.video_window.deiconify()
            self.is_showing_video_window = True
            self.BShowVideo["text"] = "Hide Camera"
        else:
            self.video_window.withdraw()
            self.is_showing_video_window = False
            self.BShowVideo["text"] = "Open Camera"

    def end_program(self):
        self.main_window.destroy()

    def startBalance(self):
        if self.arduino.arduinoIsConnected:
            if not self.is_balincing_ball:
                self.is_balincing_ball = True
                self.BStartBalance["text"] = "Stop"
            else:
                self.is_balincing_ball = False
                self.BStartBalance["text"] = "Start"
        else:
            if tk.messagebox.askokcancel("Warning", "Hey you didn't connect with Arduino"):
                self.do_nothing()

    def do_nothing(self):
        pass


if __name__ == '__main__':
    program = UI()
