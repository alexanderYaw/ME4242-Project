import tkinter as tk
from tkinter import ttk

import cv2 as cv
from PIL import ImageTk, Image

"""
    This class is used to create a camera feed window that displays the video feed from a camera.
    It runs in a separate thread and can be started and stopped using the start() and stop() methods.
    The camera feed is displayed in a window named 'mask'.
    The class uses OpenCV to create the window and display the video feed.
    The camera feed is passed to the class as a frame parameter.
"""	
class CameraFeedWindow():
    windowCount = 0
    def __init__(self, rootWindow, name=f"Camera Feed Window", trackbarParams={}, on_change=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        CameraFeedWindow.windowCount += 1
        self.rootWindow = rootWindow
        self.name = name
        self.running = True
        self.window = tk.Toplevel(self.rootWindow)
        self.image = None

        self.window.title(self.name)
        # self.window.geometry("640x480")
        self.window.resizable(False, False)
        self.canvas = tk.Canvas(self.window, width=640, height=480)
        self.canvasImage = self.canvas.create_image(320, 240, anchor="center")
        self.canvas.pack()

        self.default_param = trackbarParams.copy()
        self.trackbarParams = trackbarParams
        self.trackbars = {}
        if self.trackbarParams:
            for key, _ in self.trackbarParams.items():
                self.trackbars[key] = ttk.Scale(self.window, from_=0, to=255, orient='horizontal')
                self.trackbars[key].set(self.default_param[key])
                self.trackbars[key].configure(command = self.on_change if on_change is None else on_change)
                ttk.Label(self.window, text=key).pack()
                self.trackbars[key].pack(fill="x", padx=5)
            self.resetButton = ttk.Button(self.window, text="Reset From File", command=self.reset)
            self.resetButton.pack(pady=5)

        self.window.protocol("WM_DELETE_WINDOW", self.closed)

    def closed(self):
        CameraFeedWindow.windowCount -= 1
        self.window.destroy()
        self.running = False

    def on_change(self, value):
        for key, _ in self.trackbarParams.items():
            self.trackbarParams[key] = int(self.trackbars[key].get())

    def reset(self):
        for key, val in self.trackbarParams.items():
            self.trackbars[key].set(self.default_param[key])
            self.trackbarParams[key] = self.default_param[key]

    def set_frame(self, frame):
        # Convert the frame to a format that can be displayed in the Tkinter window
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame = cv.resize(frame, (640, 480))
        if self.rootWindow and self.window and self.window.winfo_exists():
            self.image = ImageTk.PhotoImage(image=Image.fromarray(frame))
            self.canvas.imgref = self.image
            self.canvas.itemconfig(self.canvasImage, image=self.image)