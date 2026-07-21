"""Camera identifier GUI: pick a /dev/video* from the dropdown, see its feed.

    pixi run campick

Wave at a camera to find out which device it is, then set WRIST_DEV/EXT_DEV
for `pixi run cams` accordingly. Title bar shows resolution + brightness
(a stuck-black camera reads ~0).
"""

import glob
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk


class Picker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("camera picker")
        devs = sorted(glob.glob("/dev/video*"))
        self.combo = ttk.Combobox(self.root, values=devs, state="readonly")
        self.combo.pack(fill="x", padx=4, pady=4)
        self.combo.bind("<<ComboboxSelected>>", self.select)
        self.label = tk.Label(self.root, text="pick a device", width=80,
                              height=30, bg="black", fg="white")
        self.label.pack()
        self.cap = None
        self.root.after(33, self.tick)

    def select(self, _=None):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.combo.get())
        if not self.cap.isOpened():
            self.label.config(image="", text="cannot open (metadata node?)")
            self.cap = None

    def tick(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            if ok:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(rgb))
                self.label.config(image=img, text="", width=rgb.shape[1],
                                  height=rgb.shape[0])
                self.label.img = img  # keep a ref or tk drops the frame
                h, w = frame.shape[:2]
                self.root.title(
                    f"{self.combo.get()}  {w}x{h}  "
                    f"brightness {frame.mean():.0f}")
            else:
                self.label.config(image="", text="no frames")
        self.root.after(33, self.tick)

    def run(self):
        self.root.mainloop()
        if self.cap is not None:
            self.cap.release()


if __name__ == "__main__":
    Picker().run()
