"""Camera identifier GUI: pick a /dev/video* from the dropdown, see its feed.

    pixi run campick

Wave at a camera to find out which device it is, then set WRIST_DEV/EXT_DEV
for `pixi run cams` accordingly. Title bar shows resolution + brightness
(a stuck-black camera reads ~0). The feed scales with the window, aspect
preserved - maximize away.
"""

import glob
import tkinter as tk
from tkinter import font, ttk

import cv2
from PIL import Image, ImageTk


class Picker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("camera picker")
        self.root.geometry("900x640")
        ui_font = font.nametofont("TkDefaultFont")
        ui_font.configure(size=13)
        style = ttk.Style()
        style.configure("TCombobox", padding=6)

        devs = sorted(glob.glob("/dev/video*"))
        self.combo = ttk.Combobox(self.root, values=devs, state="readonly",
                                  font=ui_font)
        self.combo.pack(fill="x", padx=6, pady=6)
        self.combo.bind("<<ComboboxSelected>>", self.select)
        # the feed fills all remaining space and grows with the window
        self.label = tk.Label(self.root, text="pick a device",
                              bg="black", fg="white", font=ui_font)
        self.label.pack(fill="both", expand=True)
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
                h, w = frame.shape[:2]
                # scale to fit the label, preserve aspect
                lw = max(self.label.winfo_width(), 32)
                lh = max(self.label.winfo_height(), 32)
                s = min(lw / w, lh / h)
                frame_fit = cv2.resize(frame, (int(w * s), int(h * s)),
                                       interpolation=cv2.INTER_NEAREST)
                rgb = cv2.cvtColor(frame_fit, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(rgb))
                self.label.config(image=img, text="")
                self.label.img = img  # keep a ref or tk drops the frame
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
