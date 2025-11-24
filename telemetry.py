import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import json
import time


# ======================================================
# TELEMETRY READER THREAD
# ======================================================
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = out_queue
        self.running = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
        except Exception as e:
            self.q.put(("error", str(e)))
            return

        while self.running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self.q.put(("data", line))
            except Exception as e:
                self.q.put(("error", str(e)))
                break

        if self.ser:
            self.ser.close()

    def stop(self):
        self.running = False


# ======================================================
# GUI APPLICATION
# ======================================================
class TelemetryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Telemetry GUI")
        self.root.geometry("450x350")

        self.reader = None
        self.queue = queue.Queue()

        # ================================
        # PORT SELECTION UI
        # ================================
        ttk.Label(root, text="Select Serial Port").pack(pady=5)

        self.port_combo = ttk.Combobox(root, values=self.list_ports(), state="readonly")
        self.port_combo.pack(pady=5)

        ttk.Label(root, text="Baud Rate").pack(pady=5)

        self.baud_combo = ttk.Combobox(root, values=["9600", "57600", "115200"], state="readonly")
        self.baud_combo.current(2)
        self.baud_combo.pack(pady=5)

        # Start/Stop Buttons
        self.start_button = ttk.Button(root, text="Start Telemetry", command=self.start)
        self.start_button.pack(pady=10)

        self.stop_button = ttk.Button(root, text="Stop", command=self.stop, state="disabled")
        self.stop_button.pack(pady=5)

        # ================================
        # TELEMETRY OUTPUT BOX
        # ================================
        ttk.Label(root, text="Incoming Data").pack(pady=5)

        self.text_box = tk.Text(root, height=10, width=50)
        self.text_box.pack(pady=5)

        # Periodic queue checking
        self.root.after(50, self.process_queue)

    # ==================================================
    # HELPERS
    # ==================================================
    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def log(self, msg):
        self.text_box.insert(tk.END, msg + "\n")
        self.text_box.see(tk.END)

    # ==================================================
    # START TELEMETRY
    # ==================================================
    def start(self):
        port = self.port_combo.get()
        baud = self.baud_combo.get()

        if not port:
            messagebox.showerror("Error", "Please select a serial port.")
            return

        try:
            baud = int(baud)
        except:
            messagebox.showerror("Error", "Invalid baud rate.")
            return

        self.reader = SerialReader(port, baud, self.queue)
        self.reader.start()

        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

        self.log(f"[INFO] Started telemetry on {port} @ {baud}")

    # ==================================================
    # STOP TELEMETRY
    # ==================================================
    def stop(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
            self.log("[INFO] Telemetry stopped.")

        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")

    # ==================================================
    # PROCESS INCOMING MESSAGES
    # ==================================================
    def process_queue(self):
        while not self.queue.empty():
            msg_type, content = self.queue.get()

            if msg_type == "data":
                self.log(f"{content}")

            elif msg_type == "error":
                self.log(f"[ERROR] {content}")
                self.stop()

        self.root.after(50, self.process_queue)


# ======================================================
# MAIN ENTRY POINT
# ======================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
