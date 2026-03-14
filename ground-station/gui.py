import tkinter as tk
from tkinter import messagebox
import ttkbootstrap as tb
from ttkbootstrap.constants import *
import serial
import serial.tools.list_ports
import threading
import json
import csv
import time
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Line3DCollection

def get_lander_lines():
    lines = []
    colors = []
    
    def add_line(p1, p2, color):
        lines.append([p1, p2])
        colors.append(color)

    for theta in np.linspace(0, 2*np.pi, 12, endpoint=False):
        add_line([np.cos(theta), np.sin(theta), -3], [np.cos(theta), np.sin(theta), 3], '#808000')
    for z in [-3, 3]:
        pts = [[np.cos(t), np.sin(t), z] for t in np.linspace(0, 2*np.pi, 24)]
        for i in range(len(pts)-1):
            add_line(pts[i], pts[i+1], '#808000')
            
    for theta in np.linspace(0, 2*np.pi, 8, endpoint=False):
        add_line([np.cos(theta), np.sin(theta), 3], [0.5*np.cos(theta), 0.5*np.sin(theta), 5], 'white')
        add_line([0.5*np.cos(theta), 0.5*np.sin(theta), 5], [0, 0, 6], 'white')
    pts = [[0.5*np.cos(t), 0.5*np.sin(t), 5] for t in np.linspace(0, 2*np.pi, 24)]
    for i in range(len(pts)-1):
        add_line(pts[i], pts[i+1], 'white')

    for theta in np.linspace(0, 2*np.pi, 12, endpoint=False):
        add_line([1.2*np.cos(theta), 1.2*np.sin(theta), -3], [1.2*np.cos(theta), 1.2*np.sin(theta), -2], 'white')
    for z in [-3, -2]:
        pts = [[1.2*np.cos(t), 1.2*np.sin(t), z] for t in np.linspace(0, 2*np.pi, 24)]
        for i in range(len(pts)-1):
            add_line(pts[i], pts[i+1], 'white')

    for theta in [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]:
        top = [1.2*np.cos(theta), 1.2*np.sin(theta), -2]
        bot = [3.5*np.cos(theta), 3.5*np.sin(theta), -6]
        mid = [np.cos(theta), np.sin(theta), -3]
        add_line(top, bot, 'white')
        add_line(mid, bot, 'white')
        add_line([bot[0]-0.5, bot[1], bot[2]], [bot[0]+0.5, bot[1], bot[2]], 'white')
        add_line([bot[0], bot[1]-0.5, bot[2]], [bot[0], bot[1]+0.5, bot[2]], 'white')

    for theta in np.linspace(0, 2*np.pi, 8, endpoint=False):
        add_line([0.8*np.cos(theta), 0.8*np.sin(theta), -3], [0.6*np.cos(theta), 0.6*np.sin(theta), -4], '#c0C0C0')
    
    for theta in np.linspace(0, 2*np.pi, 4, endpoint=False):
        add_line([0.6*np.cos(theta), 0.6*np.sin(theta), -4], [0, 0, -8], '#fF4500')

    return np.array(lines), colors


class GroundStationUI:
    def __init__(self, root):
        self.root = root
        
        self.serial_port = None
        self.is_reading = False
        
        self.csv_filename = f"telemetry_log_{int(time.time())}.csv"
        self.csv_file = None
        self.csv_writer = None
        self.init_csv()
        
        plt.style.use('dark_background')
        #tweak plot styling to match the themes
        plt.rcParams['figure.facecolor'] = '#222222'
        plt.rcParams['axes.facecolor'] = '#222222'
        
        self.lander_lines_base, self.lander_colors = get_lander_lines()
        self.pos_history_x = []
        self.pos_history_y = []
        self.pos_history_z = []
        self.last_draw_time = 0
        self.draw_interval = 0.1
        
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.last_px = 0.0
        self.last_py = 0.0
        self.last_pz = 0.0
        
        self.create_widgets()
        
    def init_csv(self):
        file_exists = os.path.isfile(self.csv_filename)
        self.csv_file = open(self.csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            self.csv_writer.writerow([
                "timestamp", "seq", "rssi", "success_rate", "lost",
                "roll", "pitch", "yaw",
                "ax", "ay", "az",
                "px", "py", "pz", "pressure"
            ])
            self.csv_file.flush()

    def create_widgets(self):
        top_frame = tb.Frame(self.root, padding=15)
        top_frame.pack(fill=X)
        
        title_lbl = tb.Label(top_frame, text="PROJECT LANDER DASHBOARD", font=("Helvetica", 24, "bold"), bootstyle=LIGHT)
        title_lbl.pack(side=LEFT, padx=(0, 20))
        
        conn_frame = tb.Frame(top_frame)
        conn_frame.pack(side=RIGHT)
        
        tb.Label(conn_frame, text="COM Port:", font=("Helvetica", 12)).pack(side=LEFT, padx=5)
        self.port_var = tk.StringVar()
        self.port_cb = tb.Combobox(conn_frame, textvariable=self.port_var, width=15, font=("Helvetica", 11))
        self.port_cb.pack(side=LEFT, padx=5)
        self.refresh_ports()
        
        tb.Button(conn_frame, text="Refresh", command=self.refresh_ports, bootstyle=(SECONDARY, OUTLINE)).pack(side=LEFT, padx=5)
        self.connect_btn = tb.Button(conn_frame, text="Connect", command=self.toggle_connection, bootstyle=SUCCESS)
        self.connect_btn.pack(side=LEFT, padx=5)
        self.reset_btn = tb.Button(conn_frame, text="Zero Graph Output", command=self.zero_origin, bootstyle=(WARNING, OUTLINE))
        self.reset_btn.pack(side=LEFT, padx=5)
        
        tb.Separator(self.root, bootstyle=SECONDARY).pack(fill=X, padx=15, pady=5)
        
        data_frame = tb.Frame(self.root, padding=10)
        data_frame.pack(fill=X)
        
        #stats Card
        stats_card = tb.Labelframe(data_frame, text=" Network Analytics ", bootstyle=INFO, padding=15)
        stats_card.pack(side=LEFT, fill=BOTH, expand=True, padx=(5, 10))
        
        self.lbl_success = tb.Label(stats_card, text="Success Rate: -- %", font=("Helvetica", 16))
        self.lbl_success.pack(pady=5, anchor=W)
        self.lbl_lost = tb.Label(stats_card, text="Packets Lost: --", font=("Helvetica", 16))
        self.lbl_lost.pack(pady=5, anchor=W)
        self.lbl_rssi = tb.Label(stats_card, text="RSSI: -- dBm", font=("Helvetica", 16))
        self.lbl_rssi.pack(pady=5, anchor=W)
        self.lbl_bw = tb.Label(stats_card, text="Bandwidth: -- kbps", font=("Helvetica", 14), bootstyle=SECONDARY)
        self.lbl_bw.pack(pady=5, anchor=W)

        #telemetry Card
        telem_card = tb.Labelframe(data_frame, text=" Live Telemetry ", bootstyle=SUCCESS, padding=15)
        telem_card.pack(side=RIGHT, fill=BOTH, expand=True, padx=(10, 5))
        
        self.lbl_seq = tb.Label(telem_card, text="Sequence: --", font=("Courier", 14, "bold"), bootstyle=LIGHT)
        self.lbl_seq.pack(anchor=W, pady=3)
        self.lbl_euler = tb.Label(telem_card, text="Euler (R/P/Y):  -- / -- / --", font=("Courier", 14))
        self.lbl_euler.pack(anchor=W, pady=3)
        self.lbl_accel = tb.Label(telem_card, text="Accel (X/Y/Z):  -- / -- / --", font=("Courier", 14))
        self.lbl_accel.pack(anchor=W, pady=3)
        self.lbl_pos = tb.Label(telem_card, text="Pos.  (X/Y/Z):  -- / -- / --", font=("Courier", 14))
        self.lbl_pos.pack(anchor=W, pady=3)
        self.lbl_pressure = tb.Label(telem_card, text="Baro Pressure: -- Pa", font=("Courier", 14), bootstyle=WARNING)
        self.lbl_pressure.pack(anchor=W, pady=3)
        
        #graphs Frame
        graph_frame = tb.Frame(self.root, padding=10)
        graph_frame.pack(fill=BOTH, expand=True)
        
        self.fig = plt.Figure(figsize=(12, 5))
        self.fig.patch.set_facecolor('#222222')
        
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_3d.set_title("Orientation Render", color='white', pad=10, fontsize=14)
        self.ax_3d.set_xlim([-6, 6])
        self.ax_3d.set_ylim([-6, 6])
        self.ax_3d.set_zlim([-6, 6])
        self.ax_3d.set_facecolor('#222222')
        self.ax_3d.axis('off')
        
        self.line_collection = Line3DCollection(self.lander_lines_base, colors=self.lander_colors, linewidths=2)
        self.ax_3d.add_collection3d(self.line_collection)
        
        self.ax_pos = self.fig.add_subplot(122, projection='3d')
        self.ax_pos.set_title("Navigation State Space", color='white', pad=10, fontsize=14)
        self.ax_pos.set_facecolor('#222222')
        self.ax_pos.xaxis.pane.fill = False
        self.ax_pos.yaxis.pane.fill = False
        self.ax_pos.zaxis.pane.fill = False
        self.ax_pos.set_xlabel("X (m)", labelpad=10)
        self.ax_pos.set_ylabel("Y (m)", labelpad=10)
        self.ax_pos.set_zlabel("Z (m)", labelpad=10)
        self.pos_line, = self.ax_pos.plot([], [], [], '#00FFCC', marker='.', linestyle='-', markersize=3, linewidth=1.5)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        commands_frame = tb.Frame(self.root, padding=15)
        commands_frame.pack(fill=X)
        
        self.estop_btn = tb.Button(commands_frame, text="HARD SOFTWARE ESTOP", bootstyle=DANGER,
                                   command=self.send_estop)
        self.estop_btn.pack(side=LEFT, fill=X, expand=True, ipady=15, padx=(0, 10))
        
        self.zero_imu_btn = tb.Button(commands_frame, text="ZERO IMU (CALIBRATE)", bootstyle=PRIMARY,
                                      command=self.send_zero_imu)
        self.zero_imu_btn.pack(side=RIGHT, fill=X, expand=True, ipady=15, padx=(10, 0))
        
        status_frame = tb.Frame(self.root, padding=5)
        status_frame.pack(fill=X)
        
        self.lbl_estop_status = tb.Label(status_frame, text="ESTOP armed", font=("Helvetica", 10, "bold"), bootstyle=SUCCESS)
        self.lbl_estop_status.pack(side=LEFT, padx=15)
        
        self.lbl_zero_imu_status = tb.Label(status_frame, text="IMU Calibration: PENDING", font=("Helvetica", 10, "bold"), bootstyle=INFO)
        self.lbl_zero_imu_status.pack(side=RIGHT, padx=15)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports:
            self.port_cb.current(0)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.is_reading = False
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.config(text="Connect", bootstyle=SUCCESS)
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.is_reading = True
                self.connect_btn.config(text="Disconnect", bootstyle=DANGER)
                threading.Thread(target=self.read_serial, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {e}")

    def read_serial(self):
        while self.is_reading:
            try:
                if self.serial_port and self.serial_port.is_open:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith('{') and line.endswith('}'):
                            try:
                                data = json.loads(line)
                                self.root.after(0, self.update_ui, data)
                            except json.JSONDecodeError:
                                pass
                        elif "HW ESTOP TRIGGERED" in line:
                            self.root.after(0, lambda: self.lbl_estop_status.config(text="ESTOP System: HARDARE TRIGGERED", bootstyle=DANGER))
                else:
                    break
            except Exception as e:
                print(f"Serial read error: {e}")
                self.root.after(0, self.handle_disconnect)
                break

    def handle_disconnect(self):
        self.is_reading = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.connect_btn.config(text="Connect", bootstyle=SUCCESS)
        messagebox.showwarning("Connection Lost", "Connection to the serial port was lost. Please reconnect.")

    def update_orientation_plot(self, roll, pitch, yaw):
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
        
        cr, sr = np.cos(roll_rad), np.sin(roll_rad)
        cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
        cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
        
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        
        pts = self.lander_lines_base.reshape(-1, 3)
        rotated_pts = pts @ R.T
        self.line_collection.set_segments(rotated_pts.reshape(-1, 2, 3))

    def zero_origin(self):
        self.offset_x = self.last_px
        self.offset_y = self.last_py
        self.offset_z = self.last_pz
        self.pos_history_x.clear()
        self.pos_history_y.clear()
        self.pos_history_z.clear()

    def update_position_plot(self, px, py, pz):
        self.last_px = px
        self.last_py = py
        self.last_pz = pz
        
        self.pos_history_x.append(px - self.offset_x)
        self.pos_history_y.append(py - self.offset_y)
        self.pos_history_z.append(pz - self.offset_z)
        
        max_points = 500
        if len(self.pos_history_x) > max_points:
            self.pos_history_x = self.pos_history_x[-max_points:]
            self.pos_history_y = self.pos_history_y[-max_points:]
            self.pos_history_z = self.pos_history_z[-max_points:]
            
        self.pos_line.set_data(self.pos_history_x, self.pos_history_y)
        self.pos_line.set_3d_properties(self.pos_history_z)
        
        if len(self.pos_history_x) > 1:
            margin = 2.0
            self.ax_pos.set_xlim([min(self.pos_history_x)-margin, max(self.pos_history_x)+margin])
            self.ax_pos.set_ylim([min(self.pos_history_y)-margin, max(self.pos_history_y)+margin])
            self.ax_pos.set_zlim([min(self.pos_history_z)-margin, max(self.pos_history_z)+margin])

    def update_ui(self, data):
        if data.get("type") == "telemetry":
            self.lbl_success.config(text=f"Success Rate: {data.get('success_rate', 0):.1f} %")
            self.lbl_lost.config(text=f"Packets Lost: {data.get('lost', 0)}")
            self.lbl_rssi.config(text=f"RSSI: {data.get('rssi', 0)} dBm")
            
            roll, pitch, yaw = data.get('roll',0), data.get('pitch',0), data.get('yaw',0)
            px, py, pz = data.get('px',0), data.get('py',0), data.get('pz',0)
            
            self.lbl_seq.config(text=f"Sequence: {data.get('seq', 0)}")
            self.lbl_euler.config(text=f"Euler (R/P/Y):  {roll:7.2f} / {pitch:7.2f} / {yaw:7.2f}")
            self.lbl_accel.config(text=f"Accel (X/Y/Z):  {data.get('ax',0):7.2f} / {data.get('ay',0):7.2f} / {data.get('az',0):7.2f}")
            self.lbl_pos.config(text=f"Pos.  (X/Y/Z):  {px:7.2f} / {py:7.2f} / {pz:7.2f}")
            self.lbl_pressure.config(text=f"Baro Pressure: {data.get('pressure',0):.0f} Pa")
            
            curr_time = time.time()
            if curr_time - self.last_draw_time >= self.draw_interval:
                self.update_orientation_plot(roll, pitch, yaw)
                self.update_position_plot(px, py, pz)
                self.canvas.draw_idle()
                self.last_draw_time = curr_time
            
            if self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), data.get('seq'), data.get('rssi'), data.get('success_rate'), data.get('lost'),
                    roll, pitch, yaw,
                    data.get('ax'), data.get('ay'), data.get('az'),
                    px, py, pz, data.get('pressure')
                ])
                self.csv_file.flush()
                
        elif data.get("type") == "bandwidth":
            self.lbl_bw.config(text=f"Bandwidth: {data.get('kbps', 0):.2f} kbps")
            
        elif data.get("type") == "estop_status":
            if data.get("status") == "sent":
                self.lbl_estop_status.config(text="ESTOP System: SIGNAL DEPLOYED", bootstyle=WARNING)
            else:
                self.lbl_estop_status.config(text="ESTOP System: SIGNAL FAILURE", bootstyle=DANGER)

        elif data.get("type") == "zero_imu_status":
            if data.get("status") == "sent":
                self.lbl_zero_imu_status.config(text="IMU Calibration: OVERRIDE INITIATED", bootstyle=SUCCESS)
                self.zero_origin() #visually clear graph too
            else:
                self.lbl_zero_imu_status.config(text="IMU Calibration: OVERRIDE FAILED", bootstyle=DANGER)

    def send_estop(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"ESTOP\n")
                self.lbl_estop_status.config(text="ESTOP System: TRIGGERED!", bootstyle=DANGER)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send ESTOP: {e}")
        else:
            messagebox.showwarning("Warning", "Not connected to serial port", parent=self.root)

    def send_zero_imu(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"ZERO_IMU\n")
                self.lbl_zero_imu_status.config(text="IMU Calibration: SENDING OVERRIDE", bootstyle=PRIMARY)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send ZERO IMU: {e}")
        else:
            messagebox.showwarning("Warning", "Not connected to serial port", parent=self.root)

    def on_closing(self):
        self.is_reading = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        if self.csv_file:
            self.csv_file.close()
        self.root.quit()
        self.root.destroy()

if __name__ == "__main__":
    app_window = tb.Window(title="Lander Ground Station", themename="darkly", size=(1400, 900))
    app = GroundStationUI(app_window)
    app_window.protocol("WM_DELETE_WINDOW", app.on_closing)
    app_window.mainloop()
