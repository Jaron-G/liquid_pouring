import tkinter as tk
from tkinter import scrolledtext
import signal
import rospy
import serial
import serial.tools.list_ports
from pd_controller import PDController

from robot_joint_vec_control import RobotMover


class SerialHelper:
    def __init__(self, master):
        self.master = master
        self.robot_mover = RobotMover()
        signal.signal(signal.SIGINT, self.robot_mover.signal_handler)
        master.title("Liquid pouring debugging assistant")

        frame_left = tk.Frame(master)
        frame_left.pack(side=tk.LEFT)

        frame_right = tk.Frame(master)
        frame_right.pack(side=tk.RIGHT)

        self.port_label = tk.Label(frame_left, text="Select serial port:").grid(
            row=1, column=1, pady=5
        )

        self.port_combobox = tk.StringVar(frame_left)
        self.port_list = serial.tools.list_ports.comports()
        self.port_combobox.set(self.port_list[0].device if self.port_list else "")
        self.port_menu = tk.OptionMenu(
            frame_left, self.port_combobox, *[port.device for port in self.port_list]
        )
        self.port_menu.grid(row=1, column=2, pady=5)

        self.baudrate_label = tk.Label(frame_left, text="Baud rate:")
        self.baudrate_label.grid(row=2, column=1, pady=5)

        self.baudrate_entry = tk.Entry(frame_left)
        self.baudrate_entry.insert(0, "9600")
        self.baudrate_entry.grid(row=2, column=2, pady=5)

        self.open_button = tk.Button(
            frame_left, text="Open serial port", command=self.open_serial
        )
        self.open_button.grid(row=3, column=1, pady=5)

        self.close_button = tk.Button(
            frame_left, text="Close serial port", command=self.close_serial
        )
        self.close_button.grid(row=3, column=2, pady=5)

        self.text_area = scrolledtext.ScrolledText(frame_left, width=40, height=10)
        self.text_area.grid(row=4, column=1, columnspan=2, pady=5)

        self.send_entry = tk.Entry(frame_left)
        self.send_entry.grid(row=5, column=1, columnspan=2, pady=5)

        self.send_button = tk.Button(frame_left, text="Send", command=self.send_data)
        self.send_button.grid(row=6, column=1, pady=5)
        self.receive_button = tk.Button(
            frame_left, text="Receive", command=self.receive_data
        )
        self.receive_button.grid(row=6, column=2, pady=5)
        self.serial = None

        self.open_controller_button = tk.Button(
            frame_right, text="Open Controller", command=self.open_controller
        )
        self.open_controller_button.grid(row=1, column=1, pady=5)

        self.close_controller_button = tk.Button(
            frame_right, text="Close Controller", command=self.close_controller
        )
        self.close_controller_button.grid(row=1, column=2, pady=5)

        self.weight_area = scrolledtext.ScrolledText(frame_right, width=40, height=10)
        self.weight_area.grid(row=2, column=1, columnspan=2, pady=5)

        self.kp_label = tk.Label(frame_right, text="kp：").grid(row=3, column=1, pady=5)
        self.kp_entry = tk.Entry(frame_right)
        self.kp_entry.grid(row=3, column=2, pady=5)
        self.kd_label = tk.Label(frame_right, text="kd：").grid(row=4, column=1, pady=5)
        self.kd_entry = tk.Entry(frame_right)
        self.kd_entry.grid(row=4, column=2, pady=5)

        self.min_joint_angle_label = tk.Label(frame_right, text="min_joint_angle：")
        self.min_joint_angle_label.grid(row=5, column=1, pady=5)
        self.min_joint_angle_entry = tk.Entry(frame_right)
        self.min_joint_angle_entry.grid(row=5, column=2, pady=5)
        self.max_joint_angle_label = tk.Label(frame_right, text="max_joint_angle：")
        self.max_joint_angle_label.grid(row=6, column=1, pady=5)
        self.max_joint_angle_entry = tk.Entry(frame_right)
        self.max_joint_angle_entry.grid(row=6, column=2, pady=5)

    def open_serial(self):
        port = self.port_combobox.get()
        baudrate = int(self.baudrate_entry.get())
        self.serial = serial.Serial(port, baudrate)
        self.text_area.insert(tk.END, f"The serial port {port} has been opened ，Baud rate: {baudrate}\n")

    def close_serial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.text_area.insert(tk.END, "串口已关闭\n")

    def send_data(self):
        if self.serial and self.serial.is_open:
            data = self.send_entry.get()
            self.serial.write(data.encode())
            self.text_area.insert(tk.END, f"发送: {data}\n")

    def receive_data(self):
        if self.serial and self.serial.is_open:
            data = self.serial.readline().decode().strip()
            if data:
                self.text_area.insert(tk.END, f"接收: {data}\n")

    def open_controller(self):

        port = self.port_combobox.get()
        baudrate = int(self.baudrate_entry.get())
        self.serial = serial.Serial(port, baudrate)
        self.text_area.insert(tk.END, f"已打开串口 {port}，波特率 {baudrate}\n")
        kp = float(self.kp_entry.get())
        kd = float(self.kd_entry.get())

        joint_range = (
            float(self.min_joint_angle_entry.get()),
            float(self.max_joint_angle_entry.get()),
        )
        self.weight_area.insert(
            tk.END,
            f"打开控制器，kp={kp}, kd={kd}, min_joint_angle={joint_range[0]}, max_joint_angle={joint_range[1]}\n",
        )

        pd_controller = PDController(kp, kd, joint_range)
        target_weight = 100.0

        pd_controller_flag = True
        rate = rospy.Rate(10)
        while pd_controller_flag:
            current_weight = float(self.serial.readline().decode().strip())
            joint_angle = pd_controller.calculate(target_weight, current_weight)
            print(
                f"当前重量：{current_weight}, 目标重量：{target_weight}, 关节角度：{joint_angle}"
            )
            self.robot_mover.set_wrist3_speed(joint_angle)
            rate.sleep()

        # def task1(self):
        #     self.weight_area.insert(tk.END, f"当前重量：{current_weight}\n")
        #     self.weight_area.insert(tk.END, f"目标重量：{target_weight}\n")
        #     self.weight_area.insert(tk.END, f"关节角度：{joint_angle}\n")

        # # 创建线程
        # thread1 = threading.Thread(target=task1)
        # thread1.start()
        # thread1.join()

    def close_controller(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.text_area.insert(tk.END, "串口已关闭\n")


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialHelper(root)
    root.mainloop()
