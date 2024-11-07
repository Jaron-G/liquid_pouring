class PDController:
    def __init__(self, kp, kd, joint_range):
        """
        初始化PD控制器
        :param kp: 比例增益
        :param kd: 微分增益
        :param joint_range: 机器人关节的转角范围，这里是一个元组(min, max)
        """
        self.kp = kp
        self.kd = kd
        self.joint_range = joint_range
        self.previous_error = 0  # 上一次的误差

    def calculate(self, target_weight, current_weight):
        """
        计算关节的转角
        :param target_weight: 目标重量
        :param current_weight: 当前重量
        :return: 计算出的关节转角
        """
        # 计算重量差值
        error = target_weight - current_weight

        # 计算比例控制部分
        proportional = self.kp * error

        # 计算微分控制部分
        derivative = self.kd * (error - self.previous_error)

        # 更新上一次误差
        self.previous_error = error

        # 计算总控制输出
        output = proportional + derivative

        # 限制输出在关节的转角范围内
        output = max(self.joint_range[0], min(output, self.joint_range[1]))

        return output


if __name__ == "__main__":
    import numpy as np
    from matplotlib import pyplot as plt
    import serial
    import serial.tools.list_ports

    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    ser = serial.Serial(
        port="COM3",
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    # 示例使用
    kp = 10  # 比例增益
    kd = 0.1  # 微分增益
    joint_range = (-202, -90)  # 转角范围

    # 创建PD控制器实例
    pd_controller = PDController(kp, kd, joint_range)

    # 假设目标重量是100kg，当前重量是90kg
    target_weight = 100

    pd_controller_flag = True
    while pd_controller_flag:
        current_weight = float(ser.readline().decode().strip())
        joint_angle = pd_controller.calculate(target_weight, current_weight)
        print(
            f"当前重量：{current_weight}, 目标重量：{target_weight}, 关节角度：{joint_angle}"
        )

    # line = np.linspace(95, 120, 200)

    # jointlist = []
    # for i in line:
    #     # 计算关节转角
    #     joint_angle = pd_controller.calculate(target_weight, i)
    #     print(f"{i}: {joint_angle}")
    #     jointlist.append(joint_angle)
    # plt.plot(line, jointlist)
    # plt.show()
