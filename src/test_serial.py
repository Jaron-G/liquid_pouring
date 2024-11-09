import serial
import serial.tools.list_ports

# 获取所有串口设备实例。
# 如果没找到串口设备，则输出：“无串口设备。”
# 如果找到串口设备，则依次输出每个设备对应的串口号和描述信息。
ports_list = list(serial.tools.list_ports.comports())
if len(ports_list) <= 0:
    print("无串口设备。")
else:
    print("可用的串口设备如下：")
for comport in ports_list:
    print(list(comport)[0], list(comport)[1])
    

# 调用函数接口打开串口时传入配置参数
ser = serial.Serial(
port="/dev/ttyUSB1",
baudrate=9600,
bytesize=serial.EIGHTBITS,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
)


# 打开串口，将波特率配置为9600，停止位1，无校验位，数据位8位。
if ser.isOpen(): # 判断串口是否成功打开
    print("打开串口成功。")
    print(ser.name) # 输出串口号
else:
    print("打开串口失败。")


while True:
    line = ser.readline().decode().strip()
    if line: # 如果读取结果非空，则输出
        print(line)