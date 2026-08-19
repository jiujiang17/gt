from Robotic_Arm.rm_robot_interface import *
import serial
import time


# 创建机械臂 SDK 对象。RM_TRIPLE_MODE_E 是 SDK 定义的三线程工作模式。
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# 创建到机械臂控制器的网络连接。请根据实际控制器配置修改 IP 地址和端口。
# handle.id 用于确认 SDK 成功建立连接。
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)

z_step=0.0658

# origin_x_1, origin_y_1, origin_z_1 = 0.66755, -0.022922, -0.094534
origin_x_1, origin_y_1, origin_z_1 =  0.47899,-0.099395,-0.103477
origin_x_2, origin_y_2, origin_z_2 =  0.236851,-0.116546,-0.284757+0.0658
origin_x_3, origin_y_3, origin_z_3 = 0.47899,0.099395,-0.103477
origin_x_4, origin_y_4, origin_z_4 = 0.236851,0.116546,-0.284757+0.0658

x_step = -0.0415  # 横向：从左到右
y_step = -0.0380  # 纵向：从下到上

angle_1=[33.394,20.006,-112.197,88.971,122.956,-181.26]
angle_2=[13.907,-40.094,-127.873,102.72,94.759,-104.334]
angle_3=[-34.578,20.955,-112.199,90.835,-122.097,-0.815]
angle_4=[-13.3,-40.141,-125.381,79.865,-94.015,-75.952]



def send_modbus_rtu_data(port:str='COM7', baudrate:int=115200, data_hex:str='3E 30 31 47 36 31 35 38 0D 0A'):
    try:
        # 将十六进制字符串转换为字节数组
        data_bytes = bytes.fromhex(data_hex)
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        if ser.is_open:
            print(f"串口 {port} 已打开")
            print(f"准备发送数据: {data_hex}")
            # 发送数据
            ser.write(data_bytes)
            print(f"成功发送 {len(data_bytes)} 字节数据")
            # 可选：读取响应
            response = ser.read(100)
            if response:
                print(f"收到响应: {response.hex().upper()}")
            # 关闭串口
            ser.close()
            print(f"串口 {port} 已关闭")
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except Exception as e:
        print(f"发送失败: {e}")
def board_to_real(i: int, j: int, z: float = 0.0,numble:int=1) -> list[float]:
    """
    i：左到右 0~4
    j：下到上 0~6
    返回机械臂位姿：[x, y, z, rx, ry, rz]
    """
    if not (0 <= i <= 4 and 0 <= j <= 6):
        raise ValueError("棋盘坐标范围：i 为 0~4，j 为 0~6")
    if numble==1:
        x = origin_x_1 - i * x_step
        y = origin_y_1 + j * y_step
        return [x,y,z,2.864,1.528,1.302]
    if numble==2:
        x = origin_x_2 - i * x_step
        y = origin_y_2 + j * y_step
        return [x,y,z,2.864,1.528,1.302]
    if numble==3:
        x = origin_x_3 - i * x_step
        y = origin_y_3 - j * y_step
        return [x, y, z, -3.036,1.527,-1.489]
    if numble==4:
        x = origin_x_4 - i * x_step
        y = origin_y_4 - j * y_step
        return [x, y, z, -3.036,1.527,-1.489]
    
def square_1():
    print(arm.rm_movej(angle_1, 10, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_1,numble=1), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_1-z_step,numble=1), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_1,numble=1), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(0, 6, origin_z_1,numble=1), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 6, origin_z_1-z_step,numble=1), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(0, 6, origin_z_1,numble=1), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(4, 6, origin_z_1,numble=1), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 6, origin_z_1-z_step,numble=1), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4, 6, origin_z_1,numble=1), 20, 0, 0, 1))
    time.sleep(0.5)

    print(arm.rm_movel(board_to_real(4, 0, origin_z_1,numble=1), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 0, origin_z_1-z_step,numble=1), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4,0, origin_z_1,numble=1), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(0,0, origin_z_1,numble=1), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_1-z_step,numble=1), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_1,numble=1), 20, 0, 0, 1))    
def square_2():
    print(arm.rm_movej(angle_2, 10, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_2,numble=2), 20, 0, 0, 1))
    print(board_to_real(0, 0, origin_z_2,numble=2))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_2-z_step,numble=2), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_2,numble=2), 20, 0, 0, 1))
    
    print(arm.rm_movel(board_to_real(0, 6, origin_z_2,numble=2), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 6, origin_z_2-z_step,numble=2), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(0, 6, origin_z_2,numble=2), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(4, 6, origin_z_2,numble=2), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 6, origin_z_2-z_step,numble=2), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4, 6, origin_z_2,numble=2), 20, 0, 0, 1))
    time.sleep(0.5)

    print(arm.rm_movel(board_to_real(4, 0, origin_z_2,numble=2), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 0, origin_z_2-z_step,numble=2), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4,0, origin_z_2,numble=2), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(0,0, origin_z_2,numble=2), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_2-z_step,numble=2), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_2,numble=2), 20, 0, 0, 1))
def square_3():
    print(arm.rm_movej(angle_3, 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_3,numble=3), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_3-z_step,numble=3), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_3,numble=3), 20, 0, 0, 1))
    
    print(arm.rm_movel(board_to_real(0, 6, origin_z_3,numble=3), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 6, origin_z_3-z_step,numble=3), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(0, 6, origin_z_3,numble=3), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(4, 6, origin_z_3,numble=3), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 6, origin_z_3-z_step,numble=3), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4, 6, origin_z_3,numble=3), 20, 0, 0, 1))
    time.sleep(0.5)

    print(arm.rm_movel(board_to_real(4, 0, origin_z_3,numble=3), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 0, origin_z_3-z_step,numble=3), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4,0, origin_z_3,numble=3), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(0,0, origin_z_3,numble=3), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_3-z_step,numble=3), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_3,numble=3), 20, 0, 0, 1))
def square_4():
    print(arm.rm_movej(angle_4, 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_4,numble=4), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_4-z_step,numble=4), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_4,numble=4), 20, 0, 0, 1))
    
    print(arm.rm_movel(board_to_real(0, 6, origin_z_4,numble=4), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 6, origin_z_4-z_step,numble=4), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(0, 6, origin_z_4,numble=4), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(4, 6, origin_z_4,numble=4), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 6, origin_z_4-z_step,numble=4), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4, 6, origin_z_4,numble=4), 20, 0, 0, 1))
    time.sleep(0.5)

    print(arm.rm_movel(board_to_real(4, 0, origin_z_4,numble=4), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(4, 0, origin_z_4-z_step,numble=4), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(0.5)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 5D 59 FD')
    time.sleep(1)
    print(arm.rm_movel(board_to_real(4,0, origin_z_4,numble=4), 20, 0, 0, 1))

    print(arm.rm_movel(board_to_real(0,0, origin_z_4,numble=4), 20, 0, 0, 1))
    print(arm.rm_movel(board_to_real(0, 0, origin_z_4-z_step,numble=4), 20, 0, 0, 1))
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    time.sleep(1.5)
    print(arm.rm_movel(board_to_real(0, 0, origin_z_4,numble=4), 20, 0, 0, 1))

if __name__ == "__main__":
    # with CL2CMotor(port="COM3", slave_id=1) as motor:
    #     motor.enable()  
    #     motor.home(home_mode=12, home_speed=500, wait=True)
    #     motor.move_absolute(280000, speed=200, acceleration=50, deceleration=50)
    #     motor.wait_until_complete(timeout=30)
    send_modbus_rtu_data(data_hex='02 06 01 00 00 01 49 C5')#初始化
    time.sleep(1)
    send_modbus_rtu_data(data_hex='02 06 01 05 00 00 98 04')
    print(arm.rm_movej(angle_4, 5, 0, 0, 1))
    print(arm.rm_movej([-5.84,-6.78,-124.118,86.415,-91.838,-40.061], 20, 0, 0, 1))
    # print(arm.rm_movel([0.297906,-0.287422,0.186877,-2.391,1.544,2.416], 20, 0, 0, 1))
    # print(arm.rm_movel([origin_x_2, origin_y_2, origin_z_2,-1.758,1.525,2.99], 10, 0, 0, 1))
    # print(arm.rm_movej(angle_4, 5, 0, 0, 1))
    # print(arm.rm_movel([origin_x_1, origin_y_1, origin_z_1,-3.036,1.527,-1.489], 20, 0, 0, 1))
    # print(arm.rm_movej(angle_1, 5, 0, 0, 1))

    # square_1()
    # print(arm.rm_movej([33.48,-12.35,-131.024,115.692,-74.866,-50.364], 5, 0, 0, 1))
    # print(arm.rm_movej([4.975,28.13,-132.165,97.844,102.948,-171.758], 5, 0, 0, 1))
    # print(arm.rm_movej([-2.13,31.011,-102.871,79.368,-92.296,16.83], 5, 0, 0, 1))
    # square_2()
    # print(arm.rm_movej([4.975,28.13,-132.165,97.844,102.948,-171.758], 5, 0, 0, 1))
    # square_1()
    # with CL2CMotor(port="COM3", slave_id=1) as motor:
    #     motor.move_absolute(400000, speed=200, acceleration=50, deceleration=50)
    #     motor.wait_until_complete(timeout=30)
    # square_3()
    # print(arm.rm_movej([-9.731,-83.313,132.103,98.735,91.325,32.132],20,0,0,1))
    # print(arm.rm_movej([-29.871,-92.757,132.227,-67.765,-103.439,229.984],20,0,0,1))
    # print(arm.rm_movej([91.711,-89.502,82.107,-3.782,93.842,82.656],20,0,0,1))
    # with CL2CMotor(port="COM3", slave_id=1) as motor:
    #     motor.move_absolute(370000, speed=200, acceleration=50, deceleration=50)
    #     motor.wait_until_complete(timeout=30)
    # square_1()
    arm.rm_delete_robot_arm()






