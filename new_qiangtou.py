"""枪头棋盘扫描程序。

位姿起点、末端姿态和格距从同目录的 ``arm_pos.json`` 的 ``qiangtou``
节点加载；扫描及取放动作与 ``test_spuare.py`` 保持一致。
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import serial
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


CONFIG_PATH = Path(__file__).with_name("arm_pos.json")
ROBOT_IP = "192.168.1.19"
ROBOT_PORT = 8080

# 这三个高度是原 test_spuare.py 的实际标定值。arm_pos.json 的 qiangtou
# 节点尚未提供对应字段，故保留它们以保证动作轨迹不变。
Z_MIN = -0.0578 - 0.059
Z_MIDDLE = -0.030056 - 0.059
Z_MAX = 0.094418 - 0.059

GUN_COMMAND = {
    "motor_reset": "3E 30 31 47 36 31 35 38",
    "retract": "3E 30 31 51 41 46 44 39",
}


def load_qiangtou_config(path: Path = CONFIG_PATH) -> dict[str, object]:
    """读取并校验枪头扫描所需的 JSON 配置。"""
    with path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)

    try:
        qiangtou = config["qiangtou"]
        location = qiangtou["location"]
        angle = qiangtou["angle"]
        step_width = qiangtou["step_width"]
    except (KeyError, TypeError) as exc:
        raise ValueError("arm_pos.json 缺少 qiangtou.location、angle 或 step_width") from exc

    if not isinstance(location, list) or len(location) != 6:
        raise ValueError("qiangtou.location 必须是包含 6 个数值的数组")
    if not isinstance(angle, list) or len(angle) != 6:
        raise ValueError("qiangtou.angle 必须是包含 6 个数值的数组")
    if not isinstance(step_width, (int, float)) or step_width <= 0:
        raise ValueError("qiangtou.step_width 必须为正数")

    return qiangtou


QIANGTOU = load_qiangtou_config()
ORIGIN_X, ORIGIN_Y = QIANGTOU["location"][:2]
VERTICAL_RX, VERTICAL_RY, VERTICAL_RZ = QIANGTOU["location"][3:]
STEP_WIDTH = QIANGTOU["step_width"]


def send_modbus_rtu_data(
    port: str = "COM7", baudrate: int = 115200, data_hex: str = GUN_COMMAND["motor_reset"]
) -> None:
    """向枪头控制器发送十六进制串口命令。"""
    try:
        data_bytes = bytes.fromhex(data_hex)
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        ) as ser:
            print(f"串口 {port} 已打开")
            print(f"准备发送数据: {data_hex}")
            ser.write(data_bytes)
            print(f"成功发送 {len(data_bytes)} 字节数据")
            response = ser.read(100)
            if response:
                print(f"收到响应: {response.hex().upper()}")
    except serial.SerialException as exc:
        print(f"串口错误: {exc}")
    except ValueError as exc:
        print(f"命令格式错误: {exc}")


def board_to_real(i: int, j: int) -> list[float]:
    """将 9×9 棋盘交点映射为机器人基座坐标中的 [x, y]。"""
    if not (0 <= i <= 8 and 0 <= j <= 8):
        raise ValueError("棋盘坐标必须在 0~8 范围内")
    return [ORIGIN_X + i * STEP_WIDTH, ORIGIN_Y + j * STEP_WIDTH]


def insert_four(position: list[float], z: float) -> list[float]:
    """补全为 rm_movel 所需的 [x, y, z, rx, ry, rz] 位姿。"""
    return [*position, z, VERTICAL_RX, VERTICAL_RY, VERTICAL_RZ]


def test_path(end: tuple[int, int] = (8, 8), size: int = 9) -> list[tuple[int, int]]:
    """生成与 test_spuare.py 相同的逐行往返扫描端点。"""
    if not (0 <= end[0] < size and 0 <= end[1] < size):
        raise ValueError(f"终点必须在 0~{size - 1} 范围内")

    path: list[tuple[int, int]] = []
    for y in range(size):
        for x in (0, size - 1):
            path.append((x, y))
            if path[-1] == end:
                return path
    raise ValueError("终点不在当前轨迹中")


def run(arm: RoboticArm) -> None:
    """执行扫描：左端取料，右端退枪头。"""
    for board_point in test_path((8, 8)):
        pose_middle = insert_four(board_to_real(*board_point), Z_MIDDLE)
        pose_min = insert_four(board_to_real(*board_point), Z_MIN)
        pose_max = insert_four(board_to_real(*board_point), Z_MAX)

        print(arm.rm_movel(pose_max, 50, 0, 0, 1))
        print(board_point, arm.rm_movel(pose_middle, 50, 0, 0, 1))
        if board_point[0] == 8:
            send_modbus_rtu_data(data_hex=GUN_COMMAND["retract"])
            time.sleep(1)
        else:
            print(arm.rm_movel(pose_min, 20, 0, 0, 1))
            print("取了")
        print(arm.rm_movel(pose_max, 50, 0, 0, 1))


def main() -> None:
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT)
    if getattr(handle, "id", -1) == -1:
        raise ConnectionError(f"无法连接机械臂 {ROBOT_IP}:{ROBOT_PORT}")

    try:
        send_modbus_rtu_data(data_hex=GUN_COMMAND["motor_reset"])
        run(arm)
    finally:
        arm.rm_delete_robot_arm()


if __name__ == "__main__":
    main()
