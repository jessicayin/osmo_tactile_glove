import time
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
import struct

# 定义 Modbus TCP 相关参数
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# 定义各手指数据起始地址范围
TOUCH_SENSOR_BASE_ADDR_PINKY = 3000   # 小拇指
TOUCH_SENSOR_BASE_ADDR_RING = 3058     # 无名指
TOUCH_SENSOR_BASE_ADDR_MIDDLE = 3116   # 中指
TOUCH_SENSOR_BASE_ADDR_INDEX = 3174    # 食指
TOUCH_SENSOR_BASE_ADDR_THUMB = 3232    # 大拇指

def read_register_range(client, start_addr, count):
    register_values = []
    response = client.read_holding_registers(address=start_addr, count=count)

    if isinstance(response, ExceptionResponse) or response.isError():
        print(f"读取寄存器 {start_addr} 失败: {response}")
        return None
    else:
        register_values = response.registers

    return register_values

def read_float_from_bytes(registers, index):
    """
    从寄存器中读取浮点数，给定起始索引。
    """
    # 获取4个字节
    byte0 = registers[index] & 0xFF       # 低8位
    byte1 = (registers[index] >> 8) & 0xFF # 高8位
    byte2 = registers[index + 1] & 0xFF   # 低8位
    byte3 = (registers[index + 1] >> 8) & 0xFF # 高8位

    # 数据转换
    combined = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0

    result = struct.unpack('!f', struct.pack('!I', combined))[0]
    
    return result

def read_finger_data(client, base_addr):
    """
    读取手指的法向力和切向力数据。
    法向力位于 base_addr + 32，切向力位于 base_addr + 40。
    """
    # 读寄存器数据
    register_values = read_register_range(client, base_addr, 25)  

    if register_values is None:
        return None
    
    # 读取法向力和切向力
    normal_force = read_float_from_bytes(register_values, 16)  # 法向力
    tangential_force = read_float_from_bytes(register_values, 20)  # 切向力

    return normal_force, tangential_force

def read_multiple_registers():
    client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    client.connect()

    try:
        while True:
            start_time = time.time()

            # 读取各手指数据
            pinky_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_PINKY)
            ring_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_RING)
            middle_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_MIDDLE)
            index_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_INDEX)
            thumb_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_THUMB)

            end_time = time.time()
            frequency = 1 / (end_time - start_time)

            # 输出数据
            print(f"小拇指法向力：{pinky_force[0]}, 切向力：{pinky_force[1]}")
            print(f"无名指法向力：{ring_force[0]}, 切向力：{ring_force[1]}")
            print(f"中指法向力：{middle_force[0]}, 切向力：{middle_force[1]}")
            print(f"食指法向力：{index_force[0]}, 切向力：{index_force[1]}")
            print(f"大拇指法向力：{thumb_force[0]}, 切向力：{thumb_force[1]}")
            print(f"读取频率：{frequency:.2f} Hz")

            time.sleep(0.02)

    finally:
        client.close()

if __name__ == "__main__":
    read_multiple_registers()

