from pymodbus.client.sync import ModbusTcpClient #pip3 install pymodbus==2.5.3
import time
from pymodbus.pdu import ExceptionResponse
import struct

# Register addresses
REGDICT = {
    'ID': 1000,
    'baudrate': 1001,
    'clearErr': 1004,
    'forceClb': 1009,
    'angleSet': 1486,
    'forceSet': 1498,
    'speedSet': 1522,
    'angleAct': 1546,
    'forceAct': 1582,
    'errCode': 1606,
    'statusCode': 1612,
    'temp': 1618,
    'actionSeq': 2320,
    'actionRun': 2322

}

# starting address range of each finger
TOUCH_SENSOR_BASE_ADDR_PINKY = 3000   # pinky
TOUCH_SENSOR_BASE_ADDR_RING = 3058     # ring
TOUCH_SENSOR_BASE_ADDR_MIDDLE = 3116   # middle
TOUCH_SENSOR_BASE_ADDR_INDEX = 3174    # index
TOUCH_SENSOR_BASE_ADDR_THUMB = 3232    # thumb

## TACTILE DATA
def read_register_range(client, start_addr, count):
    register_values = []
    response = client.read_holding_registers(address=start_addr, count=count)

    if isinstance(response, ExceptionResponse) or response.isError():
        print(f"Reading register {start_addr} failed: {response}")
        return None
    else:
        register_values = response.registers

    return register_values

def read_float_from_bytes(registers, index):
    """
    Read a floating point number from a register, given the starting index.
    """
    # Get 4 bytes
    byte0 = registers[index] & 0xFF       # Low 8 bits
    byte1 = (registers[index] >> 8) & 0xFF # Low 8 bits
    byte2 = registers[index + 1] & 0xFF   # Low 8 bits
    byte3 = (registers[index + 1] >> 8) & 0xFF # Low 8 bits

    # Data conversion
    combined = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0

    result = struct.unpack('!f', struct.pack('!I', combined))[0]
    
    return result

def read_finger_data(client, base_addr):
    """
    Read the normal and tangential force data of the finger.
    The normal force is located at base_addr + 32, and the tangential force is located at base_addr + 40.
    """
    # Read register data
    register_values = read_register_range(client, base_addr, 25)  

    if register_values is None:
        return None
    
    # Reading normal and tangential forces
    normal_force = read_float_from_bytes(register_values, 16)  
    tangential_force = read_float_from_bytes(register_values, 20)  

    return normal_force, tangential_force

def read_all_tactile(client):
    pinky_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_PINKY)
    ring_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_RING)
    middle_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_MIDDLE)
    index_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_INDEX)
    thumb_force = read_finger_data(client, TOUCH_SENSOR_BASE_ADDR_THUMB)
    return pinky_force, ring_force, middle_force, index_force, thumb_force


# ACTUATOR CONTROL

def open_modbus(ip, port):
    client = ModbusTcpClient(ip, port)
    client.connect()
    return client

def write_register(client, address, values):
    #Modbus write register, pass in the register address and the list of values ​​to be written
    client.write_registers(address, values)

def read_register(client, address, count):
    # Modbus 读取寄存器
    response = client.read_holding_registers(address, count)
    return response.registers if response.isError() is False else []

def write6(client, reg_name, val):
    if reg_name in ['angleSet', 'forceSet', 'speedSet']:
        val_reg = []
        for i in range(6):
            val_reg.append(val[i] & 0xFFFF)  # Take the lower 16 bits
        write_register(client, REGDICT[reg_name], val_reg)
    else:
        print('Function call error, correct way: str value is \'angleSet\'/\'forceSet\'/\'speedSet\', val is a list of length 6, value is 0~1000, -1 is allowed as a placeholder')

def read6(client, reg_name):
    # Check if the register name is within the allowed range
    if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
        # Directly read the register corresponding to reg_name, the number of registers read is 6
        val = read_register(client, REGDICT[reg_name], 6)
        if len(val) < 6:
            print('No data was read')
            return
        # print('The values ​​read are：', end='')
        # for v in val:
        #     print(v, end=' ')
        # print()
        return val
    
    elif reg_name in ['errCode', 'statusCode', 'temp']:
        #Read error code, status code or temperature, 3 registers at a time
        val_act = read_register(client, REGDICT[reg_name], 3)
        if len(val_act) < 3:
            print('No data was read')
            return
            

        # Initialize the array that stores high and low bits
        results = []
        
        # Store the high and low bits of each register separately
        for i in range(len(val_act)):
            # Read the current register and the next register
            low_byte = val_act[i] & 0xFF            # Lower eight bits
            high_byte = (val_act[i] >> 8) & 0xFF     # Lower eight bits
        
            results.append(low_byte)  
            results.append(high_byte)  

        # print('The values ​​read are:', end='')
        # for v in results:
        #     print(v, end=' ')
        # print()
    
    else:
        print('Function call error, correct way: the value of str is \'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')


"""
Position_Actual - 0-2000
Angle_Actual - 0-1000
Speed_Set - 0-1000
Force_Set - 0-3000
Temp - 0-100C
Angle_Set - 0-1000
Pos_Set - 0-2000
"""

