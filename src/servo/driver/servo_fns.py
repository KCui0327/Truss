"""
Credits to Hiwonder for the original code.

We have modified the code to suit USB connections and adapted it for our specific hardware setup.
This code is intended for controlling servos via a USB interface.
"""

from .servo_driver import *
import time

def setBusServoID(oldid, newid, serialHandle=None):
    """
    配置舵机id号, 出厂默认为1
    :param oldid: 原来的id， 出厂默认为1
    :param newid: 新的id
    """
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid, serialHandle)

def getBusServoID(id=None):
    """
    读取串口舵机id
    :param id: 默认为空
    :return: 返回舵机id
    """
    
    while True:
        if id is None:  # 总线上只能有一个舵机
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ, serialHandle)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ, serialHandle)
        # 获取内容
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ, serialHandle)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time, serialHandle=None):
    """
    驱动串口舵机转到指定位置
    :param id: 要驱动的舵机id
    :pulse: 位置
    :use_time: 转动需要的时间
    """

    if serialHandle is None:
        raise ValueError("serialHandle is None")

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time, serialHandle)
    return pulse  #x新增反馈可以删除\

def getBusServoPulse(id, serialHandle=None, timeout=1):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")

    #CUSTOM TIMEOUT
    start = time.time()
    end = start
    while (end - start) <= timeout:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ, serialHandle)
        if msg is not None:
            return msg
        end = time.time()
    return None

def getBusServoTemp(id, serialHandle=None):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")

    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ, serialHandle)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    读取舵机电压
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ, serialHandle)
        if msg is not None:
            return msg

def stopBusServo(id=None, serialHandle=None):
    '''
    停止舵机运行
    :param id:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP, serialHandle)

def setBusServoDeviation(id, d=0, serialHandle=None):
    """
    调整偏差
    :param id: 舵机id
    :param d:  偏差
    """
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d, serialHandle)

def saveBusServoDeviation(id, serialHandle=None):
    """
    配置偏差，掉电保护
    :param id: 舵机id
    """
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE, serialHandle)

time_out = 50
def getBusServoDeviation(id, serialHandle=None):
    '''
    读取偏差值
    :param id: 舵机号
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    
    # 发送读取偏差指令
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ, serialHandle)
        # 获取
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ, serialHandle)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit(id, low, high, serialHandle=None):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high, serialHandle)

def getBusServoAngleLimit(id, serialHandle=None):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ, serialHandle)
        if msg is not None:
            count = 0
            return msg

def setBusServoVinLimit(id, low, high, serialHandle=None):
    '''
    设置舵机电压范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high, serialHandle)

def getBusServoVinLimit(id, serialHandle=None):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ, serialHandle)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp, serialHandle=None):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp, serialHandle)

def getBusServoTempLimit(id, serialHandle=None):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ, serialHandle)
        if msg is not None:
            return msg

def restBusServoPulse(oldid, serialHandle=None):
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    # 舵机清零偏差和P值中位（500）
    serial_servo_set_deviation(oldid, 0)    # 清零偏差
    time.sleep(0.1)
    
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100, serialHandle)    # 中位

def unloadBusServo(id, serialHandle=None):
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0, serialHandle)

def getBusServoLoadStatus(id, serialHandle=None):
    if serialHandle is None:
        raise ValueError("serialHandle is None")
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ, serialHandle)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ, serialHandle)
        if msg is not None:
            return msg

def getStatus(servo_id=None, serialHandle=None):
    angle_range = getBusServoAngleLimit(servo_id, serialHandle)
    vin_range = getBusServoVinLimit(servo_id, serialHandle)
    temp_limit = getBusServoTempLimit(servo_id, serialHandle)
    load_state = getBusServoLoadStatus(servo_id, serialHandle)
    
    curr_pulse = getBusServoPulse(servo_id, serialHandle)
    curr_vin = getBusServoVin(servo_id, serialHandle)
    curr_temp = getBusServoTemp(servo_id, serialHandle)
    
    print('*******current status**********')
    print('id:%s'%(str(servo_id).ljust(3)))
    print('angle_range:%s'%(str(angle_range).ljust(4)))
    print('voltage_range:%s'%(str(vin_range).ljust(5)))
    print('temp_limit:%s'%(str(temp_limit).ljust(4)))
    print('lock:%s'%(str(load_state).ljust(4)))
    print('*******************************')
    print('curr_pulse:%s'%(str(curr_pulse).ljust(4)))
    print('curr_vin:%s'%(str(curr_vin).ljust(4)))
    print('curr_temp:%s'%(str(curr_temp).ljust(4)))
    print('*******************************')

def ServoSetup(serialHandle=None):
    print('''
    --------------------------               
    1 id                
    2 dev               
    3 save_dev          
    4 help      
    5 voltage_range   
    6 temperature_warn
    7 lock              
    8 exit
    --------------------------''')

    if serialHandle is None:
        raise ValueError("serialHandle is None")
    # 4 angle_range 
    while True:
        try:
            #getStatus()
            mode = int(input('input mode:'))
            if mode == 1:
                oldid = int(input('current id:'))
                newid = int(input('new id(0-253):'))
                setBusServoID(oldid, newid, serialHandle)
            elif mode == 2:
                servo_id = int(input('servo id:'))
                dev = int(input('deviation(-125~125):'))
                if dev < 0:
                    dev = 0xff + dev + 1
                setBusServoDeviation(servo_id, dev, serialHandle)
            elif mode == 3:
                servo_id = int(input('servo id:'))
                saveBusServoDeviation(servo_id, serialHandle)
            # elif mode == 4:
            #     servo_id = int(input('servo id:'))
            #     min_pos = int(input('min pos(0-1000):'))
            #     max_pos = int(input('max pos(0-1000):'))
            #     setBusServoAngleLimit(servo_id, min_pos, max_pos)        
            elif mode == 5:
                servo_id = int(input('servo id:'))
                min_vin = int(input('min vin(4500-14000):'))
                max_vin = int(input('max vin(4500-14000):'))
                setBusServoVinLimit(servo_id, min_vin, max_vin, serialHandle)  
            elif mode == 6:
                servo_id = int(input('servo id:'))
                max_temp = int(input('temperature(0-85):'))
                setBusServoMaxTemp(servo_id, max_temp, serialHandle)  
            elif mode == 7:
                servo_id = int(input('servo id:'))
                status = int(input('status 0 or 1:'))
                unloadBusServo(servo_id, status, serialHandle) 
            elif mode == 4:
                print('''
    --------------------------
    1 id                
    2 dev               
    3 save_dev          
    4 help      
    5 voltage_range   
    6 temperature_warn
    7 lock              
    8 exit
    --------------------------''')
            elif mode == 8:
                break
            else:
                print('error mode')
        except KeyboardInterrupt:
            break