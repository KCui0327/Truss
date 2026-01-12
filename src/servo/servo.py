import time
import servo_fns

""" servo_id = BoardTemp.getBusServoID()
print(f"servo_id{servo_id}")
Pulse = BoardTemp.getBusServoPulse(servo_id) # 获取舵机的位置信息(obtain position information of the servo)
Temp = BoardTemp.getBusServoTemp(servo_id) # 获取舵机的温度信息(obtain temperature information of the servo)
Vin = BoardTemp.getBusServoVin(servo_id) # 获取舵机的电压信息(obtain voltage information of the servo)
print(f"Pulse{Pulse}")
print(f"Temp{Temp}")
print(f"Vin{Vin}") """

def angle_to_pos(angle):
    return int(angle * (1000/240))


#Servo 1: 125 degrees
#Servo 2: 130 degrees
angle1 = angle_to_pos(125)
angle2 = angle_to_pos(130)
servo_fns.setBusServoPulse(1, angle1, 5000) # 6号舵机转到800位置，用时1000ms(Rotate servo 1 to position 800 in 1000ms)
servo_fns.setBusServoPulse(2, angle2, 5000) # 6号舵机转到800位置，用时1000ms(Rotate servo 1 to position 800 in 1000ms)
time.sleep(5)
""" BoardTemp.setBusServoPulse(2, angle2, 5000) # 6号舵机转到800位置，用时1000ms(Rotate servo 1 to position 800 in 1000ms)
time.sleep(5) """

#BoardTemp.setBusServoPulse(6, angle, 5000) # 6号舵机转到800位置，用时1000ms(Rotate servo 1 to position 800 in 1000ms)
#time.sleep(5) # 延时0.5s(delay for 0.5s)
#angle = BoardTemp.getBusServoPulse(servo_id)
#print(f"angle:{angle}")

""" BoardTemp.setBusServoPulse(servo_id, 200, 5000) # 6号舵机转到200位置，用时1000ms(Rotate servo 1 to position 200 in 1000ms)
time.sleep(5) # 延时0.5s(delay for 0.5s)
angle = BoardTemp.getBusServoPulse(servo_id)
print(f"angle:{angle}") """

""" angle = angle_to_pos(120)
print(angle)

for i in range(3,7):
    BoardTemp.setBusServoPulse(i, angle, 3000) # 6号舵机转到800位置，用时1000ms(Rotate servo 1 to position 800 in 1000ms)
    time.sleep(5)
    angle = BoardTemp.getBusServoPulse(i)
    print(f"{i} angle:{angle}") """


""" def getBusServoStatus(servo_id):
    Pulse = Board.getBusServoPulse(servo_id) # 获取舵机的位置信息(obtain position information of the servo)
    Temp = Board.getBusServoTemp(servo_id) # 获取舵机的温度信息(obtain temperature information of the servo)
    Vin = Board.getBusServoVin(servo_id) # 获取舵机的电压信息(obtain voltage information of the servo)
    print('Pulse: {}\nTemp:  {}\nVin:   {}\nid:   {}\n'.format(Pulse, Temp, Vin, servo_id)) # 打印状态信息(print status information)
    time.sleep(0.5) # 延时方便查看(delay for easier check)

   
Board.setBusServoPulse(servo_id, 500, 1000) # 1号舵机转到500位置用时1000ms(Rotate servo 1 to position 500 in 1000ms)
time.sleep(1) # 延时1s(delay for 1s)
getBusServoStatus(servo_id) # 读取总线舵机状态(read bus servo status) """
 
