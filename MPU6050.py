"""
>>> This is the Module of "MPU6050" device which have combine a 3-axis gyroscope and a 3-axis accelerometer on the same
silicon together with an onboard Digital Motion Processor (DMP) capable of processing complex 9-axis MotionFusion algorithms.

>>> Variable Definations
     
    --> i2c              : I2C object variable 
    --> ID               : Enter the ID of the I2C port.
    --> Scl              : Serial Clock pin number.
    --> Sda              : Seria Data pin number.
    --> Upper            : Higher 8-bits of 16-bits data.
    --> Lower            : Lower 8-bits of 16-bits data. 
    --> MPU6050_address  : MPU6050 device address.
    --> num              : Enter value from 0 to 3 to get appropate data.

"""


#----------------------------Importing Modules----------------------------#
from machine import Pin,I2C             #Micropython Module 
import math                             #Mathematical Module   


#----------------------------MPU6050 Register Map----------------------------#
#Intial registers
PWR_MGMT = 0x6B              #Getting out from intial sleep mode
REQ_START = 0X00             #Request to start register

#Temperature registers
TEMP_OUT_H = 0x41            #Temperature 16-8 bit register
TEMP_OUT_L = 0x42            #Temperature 7-0 bit register

#Accelerometer registers
ACCEL_CONFIG   = 0X1C        #Set the register configure accelerometer
ACCEL_2GFORCE  = 0X00        #set the register to Get gfore value (+/-2g)
ACCEL_4GFORCE  = 0X10        #set the register to Get gfore value (+/-4g)
ACCEL_8GFORCE  = 0X20        #set the register to Get gfore value (+/-8g)
ACCEL_16GFORCE = 0X30        #set the register to Get gfore value (+/-16g)

ACCEL_XOUT_UB = 0X3B         #Accelerometer x-axis 16-8 bit register
ACCEL_XOUT_LB = 0X3C         #Accelerometer x-axis 7-0 bit register
ACCEL_YOUT_UB = 0X3D         #Accelerometer y-axis 16-8 bit register
ACCEL_YOUT_LB = 0X3E         #Accelerometer y-axis 7-0 bit register
ACCEL_ZOUT_UB = 0X3F         #Accelerometer z-axis 16-8 bit register
ACCEL_ZOUT_LB = 0X40         #Accelerometer z-axis 7-0 bit register

#Gyroscope registers
GYRO_CONFIG  = 0X1B          #Set the register configure gyroscop
GYRO_250DPS  = 0X00          #Set the register to set dps value (250dps)
GYRO_500DPS  = 0X10          #Set the register to set dps value (500dps)
GYRO_1000DPS = 0X20          #Set the register to set dps value (1000dps)
GYRO_2000DPS = 0X30          #Set the register to set dps value (2000dps)

GYRO_XOUT_UB = 0X43          #Gyroscope x-axis 16-8 bit register
GYRO_XOUT_LB = 0X44          #Gyroscope x-axis 7-0 bit register
GYRO_YOUT_UB = 0X45          #Gyroscope y-axis 16-8 bit register
GYRO_YOUT_LB = 0X46          #Gyroscope y-axis 7-0 bit register
GYRO_ZOUT_UB = 0X47          #Gyroscope z-axis 16-8 bit register
GYRO_ZOUT_LB = 0X48          #Gyroscope z-axis 16-8 bit register


#----------------------------Sensitivity Parameters----------------------------#
TEMP_LSBC = 340.0            # 
TEMP_OFFSET = 36.53          #

ACCEL_Sens_2G  = 16384.0     #+/-2G force value (LSB/G)
ACCEL_Sens_4G  =  8192.0     #+/-4G force value (LSB/G)
ACCEL_Sens_8G  =  4096.0     #+/-8G force value (LSB/G)
ACCEL_Sens_16G =  2048.0     #+/-16G force value (LSB/G)

GYRO_Sens_250DPS  = 131.0    #250DPS gyro value
GYRO_Sens_500DPS  = 65.5     #500DPS gyro value
GYRO_Sens_1000DPS = 32.8     #1000DPS gyro value
GYRO_Sens_2000DPS = 16.4     #2000DPS gyro value

RadtoDeg = 57.2958           #Radian to Degree           


#----------------------------List----------------------------#
ACCEL_GFORCE = [ACCEL_2GFORCE, ACCEL_4GFORCE, ACCEL_8GFORCE, ACCEL_16GFORCE]
ACCEL_Sens   = [ACCEL_Sens_2G, ACCEL_Sens_4G, ACCEL_Sens_8G, ACCEL_Sens_16G]

GYRO_DPS     = [GYRO_250DPS, GYRO_500DPS, GYRO_1000DPS, GYRO_2000DPS] 
GYRO_Sens    = [GYRO_Sens_250DPS, GYRO_Sens_500DPS, GYRO_Sens_1000DPS, GYRO_Sens_2000DPS]    
    

#----------------------------Adding two 8-bits data----------------------------#
def register_combine(upper, lower):
    if not upper[0] & 0x80:
        return upper[0] << 8 | lower[0]
    return -((upper[0] ^ 255) << 8) |  (lower[0] ^ 255) + 1
    #print("register_combine is Sucessfully run!!!")


#----------------------------I2C Communication Hardware confugration----------------------------#
def I2C_Inti(Id, Sda, Scl):
    i2c = I2C(0, sda = Pin(Sda), scl = Pin(Scl))
    #print(i2c)
    return i2c


#----------------------------I2C device Address Request----------------------------#
def I2C_Address(i2c):
    mpu6050_address = (i2c.scan()[0])
    #print("MPU6050 Address:", hex(mpu6050_address))
    return mpu6050_address


#----------------------------MPU6050 instantion----------------------------#
def MPU6050_Inti(i2c, mpu6050_address):
    i2c.writeto_mem(mpu6050_address, PWR_MGMT, bytes([0]))     
    i2c.writeto_mem(mpu6050_address, REQ_START, bytes([0]))
    #print("MPU6050_Inti is Sucessfully run!!!")
    return 0


#----------------------------MPU6050 Accelerometer confugration----------------------------#
def MPU6050_AccelConfig(i2c,mpu6050_address,num):
    i2c.writeto_mem(mpu6050_address, ACCEL_CONFIG, bytes([0]))
    i2c.writeto_mem(mpu6050_address, ACCEL_GFORCE[num], bytes([0]))
    #print("MPU6050_Accel_Config is Sucessfully run!!!")
    return 0


#----------------------------MPU6050 Gyroscope confugration----------------------------#
def MPU6050_GyroConfig(i2c,mpu6050_address, num):
    i2c.writeto_mem(mpu6050_address, GYRO_CONFIG, bytes([0]))
    i2c.writeto_mem(mpu6050_address, GYRO_DPS[num], bytes([0]))
    #print("MPU6050_Gyro_Config is Sucessfully run!!!")
    return 0


#----------------------------MPU6050 Temperature data reading----------------------------#
def MPU6050_TempRead(i2c, mpu6050_address):
    temp_h = i2c.readfrom_mem(mpu6050_address, TEMP_OUT_H, 1)
    temp_l = i2c.readfrom_mem(mpu6050_address, TEMP_OUT_L, 1)
    
    temp = (register_combine(temp_h, temp_l)/TEMP_LSBC) + TEMP_OFFSET
    #print(temp)
    return temp
 
 
#----------------------------MPU6050 Accelerometer dara reading----------------------------#
def MPU6050_AccelRead(i2c,mpu6050_address, num):
    accel_x_h = i2c.readfrom_mem(mpu6050_address, ACCEL_XOUT_UB, 1)
    accel_x_l = i2c.readfrom_mem(mpu6050_address, ACCEL_XOUT_LB, 1)
    accel_y_h = i2c.readfrom_mem(mpu6050_address, ACCEL_YOUT_UB, 1)
    accel_y_l = i2c.readfrom_mem(mpu6050_address, ACCEL_YOUT_LB, 1)
    accel_z_h = i2c.readfrom_mem(mpu6050_address, ACCEL_ZOUT_UB, 1)
    accel_z_l = i2c.readfrom_mem(mpu6050_address, ACCEL_ZOUT_LB, 1)
    
    accel_x = register_combine(accel_x_h, accel_y_l)/ACCEL_Sens[num]
    accel_y = register_combine(accel_y_h, accel_y_l)/ACCEL_Sens[num]
    accel_z = register_combine(accel_z_h, accel_z_l)/ACCEL_Sens[num]
    #print(accel_x, accel_y, accel_z)
    return accel_x, accel_y, accel_z


#----------------------------MPU6050 Gyroscope dara reading----------------------------#
def MPU6050_GyroRead(i2c, mpu6050_address, num):
    gyro_x_h = i2c.readfrom_mem(mpu6050_address, GYRO_XOUT_UB, 1)
    gyro_x_l = i2c.readfrom_mem(mpu6050_address, GYRO_XOUT_LB, 1)
    gyro_y_h = i2c.readfrom_mem(mpu6050_address, GYRO_YOUT_UB, 1)
    gyro_y_l = i2c.readfrom_mem(mpu6050_address, GYRO_YOUT_LB, 1)
    gyro_z_h = i2c.readfrom_mem(mpu6050_address, GYRO_ZOUT_UB, 1)
    gyro_z_l = i2c.readfrom_mem(mpu6050_address, GYRO_ZOUT_LB, 1)
    
    gyro_x = register_combine(gyro_x_h, gyro_y_l)/GYRO_Sens[num]
    gyro_y = register_combine(gyro_y_h, gyro_y_l)/GYRO_Sens[num]
    gyro_z = register_combine(gyro_z_h, gyro_z_l)/GYRO_Sens[num]
    #print(gyro_x,gyro_y,gyro_z)
    return gyro_x,gyro_y,gyro_z


#----------------------------Angle from AccelRead----------------------------#
def Angle_Accel(i2c,mpu6050_address,num):
    Read = MPU6050_AccelRead(i2c,mpu6050_address, num)
    x_axis = (math.atan(Read[0]/(math.sqrt(math.pow(Read[1],2))+math.pow(Read[2],2))))* RadtoDeg
    y_axis = (math.atan(Read[1]/(math.sqrt(math.pow(Read[0],2))+math.pow(Read[2],2))))* RadtoDeg
    z_axis = (math.atan((math.sqrt(math.pow(Read[0],2))+math.pow(Read[1],2))/Read[2]))* RadtoDeg
    
    return x_axis,y_axis,z_axis
    

#----------------------------Accel error calculation----------------------------#
def Accel_errorRead(i2c,mpu6050_address,num):
    Accel_error = 0
    Acc_angle_error_x = 0
    Acc_angle_error_y = 0
    Acc_angle_error_z = 0
    
    if Accel_error==0:
        for i in range(200):
            d = MPU6050_AccelRead(i2c,mpu6050_address, num)
            Acc_angle_error_x = Acc_angle_error_x + (math.atan(d[0]/(math.sqrt(math.pow(d[1],2))+math.pow(d[2],2))))
            Acc_angle_error_y = Acc_angle_error_y + (math.atan(d[1]/(math.sqrt(math.pow(d[0],2))+math.pow(d[2],2))))
            Acc_angle_error_z = Acc_angle_error_z + (math.atan((math.sqrt(math.pow(d[0],2))+math.pow(d[1],2))/d[2]))
            
            if i == 199:
                Acc_angle_error_x = Acc_angle_error_x/200
                Acc_angle_error_y = Acc_angle_error_y/200
                Acc_angle_error_z = Acc_angle_error_z/200
                Accel_error=1
                
    return Acc_angle_error_x,Acc_angle_error_y,Acc_angle_error_z
 
 
#----------------------------Gyro error calculation----------------------------#
def Gyro_errorRead(i2c,mpu6050_address,num):
    Gyro_error = 0
    Gyro_error_x = 0
    Gyro_error_y = 0
    Gyro_error_z = 0
    
    if Gyro_error == 0:
        for i in range(200):
            d = MPU6050_GyroRead(i2c, mpu6050_address, num)
            Gyro_error_x = Gyro_error_x + d[0]
            Gyro_error_y = Gyro_error_y + d[1]
            Gyro_error_z = Gyro_error_z + d[2]
            
            if i == 199:
                Gyro_error_x = Gyro_error_x/200
                Gyro_error_y = Gyro_error_y/200
                Gyro_error_z = Gyro_error_z/200
                Gyro_erro = 1
            
    return Gyro_error_x,Gyro_error_y,Gyro_error_z