from machine import Pin,I2C

i2c = machine.I2C(0, scl = Pin(9), sda = Pin(8), freq=400000)

#print("I2C adderess: " +hex(i2c.scan()[0]))