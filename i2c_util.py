import os
import subprocess
import Jetson.GPIO as GPIO


# def name_fun(param: int) -> void:
    

def detect_i2c(port):
    subprocess.call(["i2cdetect", "-y", "-r", str(port)])

def write_i2c_reg(port, addr, reg, data):
    subprocess.call(["i2cset", "-y", "-f", str(port), hex(addr), hex(reg), hex(data)])


def read_i2c_reg(port, addr, reg) -> int:
    data = subprocess.check_output(["i2cget", "-y", "-f", str(port), hex(addr), hex(reg)])
    # convert b'int\n' to int
    return data

detect_i2c(2)
print(read_i2c_reg(2, 0x50, 0x01))
GPIO.setmode(GPIO.BOARD)
mode = GPIO.getmode()


print(mode)
print(GPIO.model)

# i2c = board.I2C()
# bme280 = adafruit_bme280.adafruit_bme280_I2C(i2c)

