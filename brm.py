import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

GPIO.output(11,0)

password = raw_input()

while True:
    if str(password) == 'SSAGhabRELEASE':
        zero = time.time()
        while time.time() - zero < 9001:
            GPIO.output(11,1)
        break
    else:
        password = raw_input()
GPIO.cleanup()


'''
NOTES

Orange wire must be connected to pin 5 of MOSFET
Yellow wire must be connected to pin 7 of MOSFET
If raspi pin is high, motor will retract
If raspi pin is low, motor will expand

'''
