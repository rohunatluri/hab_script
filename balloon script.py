from __future__ import print_function

import serial
#import RPi.GPIO as GPIO
from time import sleep

RADIO_CALLSIGN = "HAB"
SERIAL_PORT = "/dev/ttyUSB0" # COMX on Windows, /dev/SERIAL_PORT on Linux
BAUDRATE = 9600
SERIAL_INPUT_TIMEOUT = 5
SERIAL_OUTPUT_TIMEOUT = 2

GPS_LOG_FILE_LOCATION = r"logData/gps_log.txt"
SCRIPT_LOG_FILE_LOCATION = r"logData/balloon_script_log.txt"
TELEMETRY_FILE_BASE = r"logData/dictionary.txt"

GPS_SERIAL_PORT = "/dev/ttyAMA0"
GPS_BAUDRATE = 4800

SHORT_SLEEP_DURATION = 0.05
LONG_SLEEP_DURATION = 1
HEARTBEAT_INTERVAL = 100

def runBalloonScript():
    scriptLog = open(SCRIPT_LOG_FILE_LOCATION, "a")
    gpsLog = open(GPS_LOG_FILE_LOCATION, "a")
    telemetryFile = open(TELEMETRY_FILE_BASE, "r")
    messageToSend = "NULL_MESSAGE"
    
    while(True):
        sleep(2)
        # send
        try:
            messageToSend = formatData(gpsSerialInput(), getSensorData())
            if (messageToSend != "INVALID DATA"):
                print("Sending: " + messageToSend)
                sendSerialOutput(messageToSend)
            gpsLog.writelines(messageToSend)
        except:
            messageToSend = ""
            messageToSend += telemetryFile.readline()
            print("File: " + messageToSend)
            sendSerialOutput(messageToSend)
            
        # receive
        messageReceived = radioSerialInput()
        
        # act on receive
        handleMessage(messageReceived)

        scriptLog.writelines(messageReceived)

    
def handleMessage(message):
    for line in message:
        if not ("No messages received." in line):
            if ("releaseBalloonNow" in line):
                releaseBalloon()
                break
            else:
                print("Received unknown: {0}".format(line))
                print("Saving message to file")
                print("Sending NO ACK message back to GS")

def formatData(gpsString, dataString):
    finalDataString = "INVALID DATA"
    currGpsString = ""
    
    try:
        gpsSplit = gpsString.split(",")
        currGpsString = "{},{},{},{}".format(gpsSplit[1][:6],
                                             gpsSplit[2],
                                             gpsSplit[4],
                                             gpsSplit[9])

    except:
        currGpsString = "0,0,0,0"

    if (dataString == "NO DATA" and gpsSplit == "0,0,0,0"):
       print ("INVALID DATA STRINGS GIVEN")
    else:
        finalDataString = currGpsString + dataString + "\n"

    return finalDataString

def getSensorData():
    return ",addVoltage,addInside,addOutside,addBattery"

def gpsSerialInput():
    messageReceived = "NO_GPS_DATA\n"
    serialInput = ""
    retries = 5
    iterationsToWait = 100
    
    try:
        ser=serial.Serial(port = GPS_SERIAL_PORT, baudrate = GPS_BAUDRATE, timeout = SERIAL_INPUT_TIMEOUT)
        
        sleep(0.5)
        
        while (retries > 0 and iterationsToWait > 0):
            if (ser.inWaiting() > 0):                # If there's a buffer for us to read
                serialInput = ser.readline(1024)
                if (serialInput[:6] == r"$GPGGA"):    # Makes sure this is the line we want
                    break                            # This is our stop
                else:
                    # print("Discarding unused data: " + serialInput)
                    serialInput = ""                # This is not the data we're looking for
                    retries -= 1
            else:
                iterationsToWait -= 1
                
        
    except:
        print("Unable to read serial input: {0} at baud {1}".format(GPS_SERIAL_PORT, GPS_BAUDRATE))
    
    if (retries > 0 and iterationsToWait > 0):        # We found what we wanted
        messageReceived = serialInput
    try:
        ser.close()
    except:
        print("Unable to close serial port.")
    
    return messageReceived


def radioSerialInput():
    message = ["No messages received."]
    serialInput = []
    retries = 5
    
    try:
        ser=serial.Serial(port = SERIAL_PORT, baudrate = BAUDRATE, timeout = 3)
             
        sleep(0.5)   
                
        while (ser.inWaiting() > 0 and retries > 0):
            serialInput.append(ser.readline(1024))
            retries -= 1
         
        if (len(serialInput) > 0):
            message = serialInput
    except:
        print("Unable to read serial input: {0} at baud {1}".format(SERIAL_PORT, BAUDRATE))
    
    try:
        ser.close()
    except:
        print("Unable to close serial port.")
    for line in message:
        print("Received: " + line)
    
    return message

def sendSerialOutput(line):
    success = False
    
    ser = None
    try:
        ser=serial.Serial(port = SERIAL_PORT, baudrate = BAUDRATE, timeout = SERIAL_OUTPUT_TIMEOUT)
        line = ser.write("HAB:" + line + "\n")
        success = True
        
    except:
        print("Unable to write to the radio's serial port")
        success = False
    
    try:
        ser.close()
    except:
        print("Unable to close serial port")
    
    return success


def transmitTelemetry():
#     Transmit Telemetry
#         Open file with current timestamp
#         Transmit entire contents in 5s increments
#         Close file
#         Return to main loop
    return None

def changeConfiguration(config = "slave"):
#     Change configuration
#         Run balloon script
#             Set to master
#             Spam confirmation
#             If response received:
#                 Keep configuration
#             If no response:
#                 Revert to slave
#         Helical set to slave
#         On loss of network, set chase's helical to master
    return None

def switchToAudioRelay():
    return None

def switchToTextRelay():
    return None

def releaseBalloon():
    sleep(4)
    print("Activating balloon release mechanism")
    print("BRM is activated")
    sendSerialOutput("\nBRM is activated\n")
    print("BRM successfully released")
    sendSerialOutput("\nBRM is released\n")
    sendSerialOutput("\nMOCK: BRM Activated\n")
#     GPIO.setmode(GPIO.BOARD)
#     
#     GPIO.setup(11, GPIO.OUT)
#     
#     GPIO.output(11,0)
#     
#     password = raw_input()
#     
#     while True:
#         if str(password) == 'SSAGhabRELEASE':
#             zero = time.time()
#             while time.time() - zero < 9001:
#                 GPIO.output(11,1)
#             break
#         else:
#             password = raw_input()
#     GPIO.cleanup()
    
    '''
    NOTES
    
    Orange wire must be connected to pin 5 of MOSFET
    Yellow wire must be connected to pin 7 of MOSFET
    If raspi pin is high, motor will retract
    If raspi pin is low, motor will expand
    
    '''

    return None

def transmitPicture():
#     Get picture
#         ls -> get last item
#         Send item
#             Timeout - 10s
#         Return to normal broadcast
    return None

def reportStateOfHealth():
#     Diagnose errors
#         Check for radios on network
#         Report filesystem
#         Return log
#         Return disk usage
#         Return battery usage
#         Return processor / memory usage
#         Return online components list
    return None

if __name__ == '__main__':
    runBalloonScript()
