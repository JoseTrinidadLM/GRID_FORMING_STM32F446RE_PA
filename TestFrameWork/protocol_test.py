import serial
import string
import time

ser = serial.Serial()
ser.baudrate = 2200000
ser.port = 'COM6'
ser.timeout = None
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE

ser.open()


def readPacket():
    while(True):
        s = ser.read(1)
        if(chr(s[0]) == '$'):
            s = ser.read(1)
            if(chr(s[0]) != 'S'):
                s = ser.read(5)
            else:
                s = ser.read(3)
            break
    return s

def readHeartBeat():
    ser.reset_input_buffer()
    while(True):
        s = ser.read(1)
        if(chr(s[0]) == '$'):
            s = ser.read(1)
            if(chr(s[0]) == 'S'):
                s = ser.read(5) 
                break
    return s

def readTelemetry():
    while(True):
        s = ser.read(1)
        if(chr(s[0]) == '$'):
            s = ser.read(1)
            if(chr(s[0]) != 'S'):
                s = ser.read(5)
                break
    return s

def testCommands():
    print("Testing Command: System ON\n")
    s = b'$'+b'X'+b'n'+bytes.fromhex('01')
    ser.write(s)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    print("Testing Command: System Closed Loop\n")
    s = b'$'+b'X'+b'n'+bytes.fromhex('04')
    ser.write(s)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    print("Testing Command: System Open Loop\n")
    s = b'$'+b'X'+b'n'+bytes.fromhex('03')
    ser.write(s)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    print("Testing Command: System OFF\n")
    s = b'$'+b'X'+b'n'+bytes.fromhex('02')
    ser.write(s)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')

print(ser.name)
while(True):
    if(ser.is_open):
        testCommands()
        break
