import serial
import string
import time

ser = serial.Serial()

def usartConfig():
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

def command(command):
    print("Testing Command: System " +command+ "\n")
    s = b'$'+b'X'+b'n'+bytes.fromhex(command)
    ser.write(s)

def testCommands():
    command('01')
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('04')
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('03')
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('02')
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')

if __name__ == "__main__":
    usartConfig()
    print(ser.name)
    while(True):
        if(ser.is_open):
            testCommands()
            break
