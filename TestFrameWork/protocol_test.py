import serial
import string
import time
import struct

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
        if(ser.is_open):
            while(True):
                s = ser.read(1)
                if(chr(s[0]) == '$'):
                    s = ser.read(1)
                    if(chr(s[0]) != 'S'):
                        s = ser.read(5)
                    else:
                        s = ser.read(3)
                    break
            break
    return s

def readHeartBeat():
    while(True):
        print("Checking if port is open to receive heartbeat\n")
        if(ser.is_open):
            print("Port open")
            ser.reset_input_buffer()
            while(True):
                s = ser.read(1)
                if(chr(s[0]) == '$'):
                    t = ser.read(1)
                    if(chr(t[0]) == 'S'):
                        len = struct.unpack('!b', ser.read(1))[0]
                        s = struct.unpack('!b', ser.read(1))[0]
                        f = struct.unpack('!b', ser.read(1))[0]
                        r = ser.read(2)
                        break
            break
    return len, t, s, f, r

def readTelemetry():
    while(True):
        if(ser.is_open):
            while(True):
                s = ser.read(1)
                if(chr(s[0]) == '$'):
                    t = ser.read(1)
                    if(chr(t[0]) != 'S'):
                        len = struct.unpack('!b', ser.read(1))[0]
                        v = struct.unpack('!f', ser.read(4))
                        break
            break
    return len, t, v

def command(command):
    while(True):
        print("Checking if port is open to send command\n")
        if(ser.is_open):
            print("Testing Command: System " +command+ "\n")
            s = b'$'+b'X'+bytes.fromhex(command)
            ser.write(s)
            break

def testCommands():
    command('01')
    time.sleep(2)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('04')
    time.sleep(2)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('03')
    time.sleep(2)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')
    command('02')
    time.sleep(2)
    s = readHeartBeat()
    print("Status: "+str(s)+'\n')

def testTelemetry():
    while(True):
        if(ser.is_open):
            len, t, value = readTelemetry()
            print("Packet: "+str(t)+"Long: "+f"{len}| {value} \n")

def testHeartBeat():
    while(True):
        if(ser.is_open):
            len, t, s, f, r = readHeartBeat()
            print("Status: "+str(t)+" Long: "+f"{len}| Status {s} Frequency {f} \n")

if __name__ == "__main__":
    usartConfig()
    command('01')
    time.sleep(2)
    testHeartBeat()
