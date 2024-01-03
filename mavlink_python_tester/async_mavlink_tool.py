import asyncio
import serial_asyncio
import threading
import sys
import serial
import time

import ctu_sr_illustria_2_py_lib as dialect

# Example usage:
port_name = '/dev/tty.usbserial-A50285BI'  # Change this to match your serial port name
baud_rate = 57600  # Change the baud rate if needed

MAVlinkSystemID = dialect.MAV_SYS_COMMAND_CENTER
MAVlinkComponentID = dialect.MAV_COMP_ID_CC_COMPUTER

#helper class to encapsulate one target system on a mavlink bus
class MAVLinkTarget(object):
    SystemId = 0
    ComponentId = 0
    
    def __init__(self, sysId: int, comId: str) -> None:
        self.SystemId = sysId
        self.ComponentId = comId

class SerialPortReadThead(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""
    stopped = False

    def __init__(self, serial_port):
        self.serial_port = serial_port
        threading.Thread.__init__(self)
        self.thread = threading.Thread(target=self.process)
        self.daemon = True

    def process(self):
        while not self.stopped:
            recievedMsg = readIncommingMessages(serial_port=self.serial_port)
            print("Received:",recievedMsg)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopped = True
        time.sleep(0.1)
        self.thread.join()

def openSerial(port_name, baud_rate):
    try:
        # Open the serial port
        ser = serial.Serial(port_name, baud_rate)
        if ser.isOpen():
            print(f"Serial port {port_name} opened successfully.")
            
            return ser
            
        else:
            print(f"Failed to open serial port {port_name}.")

    except serial.SerialException as e:
        print(f"Serial Exception: {e}")

def closeSerial(serial_port):
    if serial_port.isOpen():
        serial_port.close()
        print("Serial port closed.")

def sendBytesArray(serial_port, bytes_to_send):
    try:
       
        if serial_port.isOpen():
        
            # Send the data
            serial_port.write(bytes_to_send)
            print(f"Sent: {bytes_to_send}")
            
        else:
            print(f"Serial port is closed: {port_name}.")

    except serial.SerialException as e:
        print(f"Serial Exception: {e}")

def readIncommingMessages(serial_port):
    while True:
        byte = serial_port.read()
        if byte is not None:
            message = System.parse_char(byte)
            if message is not None:
                return message

def waitForHeartbeat(serial_port):
    recievedHeartbeat = False
    print("Waiting for a heartbeat")
    while not recievedHeartbeat:
        msg = readIncommingMessages(serial_port);
        if msg.get_msgId() == dialect.MAVLINK_MSG_ID_HEARTBEAT:
            print("Heartbeat recived")
            target = MAVLinkTarget(msg.get_srcSystem(),msg.get_srcComponent())
            return target

def _readMAVLinkMessagesBlocking(serial_port):
    while not self.stopped():
        recievedMsg = readIncommingMessages(serial_port=serial_port)
        print("Received:",recievedMsg)

serialPort = openSerial(port_name,baud_rate)

#setup system with a connection
System = dialect.MAVLink(file=serialPort,srcSystem=MAVlinkSystemID,srcComponent=MAVlinkComponentID)

#wait until heartbeat of the target is detected
targetSystem = waitForHeartbeat(serial_port=serialPort)
print("Recieved Heatbeat from System:",targetSystem.SystemId," Component:",targetSystem.ComponentId)

#send the systems heartbeat
HB = System.heartbeat_encode(dialect.MAV_TYPE_ROCKET,
                                            dialect.MAV_AUTOPILOT_INVALID,
                                            dialect.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                                0,
                                                dialect.MAV_STATE_STANDBY);
System.send(HB)
print("Send: HEARTBEAT")

Reader = SerialPortReadThead(serialPort);
Reader.start()

try:
    RequestProtocol = System.command_long_encode(targetSystem.SystemId,targetSystem.ComponentId,dialect.MAV_CMD_REQUEST_MESSAGE,0,dialect.MAVLINK_MSG_ID_PROTOCOL_VERSION,0,0,0,0,0,0)
    System.send(RequestProtocol)
    print("Send: MAV_CMD_REQUEST_MESSAGE -> PROTOCOL_VERSION")


    RequestLEDstatus = System.command_long_encode(targetSystem.SystemId,targetSystem.ComponentId,dialect.MAV_CMD_SET_LEDS,0,1,0,1,0,0,0,0);
    System.send(RequestLEDstatus)
    print("Send: MAV_CMD_SET_LEDS")

    time.sleep(1);

    RequestLEDstatus = System.command_long_encode(targetSystem.SystemId,targetSystem.ComponentId,dialect.MAV_CMD_SET_LEDS,0,1,1,1,0,0,0,0);
    System.send(RequestLEDstatus)
    print("Send: MAV_CMD_SET_LEDS")

    while 1:
        time.sleep(1.5)   

except KeyboardInterrupt:
    Reader.stop();
    print("threads successfully closed")


