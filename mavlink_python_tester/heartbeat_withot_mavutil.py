import ctu_sr_illustria_2_py_lib as dialect
import time

import serial

MAVlinkSystemID = 3
MAVlinkComponentID = 5

#helper class to encapsulate one target system on a mavlink bus
class MAVLinkTarget(object):
    SystemId = 0
    ComponentId = 0
    
    def __init__(self, sysId: int, comId: str) -> None:
        self.SystemId = sysId
        self.ComponentId = comId


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




# Example usage:
port_name = '/dev/tty.usbserial-A50285BI'  # Change this to match your serial port name
baud_rate = 57600  # Change the baud rate if needed

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
print("sent HEARTBEAT")

RequestProtocol = System.command_long_encode(targetSystem.SystemId,targetSystem.ComponentId,dialect.MAV_CMD_REQUEST_MESSAGE,0,dialect.MAVLINK_MSG_ID_PROTOCOL_VERSION,0,0,0,0,0,0)

while 1:
    System.send(RequestProtocol)
    print("sent MAV_CMD_REQUEST_MESSAGE")
    time.sleep(1.5)


closeSerial(serialPort)