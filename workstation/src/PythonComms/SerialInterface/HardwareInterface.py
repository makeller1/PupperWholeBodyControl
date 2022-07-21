import serial
import msgpack
import numpy as np
from enum import Enum

class SerialReaderState(Enum):
    WAITING_BYTE1 = 0
    WAITING_BYTE2 = 1
    READING_LENGTH_BYTE1 = 2
    READING_LENGTH_BYTE2 = 3
    READING = 4


class NonBlockingSerialReader:
    def __init__(self, serial_handle, start_byte=69, start_byte2=69):
        self.start_byte = start_byte
        self.start_byte2 = start_byte2
        self.serial_handle = serial_handle
        self.byte_buffer = b""
        self.mode = SerialReaderState.WAITING_BYTE1
        self.message_length = -1

    def chew(self):
        while True:
            raw_data = self.serial_handle.read(1024)
            if not raw_data:
                break
            for in_byte in raw_data:
                if self.mode == SerialReaderState.WAITING_BYTE1:
                    if in_byte == self.start_byte:
                        self.mode = SerialReaderState.WAITING_BYTE2
                elif self.mode == SerialReaderState.WAITING_BYTE2:
                    if in_byte == self.start_byte2:
                        self.mode = SerialReaderState.READING_LENGTH_BYTE1
                    else:
                        self.mode = SerialReaderState.WAITING_BYTE1
                elif self.mode == SerialReaderState.READING_LENGTH_BYTE1:
                    self.message_length = int(in_byte) * 256
                    self.mode = SerialReaderState.READING_LENGTH_BYTE2
                elif self.mode == SerialReaderState.READING_LENGTH_BYTE2:
                    self.message_length += int(in_byte)
                    self.mode = SerialReaderState.READING
                elif self.mode == SerialReaderState.READING:
                    self.byte_buffer += bytes([in_byte])
                    if len(self.byte_buffer) == self.message_length:
                        self.message_length = -1
                        self.mode = SerialReaderState.WAITING_BYTE1
                        temp = self.byte_buffer
                        self.byte_buffer = b""
                        return temp
        return None


class HardwareInterface:
    def __init__(self, port, baudrate=115200, start_byte=0x00):
        self.start_byte = start_byte
        self.serial_handle = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
            write_timeout=0
        )
        self.reader = NonBlockingSerialReader(self.serial_handle)

    def set_max_current(self, max_current):
        print("Setting MAX CURRENT: ", max_current)
        self.send_dict({"max_current": max_current})
        
    def get_robot_states(self): # mathew
        # decoded_data:
        # ts : current time 
        # pos : position
        # cur : current
        # pref : reference pos
        # lcur : ?
        decoded_data = None
        while True:
            data = self.reader.chew()
            if not data:
                return decoded_data
            try:
                decoded_data = msgpack.unpackb(data)
                # print("Decoded data: \n",decoded_data)
                return decoded_data

            except ValueError as e:
                print(e)

    def log_incoming_data(self, log_file):
        decoded_data = None
        while True:
            data = self.reader.chew()
            if not data:
                return decoded_data
            try:
                decoded_data = msgpack.unpackb(data)
                data_str = ""
                for value in decoded_data.values():
                    if type(value) == list:
                        for v in value:
                            data_str += "%0.3f" % v + ","
                    else:
                      data_str += str(value) + ","
                log_file.write(data_str[:-1] + "\n")
            except ValueError as e:
                print(e)


    def write_logfile_header(self, logfile):
        header = "Timestamp,"
        for attribute in [
            "Position",
            "Velocity",
            "Current",
            "PositionRef",
            "LastCommand",
        ]:
            for i in range(12):
                header += f"{attribute}_{i},"
        header += "\n"
        logfile.write(header)

    def send_dict(self, dict_msg):
        payload = msgpack.packb(dict_msg, use_single_float=True)
        start_sequence = bytes([self.start_byte, len(payload)])
        self.serial_handle.write(start_sequence + payload)

    def set_trq_mode(self):
        self.send_dict({"trq_mode": True})

    def run_calibration(self):
        # Run IMU calibration
        self.send_dict({"calibrate": True})

    def zero_motors(self):
        self.send_dict({"zero": True})

    def set_torque(self, torques_list): # - mathew
        """Sends desired motor torques to the Teensy ( Note: currently just currents ) -mathew

        Parameters
        ----------
        torques_list : list of size 12
            Desired torques on each motor [Nm]
        
        With order:
                [FR hip, FR shoulder, FR knee,
                 FL hip, FL shoulder, FL knee,
                 BR hip, BR shoulder, BR knee,
                 BL hip, BL shoulder, BL knee]
        """
        #torques_list = torques.flatten("F").tolist() # Change to recieve list already
        self.send_dict({"trq": torques_list})
