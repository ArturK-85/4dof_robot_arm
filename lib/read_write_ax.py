import os
import sys, tty, termios
from dynamixel_sdk import *

import json_file

class ReadWriteServoData():
    """Read Dynamixel servo data."""

    def __init__(self,
       port='/dev/ttyUSB0',
       baudrate=1000000,
       protocol_version = 1.0,
       filename='data/arm_json/dxl_controll_table.json'):

                 self.port_num = PortHandler(port)
                 # Initialize PortHandler instance
                 # Set the port path
                 # Get methods and members of
                 # PortHandlerLinux or PortHandlerWindows

                 self.portHandler = PortHandler(port)
                 # Initialize PacketHandler instance
                 # Set the protocol version
                 # Get methods and members of
                 # Protocol1PacketHandler or Protocol2PacketHandler

                 self.packetHandler = PacketHandler(protocol_version)

                 # Open port
                 if self.portHandler.openPort():
                     print("Succeeded to open the port")
                 else:
                     print("Failed to open the port")
                     print("Press any key to terminate...")
                     getch()
                     quit()

                 if self.portHandler.setBaudRate(baudrate):
                     print("Succeeded to change the baudrate")
                 else:
                     print("Failed to change the baudrate")
                     print("Press any key to terminate...")
                     getch()
                     quit()

                 self.dxl_controll_table = json_file.load(filename)

    def get_dxl_controll_addr(self, function_name):
        """ This function returns code from the control table after
            providing the function name as argument """

        dxl_controll_addr = self.dxl_controll_table.get(function_name, 'Error')

        return dxl_controll_addr

    def write_data(self, ax_addr, data, dxl_id):
        """ Write data to servo, wrtie_data(address, id) """

        ax_addr = self.get_dxl_controll_addr(ax_addr)

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                                   self.portHandler, dxl_id,
                                   ax_addr, data)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def read_data(self, ax_addr, dxl_id):
        """ Read data from servo. read_data(address, id) """

        ax_addr = self.get_dxl_controll_addr(ax_addr)

        data, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                                         self.portHandler, dxl_id, ax_addr)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return data

    def ping(self, dxl_id):
        """ Ping Dynamixel to servo, use as argument servo ID. """

        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(
                                                     self.portHandler, dxl_id)

        return dxl_model_number

    def scan(self, dynamixel_id_bytes=None):
        """ Scan all avalaible sevros on network """

        print("Scanning avalaible Dynamixel's")
        available_ids = []

        if dynamixel_id_bytes is None:
            dynamixel_id_bytes = list(range(1,254))

        for dynamixel_id in dynamixel_id_bytes:
            if 0 <= dynamixel_id <= 255:
                if self.ping(dynamixel_id):
                    available_ids.append(dynamixel_id)

                else:
                    pass

        return available_ids

    def update_servo_list(self):
        """ This function updated servo list and saved into file """

        updated_list = self.scan()
        json_file.write('dxl_servo_list.json', updated_list)
        print("Now your servo list is now updated!")
        return updated_list

    def close(self):
        """ Close serial connection """

        self.portHandler.closePort()
