import serial
import serial.tools.list_ports
from serial import SerialException
import time

class SerialMessenger():

    def __init__(self):
        self.serial_number = "USB Serial"
        self.serial_timeout = 2
        self.get_serial_port(self.serial_number)
        self.connect_to_serial(self.serial_port)


    def get_serial_port(self, serial_number, logger):
        match_port = None
        all_ports = serial.tools.list_ports.comports()
        logger.info('finding SN: {} in {} ports'.format(serial_number, len(all_ports)))

        for port in all_ports: 
            logger.info(f'port is {port}')
            if 'USB' in port.description:
                logger.info('MATCH! port : {} SN: {}'.format(port.device, port.serial_number))
                match_port = port.device

        if match_port is None:
            logger.info('no matching port for SN: {}'.format(serial_number))

        self.serial_port = match_port
        return match_port

    def connect_to_serial(self, port, logger):
        if(port is not None):
            logger.info("connecting at port {}".format(self.serial_port))
            try:
                # open and clear serial port
                self.serial = serial.Serial(port, 9600, timeout = self.serial_timeout)
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                logger.info("serial connected!")                

            except SerialException as err:
                logger.info(f"str(err): {str(err)}")

# arduino = serial.Serial(port='ttyACM0', baudrate=9600, timeout=.1)

    def readOffset(self, logger):
        # if self.serial is None:
        self.serial_number = "/dev/ttyACM0 - USB Serial"
        self.serial_timeout = 2

        self.get_serial_port(self, self.serial_number, logger)
        self.connect_to_serial(self, self.serial_port, logger)

        value = self.serial.readline()
        logger.info(f"value: {value}")
        return value

    # while True:
    #     readOffset()