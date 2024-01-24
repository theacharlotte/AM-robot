import serial
import math
import struct
import time
from time import sleep

#from am_robot.AbstractTool import AbstractTool


class ExtruderTool():
    '''
    Converts feedrate from mm/min to mm/s and passes it to extrusion controller

    Attributes:
    -----------
    feedrate: int
        rate of filament enxtrusion in mm/min

    Methods:
    -----------
    feedrate: float
        feedrate in mm/s
    '''
    def __init__(self,port,tooltype):
        '''
        Initialize extruder tool
        '''
        #super().__init__(port)

        self.tooltype = tooltype
        self.port = port

        print(self.port)
        self.ser = serial.Serial(self.port)

        self.motor_steps_per_revolution = 400.0
        self.micro_stepping = 16.0
        self.gear_ratio = 3.0  # From datasheet
        self.hobb_diameter_mm = 7.68  # 7.3 # From datasheet/manufacturer (effective and should be calibrated)

        self.steps_per_mm_filament = self.calculate_steps_per_mm()

    def disconnect(self):
        '''
        Disconnect the serial connection
        '''
        self.ser.close()

    def set_feedrate(self,feedrate):
        '''
        Set feedrate for the extruder

        Input:
        -----
        feedrate: float
            feedrate in mm/min
        '''
        feedrate = self.convert_per_minute_to_per_second(feedrate)
        motor_frequency = self.feedrate_to_motor_frequency(feedrate)
        packed = struct.pack('f',motor_frequency)
        self.ser.write(b'X'+packed)  # + 4 bytes of number

    def set_nozzletemp(self,temperature):
        '''
        Set hotend temperature in Celsius

        Input:
        -----
        temperature: float
            temperature in degree celsius
        '''
        packed = struct.pack('f',temperature)
        self.ser.write(b'H'+packed)  # + 4 bytes of number

   
    def read_temperature(self):
        '''
        Read and return temperature of hotend in Celsius

        Returns:
        -----
        temperature: float
            Temperature of hotend in Celsius

        '''
        while True:
            self.ser.write(b'T')
            b = self.ser.read(10)

            letter1 = b[0]
            letter2 = b[5]

            if letter1 == 84:
                [temperature] = struct.unpack('f',b[1:5])
                # print(f"Temperature is {temperature} degree celsius")
                return temperature
            elif letter2 == 84:
                [temperature] = struct.unpack('f',b[6:10])
                # print(f"Temperature is {temperature} degree celsius")
                return temperature
            else:
                print("Value other than T read. Ignoring...")

    def read_extrusion_speed(self):  ## Unused now, but might be useful in feedback loop
        '''
        Read and return extrusion rate in mm/s

        Returns:
        -----
        feedrate: float
            Feedrate of extrusion process in mm/s

        '''
        read_extrusion = True
        while read_extrusion:
            b = self.ser.read(5)

            letter = b[0]

            if letter == 69:
                [motor_frequency] = struct.unpack('f',b[1:5])
                feedrate = self.motor_frequency_to_feedrate(motor_frequency)
                # print(f"Extrusion rate is {feedrate} mm/s")
                read_extrusion = False
                return feedrate
            else:
                print("Value other than E read. Ignoring...")
                time.sleep(0.1)

    def enable_periodic_updates(self):
        '''
        Enable periodic updates from arduino. Updates include temperature readout and extrusion rate
        '''
        self.ser.write(b'Y')

    def disable_periodic_updates(self):
        '''
        Disable periodic updates from arduino.
        '''
        self.ser.write(b'N')

    def feedrate_to_motor_frequency(self,feedrate):
        '''
        Converts feedrate mm/s to motor frequency Hz

        Input:
        -----
        feedrate: float
            Feedrate of extrusion process in mm/s

        Returns:
        -----
        motor_frequency: float
            Motor frequency in Hertz based on steps per mm filament

        '''
        motor_frequency = self.steps_per_mm_filament * feedrate
        return motor_frequency

    def motor_frequency_to_feedrate(self,motor_frequency):
        '''
        Converts motor frequency Hz to feedrate mm/s

        Input:
        -----
        motor_frequency: float
            motor frequency in Hertz

        Returns:
        -----
        feedrate: float
            Feedrate in mm/s based on steps per mm filament

        '''
        feedrate = motor_frequency / self.steps_per_mm_filament
        return feedrate

    def calculate_steps_per_mm(self):
        '''
        Calculates the stepper motor steps per mm filament

        Returns:
        -----
        steps_per_mm: float
            Number of stepper motor 'steps' per mm input filament

        '''
        return (self.motor_steps_per_revolution * self.micro_stepping * self.gear_ratio) / (self.hobb_diameter_mm * math.pi)

    def convert_per_minute_to_per_second(self,value_per_minute):
        '''
        Devides by 60
        '''
        return value_per_minute/60.0


    def calculate_delta_t(self,first_value,second_value,feedrate):
        '''
        Returns the time taken to process the difference between two distances
        '''
        return (second_value - first_value)/self.convert_per_minute_to_per_second(feedrate)

    def calculate_max_rel_velocity(self,feedrate,robot_vel_constraint):
        '''
        Returns the relative velocity ratio between robot maximum and desired process speed
        '''
        return self.convert_per_minute_to_per_second(feedrate)/robot_vel_constraint
    
