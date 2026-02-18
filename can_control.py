"""
Class for sending commands over the CAN network to control a AK80-8 Motor.
Derived from TMotorCANControl API https://github.com/neurobionics/TMotorCANControl/tree/master
"""
import can
import time
import os
from collections import namedtuple

MIT_Params = {
            'AK80-8': {
                'P_min' : -12.5,
                'P_max' : 12.5,
                'V_min' : -37.5,
                'V_max' : 37.5,
                'T_min' : -32.0,
                'T_max' : 32.0,
                'Kp_min': 0.0,
                'Kp_max': 500.0,
                'Kd_min': 0.0,
                'Kd_max': 5.0,
                'Kt_TMotor' : 0.091, # from TMotor website (actually 1/Kvll)
                'Current_Factor' : 0.59, # to correct the qaxis current 
                'Kt_actual': 0.115,# Need to use the right constant -- 0.115 by our calcs, 0.091 by theirs. At output leads to 1.31 by them and 1.42 by us.
                'GEAR_RATIO': 8.0, # hence the 9 in the name
                'Use_derived_torque_constants': False, # true if you have a better model
                'a_hat' : [0.0, 1.15605006e+00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02]
            }
        }

# motor state from the controller, uneditable named tuple
MIT_motor_state = namedtuple('motor_state', 'position velocity current temperature error')

class CanListener(can.Listener):
    def __init__(self, controller, motor_type):
        self.controller = controller
        self.motor_type = motor_type

    def on_message_received(self, msg):
        data = bytes(msg.data)
        self.controller.parse_MIT_message(data, self.motor_type)


class CanController:

    def __init__(self):
        os.system('sudo ifconfig can0 down')
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system('sudo ifconfig can0 up')
        self.bus = can.interface.Bus(channel = 'can0', interface = 'socketcan')
        self.debug = True

    def limit_value(self, value, min, max): # From TMotorCANControl
        """
        Limits value to be between min and max

        Args:
            value: The value to be limited.
            min: The lowest number allowed (inclusive) for value
            max: The highest number allowed (inclusive) for value
        """
        if value >= max:
            return max
        elif value <= min:
            return min
        else:
            return value
        
    def float_to_uint(self, x, x_min, x_max, num_bits): # From TMotorCANControl
        """
        Interpolates a floating point number to an unsigned integer of num_bits length.
        A number of x_max will be the largest integer of num_bits, and x_min would be 0.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max-x_min
        bitratio = float((1<<num_bits)/span)
        x = self.limit_value(x, x_min, x_max-(2/bitratio))
        # (x - x_min)*(2^num_bits)/span
        
        return self.limit_value(int((x- x_min)*( bitratio )),0,int((x_max-x_min)*bitratio) )
    
    def uint_to_float(self, x, x_min, x_max, num_bits): # From TMotorCANControl
        """
        Interpolates an unsigned integer of num_bits length to a floating point number between x_min and x_max.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max-x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x*span/((1<<num_bits)-1) + x_min)
    
    # sends a message to the motor (when the motor is in MIT mode)
    def send_MIT_message(self, motor_id, data): # From TMotorCANControl
        """
        Sends an MIT Mode message to the motor, with a header of motor_id and data array of data

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
        """
        DLC = len(data)
        assert (DLC <= 8), ('Data too long in message for motor ' + str(motor_id))
        
        if self.debug:
            print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info) )
        except can.CanError:
            if self.debug:
                print("    Message NOT sent")

    # send the power on code
    def power_on(self, motor_id): # From TMotorCANControl
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC])
        
    # send the power off code
    def power_off(self, motor_id):
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])

    # send the zeroing code. Like a scale, it takes about a second to zero the position
    def zero(self, motor_id): # From TMotorCANControl
        """
        Sends the zeroing code to motor_id. This code will shut off communication with the motor for about a second.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I): # From TMotorCANControl
        """
        Sends an MIT style control signal to the motor. This signal will be used to generate a 
        current for the field-oriented controller on the motor control chip, given by this expression:

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: The CAN ID of the motor to send the message to
            motor_type: A string noting the type of motor, ie 'AK80-9'
            position: The desired position in rad
            velocity: The desired velocity in rad/s
            Kp: The position gain
            Kd: The velocity gain
            I: The additional current
        """

        position_uint16 = self.float_to_uint(position, MIT_Params[motor_type]['P_min'], 
                                                    MIT_Params[motor_type]['P_max'], 16)
        velocity_uint12 = self.float_to_uint(velocity, MIT_Params[motor_type]['V_min'], 
                                                    MIT_Params[motor_type]['V_max'], 12)
        Kp_uint12 = self.float_to_uint(Kp, MIT_Params[motor_type]['Kp_min'], 
                                                    MIT_Params[motor_type]['Kp_max'], 12)
        Kd_uint12 = self.float_to_uint(Kd, MIT_Params[motor_type]['Kd_min'], 
                                                    MIT_Params[motor_type]['Kd_max'], 12)
        I_uint12 = self.float_to_uint(I, MIT_Params[motor_type]['T_min'], 
                                                    MIT_Params[motor_type]['T_max'], 12)

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12&0x00F)<<4) | (Kp_uint12) >> 8,
            (Kp_uint12&0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12&0x00F)<<4) | (I_uint12) >> 8,
            (I_uint12&0x0FF)
        ]
        # print(data)
        self.send_MIT_message(motor_id, data)

    # convert data recieved from motor in byte format back into floating point numbers in real units
    def parse_MIT_message(self, data, motor_type):
        """
        Takes a RAW MIT message and formats it into readable floating point numbers.

        Args:
            data: the bytes of data from a python-can message object to be parsed
            motor_type: A string noting the type of motor, ie 'AK80-9'

        Returns:
            An MIT_Motor_State namedtuple that contains floating point values for the 
            position, velocity, current, temperature, and error in rad, rad/s, amps, and *C.
            0 means no error. 
            
            Notably, the current is converted to amps from the reported 
            'torque' value, which is i*Kt. This allows control based on actual q-axis current,
            rather than estimated torque, which doesn't account for friction losses.
        """
        assert len(data) == 8 or len(data) == 6, 'Tried to parse a CAN message that was not Motor State in MIT Mode'
        temp = None
        error = None
        position_uint = data[1] <<8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4]>>4) <<4 ) >> 4
        current_uint = (data[4]&0x0F)<<8 | data[5]
        
        if len(data)  == 8:
            temp = int(data[6])
            error = int(data[7])

        position = self.uint_to_float(position_uint, MIT_Params[motor_type]['P_min'], 
                                            MIT_Params[motor_type]['P_max'], 16)
        velocity = self.uint_to_float(velocity_uint, MIT_Params[motor_type]['V_min'], 
                                            MIT_Params[motor_type]['V_max'], 12)
        current = self.uint_to_float(current_uint, MIT_Params[motor_type]['T_min'], 
                                            MIT_Params[motor_type]['T_max'], 12)

        if self.debug:
            print('  Position: ' + str(position))
            print('  Velocity: ' + str(velocity))
            print('  Current: ' + str(current))
            if (temp is not None) and (error is not None):
                print('  Temp: ' + str(temp))
                print('  Error: ' + str(error))

        # returns the Tmotor "current" which is really a torque estimate
        return MIT_motor_state(position, velocity, current, temp, error)

    def can_shutdown(self):
        self.bus.shutdown()
        os.system('sudo ifconfig can0 down')
        print(f"CAN successfully shutdown")