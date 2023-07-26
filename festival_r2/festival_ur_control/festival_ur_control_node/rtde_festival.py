import numpy as np
import math
import rtde_control, rtde_receive


class Festival():
    def __init__(self, UR_IP):
        self.control_interface = rtde_control.RTDEControlInterface(UR_IP)
        self.receive_interface = rtde_receive.RTDEReceiveInterface(UR_IP)

        
    def __del__(self):
        self.control_interface.disconnect()
        self.receive_interface.disconnect()

    