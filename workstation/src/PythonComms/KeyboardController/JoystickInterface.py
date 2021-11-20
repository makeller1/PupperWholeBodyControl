import UDPComms
from KeyboardController.Command import Command


class JoystickInterface:
    def __init__(self, udp_port=8830, udp_publisher_port=8840):
        self.previous_gait_toggle = 0
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        self.udp_publisher = UDPComms.Publisher(udp_publisher_port)

    def get_command(self, do_print=False):
        try:
            msg = self.udp_handle.get()
            command = Command()

            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = msg["R1"]
            command.trot_event = gait_toggle == 1 and self.previous_gait_toggle == 0

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = msg["x"]
            command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

            activate_toggle = msg["L1"]
            command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0
            )

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            return command

        except UDPComms.timeout:
            if do_print:
                print("UDP Timed out")
            return Command()

    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        self.udp_publisher.send(joystick_msg)
