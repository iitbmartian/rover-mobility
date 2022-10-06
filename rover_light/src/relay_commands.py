import usb.core
import usb.util
import platform


class FT245R:
    def __init__(self):
        self.VID = 0x0403  # USB Vendor ID of FT245R and FT232
        self.PID = 0x6001  # USB Product ID of FT245R and FT232
        self.PROD_STR = u'FT245R USB FIFO'  # differentiate from FT232 USB UART
        self.is_connected = False
        self.dev = None
        self.RELAY_MIN = 1
        self.RELAY_MAX = 8
        self.relay_state = 0  # 8 bits representing 8 relays

    def list_dev(self):
        """
        Returns the list of FT245R devices.
        @return: device list
        """
        ret = []
        for dev in usb.core.find(find_all=True,
                                 idVendor=self.VID,
                                 idProduct=self.PID):
            # Ignore FT232 UART. Has same VID and PID as FT245R.
            # if dev.product == self.PROD_STR:
            ret.append(dev)
        return ret

    def disconnect(self):
        """
        Disables output to the device. Attaches the kernel driver if available.
                """
        self.is_connected = False
        if platform.system() != 'Windows':
            # If Linux OS already has control, there's nothing to do
            if self.dev.is_kernel_driver_active(0):
                return

        # Disable bitbang mode
        ret = self.dev.ctrl_transfer(0x40, 0x0b, 0x0000, 0x01, None, 500)
        if ret < 0:
            raise RuntimeError("relayctl: failure to disable bitbang mode")

        if platform.system() != 'Windows':
            try:
                self.dev.attach_kernel_driver(0)
            except:
                print("relayctl: could not attach kernel driver")

    def connect(self, dev):
        """
        Enables output to the device. Detaches the kernel driver if attached.
        @param dev: device
        """
        # Save the device handler so user does not have to keep passing it
        self.dev = dev

        # Detach kernel driver
        if platform.system() != 'Windows':
            if dev.is_kernel_driver_active(0):
                try:
                    dev.detach_kernel_driver(0)
                except:
                    raise RuntimeError("relayctl: failure to detach kernel driver")

        if not self.is_connected:
            # Set the active configuration. Windows errors if this is not done.
            # But Linux errors if this is done more than once (without closing)
            dev.set_configuration()

            # Enable bitbang mode
            ret = dev.ctrl_transfer(0x40, 0x0b, 0x01ff, 0x01, None, 500)
            if ret < 0:
                raise RuntimeError("relayctl: failure to enable bitbang mode")
            self.is_connected = True
            self.relay_state = self._getstatus_byte()

    def _getstatus_byte(self):
        """
        Gets a byte which represents the status of all 8 relays.
        @return: status
        """

        # Check for errors
        if not self.is_connected:
            raise IOError('Must connect to device first')

        # Read status
        buf = bytes([0x00]);
        buf = self.dev.ctrl_transfer(0xC0, 0x0c, 0x0000, 0x01, buf, 500)
        if len(buf) == 0:
            raise RuntimeError("relayctl: failure to read status")

        return buf[0]

    def getstatus(self, relay_num):
        """
        Returns 1 if relay relay_num is on, 0 if off.
        @return: status
        """

        # Check for errors
        if relay_num < self.RELAY_MIN or relay_num > self.RELAY_MAX:
            raise ValueError('Relay number {} is invalid'.format(relay_num))
        if not self.is_connected:
            raise IOError('Must connect to device first')

        # Read status
        if self.relay_state & (1 << (relay_num - 1)):
            return 1
        return 0

    def setstate(self):
        """
        Sets all relays to the state in FT245R.relay_state.
        """

        # Check for errors
        if not self.is_connected:
            raise IOError('Must connect to device first')

        # Clear the bit representing relay_num and mask it into the existing
        # relay_state
        buf = [0]
        buf[0] = self.relay_state

        # Write status
        ret = self.dev.write(0x02, buf, 500)
        if ret < 0:
            raise RuntimeError("relayctl: failure to write status")
        return

    def switchoff(self, relay_num):
        """
        Switches relay relay_num off.
        @param relay_num: which relay
        """
        relay_num = 2 * relay_num

        # Check for errors
        if relay_num < self.RELAY_MIN or relay_num > self.RELAY_MAX:
            raise ValueError('Relay number {} is invalid'.format(relay_num))
        if not self.is_connected:
            raise IOError('Must connect to device first')

        # Clear the bit representing relay_num and mask it into the existing
        # relay_state
        buf = [0]
        buf[0] = self.relay_state & ~(1 << (relay_num - 1))

        # Write status
        ret = self.dev.write(0x02, buf, 500)
        if ret < 0:
            raise RuntimeError("relayctl: failure to write status")

        # Save status
        self.relay_state = buf[0]
        return

    def switchon(self, relay_num):
        """
        Switches relay relay_num on.
        @param relay_num: which relay
        """
        relay_num = 2 * relay_num
        # Check for errors
        if relay_num < self.RELAY_MIN or relay_num > self.RELAY_MAX:
            raise ValueError('Relay number {} is invalid'.format(relay_num))
        if not self.is_connected:
            raise IOError('Must connect to device first')

        # Set the bit representing relay_num and mask it into the existing
        # relay_state
        buf = [0]
        buf[0] = self.relay_state | (1 << (relay_num - 1))

        # Write status
        ret = self.dev.write(0x02, buf, 500)
        if ret < 0:
            raise RuntimeError("relayctl: failure to write status")

        # Save status
        self.relay_state = buf[0]
        return
