import usb.util
import time
import math
import array
import time
from odrive.dfuse.DfuState import DfuState

DFU_REQUEST_SEND = 0x21
DFU_REQUEST_RECEIVE = 0xa1

DFU_DETACH    = 0x00
DFU_DNLOAD    = 0x01
DFU_UPLOAD    = 0x02
DFU_GETSTATUS = 0x03
DFU_CLRSTATUS = 0x04
DFU_GETSTATE  = 0x05
DFU_ABORT     = 0x06

SIZE_MULTIPLIERS = {' ': 1, 'K': 1024, 'M' : 1024*1024}
MAX_TRANSFER_SIZE = 2048

# Order is LSB first
def address_to_4bytes(a):
    return [ a % 256, (a >> 8)%256, (a >> 16)%256, (a >> 24)%256 ]

class DfuDevice:
    def __init__(self, device, timeout = None):
        self.dev = device
        self.timeout = timeout
        self.cfg = self.dev[0]
        self.intf = None
        #self.dev.reset()
        self.cfg.set()
        self.sectors = list(self.get_device_sectors())

    def alternates(self):
        return [(usb.util.get_string(self.dev, intf.iInterface), intf) for intf in self.cfg]

    def set_alternate(self, intf):
        if isinstance(intf, tuple):
            self.intf = intf[1]
        else:
            self.intf = intf
        
        self.intf.set_altsetting()

    def control_msg(self, requestType, request, value, buffer, timeout=None):
        return self.dev.ctrl_transfer(requestType, request, value, self.intf.bInterfaceNumber, buffer, timeout=timeout)

    def detach(self, timeout):
        return self.control_msg(DFU_REQUEST_SEND, DFU_DETACH, timeout, None)
    
    def dnload(self, blockNum, data):
        cnt = self.control_msg(DFU_REQUEST_SEND, DFU_DNLOAD, blockNum, list(data))
        return cnt
    
    def upload(self, blockNum, size):
        return self.control_msg(DFU_REQUEST_RECEIVE, DFU_UPLOAD, blockNum, size)

    def get_status(self, timeout=None):
        status = self.control_msg(DFU_REQUEST_RECEIVE, DFU_GETSTATUS, 0, 6, timeout=timeout)
        return (status[0], status[4], status[1] + (status[2] << 8) + (status[3] << 16), status[5])
    
    def clear_status(self):
        self.control_msg(DFU_REQUEST_SEND, DFU_CLRSTATUS, 0, None)

    def get_state(self):
        msg = self.control_msg(DFU_REQUEST_RECEIVE, DFU_GETSTATE, 0, 1)

        # Second chance after giving the device some time to breathe.
        if len(msg) == 0:
            time.sleep(0.5)
            msg = self.control_msg(DFU_REQUEST_RECEIVE, DFU_GETSTATE, 0, 1)

        if len(msg) == 0:
            raise Exception("Could not get device state. Firmware upgrade will abort. "
                            "Please try again. If odrivetool can't find the device "
                            "anymore after this, follow the instructions in "
                            "https://docs.odriverobotics.com/odrivetool#device-firmware-update "
                            "(\"How to force DFU mode\").")
        return msg[0]

    def abort(self):
        self.control_msg(DFU_REQUEST_RECEIVE, DFU_ABORT, 0, 0)

    def set_address(self, ap):
        return self.dnload(0x0, [0x21] + address_to_4bytes(ap))

    def write(self, block, data):
        return self.dnload(block + 2, data)

    def read(self, block, size):
        return self.upload(block + 2, size)
    
    def erase(self, pa):
        return self.dnload(0x0, [0x41] + address_to_4bytes(pa))

    def leave(self):
        return self.dnload(0x0, []) # Just send an empty data.

    def wait_while_state(self, state, timeout=None):
        if not isinstance(state, (list, tuple)):
            states = (state,)
        else:
            states = state

        try:
            status = self.get_status()
        except:
            time.sleep(0.100)
            status = self.get_status()
        
        while (status[1] in states):
            claimed_timeout = status[2]
            actual_timeout = int(max(timeout or 0, claimed_timeout))
            #print("timeout = %f, claimed = %f" % (timeout, status[2]))
            #time.sleep(timeout)
            status = self.get_status(timeout=actual_timeout)
        
        return status

    ## High level functions ##
    # by ODrive Robotics

    def get_device_sectors(self):
        """
        Returns a list of all sectors on the device.
        Each sector is represented as a dictionary with the following keys:
        - name: name of the associated memory region (e.g. "Internal Flash")
        - alt: USB alternate setting associated with this memory region
        - addr: Start address of the sector (e.g. 0x08004000 for the second flash sectors)
        - baseaddr: Start address of the memory region associated with the sector
                    (e.g. 0x08000000 for all flash sectors)
        - len: Number of bytes in the sector
        """
        for name, alt in self.alternates():
            # example for name:
            # '@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg'
            label, baseaddr, layout = name.split('/')
            baseaddr = int(baseaddr, 0) # convert hex to decimal
            addr = baseaddr

            for sector in layout.split(','):
                repeat, size = map(int, sector[:-2].split('*'))
                size *= SIZE_MULTIPLIERS[sector[-2].upper()]
                mode = sector[-1]

                while repeat > 0:
                    # TODO: verify if the section is writable
                    yield {
                        'name': label.strip().strip('@'),
                        'alt': alt,
                        'baseaddr': baseaddr,
                        'addr': addr,
                        'len': size,
                        'mode': mode
                    }

                    addr += size
                    repeat -= 1

    def set_alternate_safe(self, alt):
        self.set_alternate(alt)
        if self.get_state() == DfuState.DFU_ERROR:
            self.clear_status()
            self.wait_while_state(DfuState.DFU_ERROR)

    #def clear_error(self)
    def set_address_safe(self, addr):
        self.set_address(addr)
        status = self.wait_while_state(DfuState.DFU_DOWNLOAD_BUSY)
        if status[1] != DfuState.DFU_DOWNLOAD_IDLE:
            raise RuntimeError("An error occured. Device Status: {!r}".format(status))
        # take device out of DFU_DOWNLOAD_SYNC and into DFU_IDLE
        self.abort()
        status = self.wait_while_state(DfuState.DFU_DOWNLOAD_SYNC)
        if status[1] != DfuState.DFU_IDLE:
            raise RuntimeError("An error occured. Device Status: {!r}".format(status))
        

    def erase_sector(self, sector):
        self.set_alternate_safe(sector['alt'])
        self.erase(sector['addr'])
        status = self.wait_while_state(DfuState.DFU_DOWNLOAD_BUSY, timeout=sector['len']/32)
        if status[1] != DfuState.DFU_DOWNLOAD_IDLE:
            raise RuntimeError("An error occured. Device Status: {!r}".format(status))

    def write_sector(self, sector, data):
        self.set_alternate_safe(sector['alt'])
        self.set_address_safe(sector['addr'])

        transfer_size = math.gcd(sector['len'], MAX_TRANSFER_SIZE)
        
        blocks = [data[i:i + transfer_size] for i in range(0, len(data), transfer_size)]
        for blocknum, block in enumerate(blocks):
            #print('write to {:08X} ({} bytes)'.format(
            #        sector['addr'] + blocknum * transfer_size, len(block)))
            self.write(blocknum, block)
            status = self.wait_while_state(DfuState.DFU_DOWNLOAD_BUSY)
            if status[1] != DfuState.DFU_DOWNLOAD_IDLE:
                raise RuntimeError("An error occured. Device Status: {!r}".format(status))

    def read_sector(self, sector):
        """
        Reads data from the specified sector
        Returns: a byte array containing the data
        """
        self.set_alternate_safe(sector['alt'])
        self.set_address_safe(sector['addr'])

        transfer_size = math.gcd(sector['len'], MAX_TRANSFER_SIZE)
        #blocknum_offset = int((sector['addr'] - sector['baseaddr']) / transfer_size)

        
        data = array.array(u'B')
        for blocknum in range(int(sector['len'] / transfer_size)):
            #print('read at {:08X}'.format(sector['addr'] + blocknum * transfer_size))
            deviceBlock = self.read(blocknum, transfer_size)
            data.extend(deviceBlock)
        self.abort() # take device into DFU_IDLE
        return data

    def jump_to_application(self, address):
        self.set_address_safe(address)
        #self.set_address(address)
        #status = self.wait_while_state(DfuState.DFU_DOWNLOAD_BUSY)
        #if status[1] != DfuState.DFU_DOWNLOAD_IDLE:
        #    raise RuntimeError("An error occured. Device Status: {}".format(status[1]))

        self.leave()
        status = self.wait_while_state(DfuState.DFU_MANIFEST_SYNC)
        if status[1] != DfuState.DFU_MANIFEST:
            raise RuntimeError("An error occured. Device Status: {}".format(status[1]))
