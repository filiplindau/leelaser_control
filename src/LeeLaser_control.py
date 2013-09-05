'''
Created on Aug 28, 2012

@author: KMLabs
'''
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import logging
import time

logging.basicConfig(level=logging.WARNING)

class LeeLaserError(Exception): pass

class LDP200HardwareParameters:
    '''
    Parameters stored in this class is in system units. They are for internal 
    representation only. To get useful units use getXX function in parent class.
    '''
    def __init__(self, readLen=55):
        self.current = None    # Actual diode current
        self.percent = None    # Sets the diode current from 0.0 to 100.0%
        self.frequecy = None    # Q-switch modulation frequency
        self.width = None      # Q-switch modulation pulse width
        self.SHGTemperature = None # Monitors the SHG temperature
        self.pumpTemperature = None # Monitors the water temperature at the pump
        self.headTemperature = None # Monitors the water temperature return from the laser cavity
        self.MACAddress = None   # The last two bytes of the controller MAC address. The first four bytes are c2-de-d0-00
        self.powerMeter = None     # Monitors the power from the laser if available
        self.laserState = None       # State of the laser
        self.shutterState = None     # State of the laser shutter
        self.faultStatus = None      # Fault status
        self.controlMode = None      # Control mode (remote)
        self.HSSState = None
        self.IPAddressHigh = None
        self.IPAddressLow = None
        self.subnetMaskHigh = None
        self.subnetMaskLow = None
        self.gatewayHigh = None
        self.gatewayLow = None
        self.SHGSetpoint = None    # Temperature of the SHG crystal 21.0 - 38.0 degC
        self.pumpSetpoint = None   # Temperature of the cooling water 20.0-25.0 degC
        self.systemSoftwareVersion = None
        self.bootloaderSoftwareVersion = None
        self.modulationSource = None  # Q-switch modulation/fps source
        self.diodeBoost = None    # Boost level for the diode current. Should be used for aging only
        self.externalOffset = None    # Compensates for any offset in the external current command
        self.ethernetOptions = None   # Ethernet options (bitmapped)
        self.rs232Options = None      # Serial options (bitmapped)
        self.systemSoftwareRevision = None    
        self.bootloaderSoftwareRevision = None
        self.remoteFreq1t7 = None
        self.remoteWidth1t7 = None
        self.meterOffset = None
        self.meterGain = None
        self.externalCommand = None
        self.gateState = None
        self.externalShutterSolenoidTemperature = None
        self.externalShutterHousingTemperature = None
        self.laserFaultsRegister1 = None
        self.laserFaultsRegister2 = None
        self.laserFaultsRegister3 = None
        self.laserFaultsRegister4 = None
        
        self.readLen = readLen        # Number of fields in the read vector
        
        self.registers = None
        
    def setAllFields(self, reg):
        '''
        Sets all fields in the data structure from a register vector.
        The length of the vector must be self.readLen.
        '''
        if reg.__len__() != self.readLen:
            logging.exception('Error in setAllFields: register vector wrong length.')
            return
        self.current = reg[0x00]
        self.percent = reg[0x01]
        self.frequecy = reg[0x02]
        self.width = reg[0x03]
        self.SHGTemperature = reg[0x04]
        self.pumpTemperature = reg[0x05]
        self.headTemperature = reg[0x06]
        self.MACAddress = reg[0x07]
        self.powerMeter = reg[0x08]
        self.laserState = reg[0x09]
        self.shutterState = reg[0x0a]
        self.faultStatus = reg[0x0b]
        self.controlMode = reg[0x0c]
        self.HSSState = reg[0x0d]
        self.IPAddressHigh = reg[0x0e]
        self.IPAddressLow = reg[0x0f]
        self.subnetMaskHigh = reg[0x10]
        self.subnetMaskLow = reg[0x11]
        self.bootloaderSoftwareVersion = reg[0x12]
        self.SHGSetpoint = reg[0x16]
        self.pumpSetpoint = reg[0x17]
        self.gatewayHigh = reg[0x18]
        self.gatewayLow = reg[0x19]
        self.systemSoftwareVersion = reg[0x1a]
        self.modulationSource = reg[0x1b]
        self.diodeBoost = reg[0x1c]
        self.externalOffset = reg[0x1d]
        self.ethernetOptions = reg[0x1e]
        self.rs232Options = reg[0x1f]
        self.systemSoftwareRevision = reg[0x20]    
        self.remoteFreq1t7 = reg[0x21:0x27]
        self.remoteWidth1t7 = reg[0x28:0x2e]
        self.meterOffset = reg[0x2f]
        self.meterGain = reg[0x30]
        self.externalCommand = reg[0x31]
        self.gateState = reg[0x32]
        self.bootloaderSoftwareRevision = reg[0x33]
#        self.externalShutterSolenoidTemperature=reg[0x3a]
#        self.externalShutterHousingTemperature=reg[0x3b]

        if reg.__len__() > 0x3f:
            self.laserFaultsRegister1 = reg[0x3c]
            self.laserFaultsRegister2 = reg[0x3d]
            self.laserFaultsRegister3 = reg[0x3e]
            self.laserFaultsRegister4 = reg[0x3f]

        self.registers = reg

class LeeLaserControl:
    def __init__(self, ip='128.100.101.190', port=502, slaveId=1):
        '''
        Create an instance of the LeeLaserControl class.
        The targeted device is LDP200. The register map might not
        be valid for other devices.
        The default ip is for use with the KMLabs provided router.
        The modbus client is initialized immediately.
        '''
        self.client = None      
        self.connected = False
        self.readLen = 64     # Number of registers to read over modbus
        self.ldp200 = LDP200HardwareParameters(self.readLen) 
        self.initClient(ip, portNbr=port, slvId=slaveId)
        
        self.minReadTime = 0.3      # Minimum time between reads to prevent overload of hardware in seconds
        self.lastReadTime = time.time()
        
        
    def initClient(self, ip, portNbr=502, slvId=1):
        ''' 
        Initialize the connection with the modbus device.
        Default port numbers and slave id works for the LeeLasers.
        '''
        self.disconnectClient()
        try:
            self.client = ModbusClient(ip, port=portNbr)
            self.slaveId = slvId
            self.connectClient()
            
        except Exception, e:
            self.client = None
            ex = LeeLaserError(''.join(('Could not connect', str(e))))
            logging.exception(''.join(('Error initializing ModbusClient', str(e))))
            print 'Error initializing ModbusClient ', str(e)
            raise(ex)
        
    def connectClient(self):
        if self.client != None:
            self.connected = self.client.connect()
        
    def closeClient(self):
        if self.client != None:
            self.client.close()
            self.client = None
            self.connected = False
        
    def disconnectClient(self):
        '''
        Disconnect from the modbus device to free up the socket.
        '''
        if self.client != None:
            self.client.close()
            self.connected = False
            
            
    def readHardware(self):
        '''
        Read the hardware registers of the LeeLaser LDP200.
        '''
        if self.client != None:
            try:
                # First check when the last read was to prevent overloading the hardware
                dt = time.time() - self.lastReadTime
                if dt < self.minReadTime:
                    time.sleep(dt)
                # 59 registers in the register map.
                rr = self.client.read_input_registers(0, self.readLen, self.slaveId)
                self.lastReadTime = time.time()
                self.registers = rr
                if rr.function_code == 4:
                    # The request returned ok.
                    self.ldp200.setAllFields(rr.registers)
                else:
                    ex = LeeLaserError(str(rr.function_code))
                    raise(ex)
                    
            except Exception as e:
                ex = LeeLaserError(''.join(('Error reading hardware registers.', str(e))))
                logging.exception(''.join(('Error reading hardware registers.', str(e))))
                raise(ex)
        else:
            ex = LeeLaserError('Not connected to LeeLaser')
            logging.exception('Not connected to LeeLaser')
            raise(ex)
                
    def getCurrent(self):
        current = self.ldp200.current * 0.1
        return (current, 'A')
    
    def getPercentCurrent(self):
        data = self.ldp200.percent * 0.1
        return (data, '%')

    def getFrequency(self):
        data = self.ldp200.frequecy * 1.0
        return (data, 'Hz')
    
    def getWidth(self):
        data = self.ldp200.width * 0.01
        return (data, 'us')
    
    def getPumpTemperature(self):
        data = self.ldp200.pumpTemperature * 0.1
        return (data, 'degC')

    def getSHGTemperature(self):
        data = self.ldp200.SHGTemperature * 0.1
        return (data, 'degC')

    def getHeadTemperature(self):
        data = self.ldp200.headTemperature * 0.1
        return (data, 'degC')

    def getLaserState(self):
        data = self.ldp200.laserState
        if data == 0:
            s = 'Fault'
        elif data == 1:
            s = 'Off'
        elif data == 2:
            s = 'Cool Off Delay'
        elif data == 3:
            s = 'Cooling Start Delay'
        elif data == 4:
            s = 'Cooling'
        elif data == 5:
            s = 'Starting Power Supply'
        elif data == 6:
            s = 'On'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getShutterState(self):
        data = self.ldp200.shutterState
        if data == 0:
            s = 'Software Disabled'
        elif data == 1:
            s = 'Shutter Closed'
        elif data == 2:
            s = 'Open Delay'
        elif data == 3:
            s = 'Shutter Open'
        elif data == 4:
            s = 'Hardware Disabled'
        elif data == 5:
            s = 'Starting Power Supply'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getFaultStatus(self):
        data = self.ldp200.faultStatus
        if data == 0:
            s = 'Power up'
        elif data == 1:
            s = 'No faults'
        elif data == 2:
            s = 'Current limit'
        elif data == 3:
            s = 'Water overtemp'
        elif data == 4:
            s = 'Water undertemp'
        elif data == 5:
            s = 'Water flow low'
        elif data == 6:
            s = 'Laser head disconnected'
        elif data == 7:
            s = 'Overcurrent fault'
        elif data == 8:
            s = 'RESERVED'
        elif data == 9:
            s = 'Water level low'
        elif data == 10:
            s = 'Laser box open'
        elif data == 11:            
            s = 'Fiber short'
        elif data == 12:
            s = 'Fiber break'
        elif data == 13:
            s = 'Safety shutter fault'
        elif data == 14:
            s = 'Shutter hardware fault'
        elif data == 15:
            s = 'High speed shutter overtemp'
        elif data == 17:
            s = 'SHG overtemp'
        elif data == 18:
            s = 'RF1 high power'
        elif data == 20:
            s = 'RF1 low power'
        elif data == 22:
            s = 'RF1 overtemp'
        elif data == 24:
            s = 'RF1 VSWR'
        elif data == 26:
            s = 'Timer fault'
        elif data == 27:
            s = 'Shutter interlock'
        elif data == 29:
            s = 'SHG undertemp'
        elif data == 30:
            s = 'Shutter interlock 1'
        elif data == 31:
            s = 'Shutter solenoid overtemp'
        elif data == 32:
            s = 'Shutter housing overtemp'
        elif data == 50:
            s = 'Emergency stop'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getLaserFaults(self):
        faultList = []
        reg1 = self.ldp200.laserFaultsRegister1
        if reg1 == None:
            ex = LeeLaserError('Laser fault register 1 not initialized')
            raise(ex)
        else:
            if (reg1 & 0x0001) > 0:
                faultList.append('Reserved 0x0001')
            if (reg1 & 0x0002) > 0:
                faultList.append('Reserved 0x0002')
            if (reg1 & 0x0004) > 0:
                faultList.append('Current Limit')
            if (reg1 & 0x0008) > 0:
                faultList.append('Water Overtemp')
            if (reg1 & 0x0010) > 0:
                faultList.append('Water Undertemp')
            if (reg1 & 0x0020) > 0:
                faultList.append('Water Flow')
            if (reg1 & 0x0040) > 0:
                faultList.append('Head Disconnected')
            if (reg1 & 0x0080) > 0:
                faultList.append('Overcurrent')
            if (reg1 & 0x0100) > 0:
                faultList.append('Reserved 0x0100')
            if (reg1 & 0x0200) > 0:
                faultList.append('Water Level')
            if (reg1 & 0x0400) > 0:
                faultList.append('Laserbox Open')
            if (reg1 & 0x0800) > 0:
                faultList.append('Fiber Short')
            if (reg1 & 0x1000) > 0:
                faultList.append('Fiber Break')
            if (reg1 & 0x2000) > 0:
                faultList.append('Shutter Fault')
            if (reg1 & 0x4000) > 0:
                faultList.append('Shutter Hardware')
            if (reg1 & 0x8000) > 0:
                faultList.append('HSS Overtemp')
        reg2 = self.ldp200.laserFaultsRegister2
        if reg2 == None:
            ex = LeeLaserError('Laser fault register 2 not initialized')
            raise(ex)
        else:
            if (reg2 & 0x0001) > 0:
                faultList.append('THG Overtemp')
            if (reg2 & 0x0002) > 0:
                faultList.append('SHG Overtemp')
            if (reg2 & 0x0004) > 0:
                faultList.append('RF1 High Power')
            if (reg2 & 0x0008) > 0:
                faultList.append('RF2 High Power')
            if (reg2 & 0x0010) > 0:
                faultList.append('RF1 Low Power')
            if (reg2 & 0x0020) > 0:
                faultList.append('RF2 Low Power')
            if (reg2 & 0x0040) > 0:
                faultList.append('RF1 Overtemp')
            if (reg2 & 0x0080) > 0:
                faultList.append('RF2 Overtemp')
            if (reg2 & 0x0100) > 0:
                faultList.append('RF1 VSWR')
            if (reg2 & 0x0200) > 0:
                faultList.append('RF2 VSWR')
            if (reg2 & 0x0400) > 0:
                faultList.append('Timer Board')
            if (reg2 & 0x0800) > 0:
                faultList.append('Shutter Interlock 2')
            if (reg2 & 0x1000) > 0:
                faultList.append('THG Undertemp')
            if (reg2 & 0x2000) > 0:
                faultList.append('SHG Undertemp')
            if (reg2 & 0x4000) > 0:
                faultList.append('Shutter Solenoid Overtemp')
            if (reg2 & 0x8000) > 0:
                faultList.append('Shutter Housing Overtemp')
        reg3 = self.ldp200.laserFaultsRegister3
        if reg3 == None:
            ex = LeeLaserError('Laser fault register 3 not initialized')
            raise(ex)
        else:
            if (reg3 & 0x0001) > 0:
                faultList.append('Shutter Interlock 1')
            if (reg3 & 0x0002) > 0:
                faultList.append('Water Pressure')
            if (reg3 & 0x0004) > 0:
                faultList.append('Brown-Out Reset')
            if (reg3 & 0x0008) > 0:
                faultList.append('Watchdog Reset')
            if (reg3 & 0x0010) > 0:
                faultList.append('CPU Reset')
        reg4 = self.ldp200.laserFaultsRegister4
        if reg4 == None:
            ex = LeeLaserError('Laser fault register 4 not initialized')
            raise(ex)
        else:
            if (reg4 & 0x0010) > 0:
                faultList.append('Emergency Stop')
        regList = [reg1, reg2, reg3, reg4]
        return (regList, faultList)
    
    def getConnectionState(self):
        if self.client == None:
            data = 1
            s = 'Disconnected'
        else:
            if self.client.connect() == True:
                data = 0
                s = 'Connected'
            else:
                data = 1
                s = 'Disconnected'
            
        return (data, s)
    
    def getControlMode(self):
        data = self.ldp200.controlMode
        if data == 1:
            s = 'Manual'
        elif data == 2:
            s = 'Hardware remote'
        elif data == 3:
            s = 'RS-232'
        elif data == 4:
            s = 'Ethernet'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getSHGSetpoint(self):
        data = self.ldp200.SHGSetpoint * 0.1
        return (data, 'degC')

    def getPumpSetpoint(self):
        data = self.ldp200.PumpSetpoint * 0.1
        return (data, 'degC')
    
    def getModulationSource(self):
        data = self.ldp200.modulationSource
        if data == 1:
            s = 'Internal'
        elif data == 2:
            s = 'External'
        elif data == 3:
            s = 'Off'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getDiodeBoost(self):
        data = self.ldp200.diodeBoost
        return (data, '%')
    
    def getEthernetOptions(self):
        '''
        Returns the options for remote control commands when in ethernet mode
        '''
        data = bin(self.ldp200.ethernetOptions % 16)    # create string with binary representation of 4 first bits
        s = ''
        if data[5] == '1':    # bit 0
            s = ''.join((s, 'Laser On/Off'))
        elif data[4] == '1':  # bit 1
            if s != '':
                s = ''.join((s, ', '))
            s = ''.join((s, 'Shutter Open/Close'))
        elif data[2] == '1':  # bit 3
            if s != '':
                s = ''.join((s, ', '))
            s = ''.join((s, 'Gating On/Off'))
        else:
            s = 'None'
        return (self.ldp200.ethernetOptions, s)
        
    def getExternalCommand(self):
        '''
        Returns external command.
        '''
        data = self.ldp200.externalCommand * 0.01
        return (data, '%')
    
    def getGateState(self):
        '''
        Returns the gate state.
        '''
        data = self.ldp200.gateState
        if data == 1:
            s = 'Gate Closed'
        elif data == 3:
            s = 'Gate Open'
        else:
            s = 'Unknown state'
        return (data, s)
    
    def getSystemSoftwareVersion(self):
        ver = self.ldp200.systemSoftwareVersion
        rev = self.ldp200.systemSoftwareRevision
        s = ''.join(('71', str(ver), '-', chr(rev)))
        return s
    
    def startLaser(self):
        try:
            rw = self.client.write_coil(0x01, 0xff00, self.slaveId)
            if rw.function_code != 0x05:
                ex = LeeLaserError(''.join(('Start laser failed, return code ', str(rw))))
                logging.exception(''.join(('Start laser failed, return code ', str(rw))))
                raise(ex)
            self.readHardware()
        except Exception, e:
            ex = LeeLaserError(''.join(('Start laser command failed.', str(e))))
            logging.exception(''.join(('Start laser command failed.', str(e))))
            raise(ex)

    def shutdownLaser(self):
        try:
            rw = self.client.write_coil(0x01, 0x0000, self.slaveId)
            if rw.function_code != 0x05:
                ex = LeeLaserError(''.join(('Shutdown laser failed, return code ', str(rw))))
                logging.exception(''.join(('Shutdown laser failed, return code ', str(rw))))
                raise(ex)
            self.readHardware()
        except Exception, e:
            ex = LeeLaserError(''.join(('Shutdown laser command failed.', str(e))))
            logging.exception(''.join(('Shutdown laser command failed.', str(e))))
            raise(ex)
            
    def openShutter(self):
        try:
            rw = self.client.write_coil(0x02, 0xff00, self.slaveId)
            if rw.function_code != 0x05:
                ex = LeeLaserError(''.join(('Open shutter failed, return code ', str(rw))))
                logging.exception(''.join(('Open shutter failed, return code ', str(rw))))
                raise(ex)
            self.readHardware()
        except Exception, e:
            ex = LeeLaserError(''.join(('Open shutter command failed.', str(e))))
            logging.exception(''.join(('Open shutter command failed.', str(e))))
            raise(ex)
    
    def closeShutter(self):
        try:
            rw = self.client.write_coil(0x02, 0x0000, self.slaveId)
            if rw.function_code != 0x05:
                ex = LeeLaserError(''.join(('Close shutter failed, return code ', str(rw))))
                logging.exception(''.join(('Close shutter failed, return code ', str(rw))))
                raise(ex)
            self.readHardware()
        except Exception, e:
            ex = LeeLaserError(''.join(('Close shutter command failed.', str(e))))
            logging.exception(''.join(('Close shutter command failed.', str(e))))
            raise(ex)

    def clearFault(self):
        try:
            rw = self.client.write_coil(0x04, 0xff00, self.slaveId)
            if rw.function_code != 0x05:
                ex = LeeLaserError(''.join(('Clear fault failed, return code ', str(rw))))
                logging.exception(''.join(('Clear fault failed, return code ', str(rw))))
                raise(ex)
            self.readHardware()
        except Exception, e:
            ex = LeeLaserError(''.join(('Clear fault command failed.', str(e))))
            logging.exception(''.join(('Clear fault command failed.', str(e))))
            raise(ex)
            
    def setPercentCurrent(self, cp):
        try:
            if (cp >= 0) & (cp <= 100):
                value = int(cp * 10)
                if self.client != None:
                    rw = self.client.write_register(0x01, value, self.slaveId)
                    if rw.function_code != 0x06:
                        ex = LeeLaserError(''.join(('setPercentCurrent failed, return code ', str(rw))))
                        logging.exception(''.join(('setPercentCurrent failed, return code ', str(rw))))
                        raise(ex)
                    self.readHardware()
                else:
                    ex = LeeLaserError('Not connected to LeeLaser')
                    logging.exception('Not connected to LeeLaser')
                    raise(ex)
            else:
                ex = LeeLaserError('Value out of range (0-100).')
                logging.exception('Value out of range (0-100).')
                raise(ex)
        except Exception, e:
            ex = LeeLaserError(''.join(('setPercentCurrent command failed.', str(e))))
            logging.exception(''.join(('setPercentCurrent command failed.', str(e))))
            raise(ex)
    
if __name__ == '__main__':
#    llc = LeeLaserControl('128.100.101.190')
    llc = LeeLaserControl('128.100.101.192', port=502, slaveId=0)
