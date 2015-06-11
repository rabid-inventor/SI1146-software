SI1146ADDR = 0x60 #Devices Default i2c address


#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
#Registers
PART_ID = 		0x00
REV_ID = 		0x01
SEQ_ID = 		0x02 
INT_CFG = 		0x03
IRQ_ENABLE = 		0x04
IRQ_MODE1 = 		0x05
IRQ_MODE2 = 		0x06 
HW_KEY = 		0x07
MEAS_RATE0 = 		0x08
MEAS_RATE1 = 		0x09
PS_LED21 = 		0x0F
PS_LED3 = 		0x10
UCOEF0 = 		0x13
UCOEF1 = 		0x14
UCOEF2 = 		0x15
UCOEF3 = 		0x16
PARAM_WR = 		0x17
COMMAND =  		0x18 
RESPONSE = 		0x20
IRQ_STATUS =  		0x21 

#DATA REGISTERS this is where the sensor data is stored
ALS_VIS_DATA0 = 	0x22
ALS_VIS_DATA1 = 	0x23
ALS_IR_DATA0  = 	0x24 
ALS_IR_DATA1 = 		0x25
PS1_DATA0 = 		0x26
PS1_DATA1 = 		0x27
PS2_DATA0 = 		0x28
PS2_DATA1 = 		0x29
PS3_DATA0  = 		0x2A 
PS3_DATA1 = 		0x2B
AUX_DATA0_UVINDEX0 = 	0x2C
AUX_DATA1_UVINDEX1 = 	0x2D

#Misc

PARAM_RD = 		0x2E
CHIP_STAT = 		0x30
ANA_IN_KEY =  		0x3B

#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
#Commands for entering into command Resegister
COM_PARAM_QUERY =	0x80
COM_PARAM_SET =  	0xA0
COM_NOP = 		0b00000000
COM_RESET = 		0b00000001
COM_ALS_FORCE = 	0b00000110 	#Forces a single ALS measurement
COM_PSAL_FORCE =	0b00000111 	#Forces a single PS and ALS measurement
COM_PS_PAUSE = 		0b00001001  	#Pauses autonomous PS
COM_ALS_PAUSE = 	0b00001010  	#Pauses autonomous ALS
COM_PSAL_PAUSE =	0b00001011  	#Pauses PS and ALS
COM_PS_AUTO = 		0b00001101   	#Starts/Restarts an autonomous PS Loop
COM_ALS_AUTO =  	0b00001110   	#Starts/Restarts an autonomous ALS Loop
COM_PSALS_AUTO = 	0b00001111   	#Starts/Restarts autonomous ALS and PS loop
COM_HW_KEY = 		0x17 		#Key to set Hardware from Sleep to Active mode.

#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
#Parameters and Options  

PARAM_I2C_ADDR = 		0x00

PARAM_CHLIST =			0x01
PARAM_CHLIST_EN_UV = 		0x80
PARAM_CHLIST_EN_AUX = 		0x40
PARAM_CHLIST_EN_ALS_IR =	0x20
PARAM_CHLIST_EN_ALS_VIS=	0x10
PARAM_CHLIST_EN_PS1 = 		0x01
PARAM_CHLIST_ENPS2 = 		0x02
PARAM_CHLIST_ENPS3 =		0x04

PARAM_PSLED12SEL =  		0x02
PARAM_PSLED12SEL_PS2NONE = 	0x00
PARAM_PSLED12SEL_PS2LED1 = 	0x10
PARAM_PSLED12SEL_PS2LED2 =  	0x20
PARAM_PSLED12SEL_PS2LED3 = 	0x40
PARAM_PSLED12SEL_PS1NONE =	0x00
PARAM_PSLED12SEL_PS1LED1 =	0x01
PARAM_PSLED12SEL_PS1LED2 =	0x02
PARAM_PSLED12SEL_PS1LED3 =	0x04

PARAM_PSLED3SEL =		0x03
PARAM_PSENCODE =		0x05
PARAM_ALSENCODE =		0x06

PARAM_PS1ADCMUX =		0x07
PARAM_PS2ADCMUX =		0x08
PARAM_PS3ADCMUX =		0x09
PARAM_PSADCOUNTER =		0x0A
PARAM_PSADCGAIN =		0x0B
PARAM_PSADCMISC =		0x0C
PARAM_PSADCMISC_RANGE =		0x20
PARAM_PSADCMISC_PSMODE =	0x04

PARAM_ALSIRADCMUX =		0x0E
PARAM_AUXADCMUX =		0x0F

PARAM_ALSVISADCOUNTER =		0x10
PARAM_ALSVISADCGAIN =		0x11
PARAM_ALSVISADCMISC =		0x12
PARAM_ALSVISADCMISC_VISRANGE =	0x20

PARAM_ALSIRADCOUNTER =		0x1D
PARAM_ALSIRADCGAIN =		0x1E
PARAM_ALSIRADCMISC =		0x1F
PARAM_ALSIRADCMISC_RANGE =	0x20

PARAM_ADCCOUNTER_511CLK =	0x70

PARAM_ADCMUX_SMALLIR  =		0x00
PARAM_ADCMUX_LARGEIR  =		0x03


#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
#Hardware Ids
SI1145 = 		0b01000101
SI1146 = 		0b01000110
SI1147 = 		0b01000111


class chipStat():
  def __init__(self):
    self.running = 0 
    self.suspend = 0
    self.sleep = 0
    
  

chip = chipStat()

import smbus
from  time import sleep as sleep

bus = smbus.SMBus(1)

def init():
  if (bus.read_byte_data(SI1146ADDR, PART_ID) == SI1146):
    print((' SI1146 Detected on I2C BUS ' + str(bus)))
  elif(): 
    print(('SI1146 Not found on I2C BUS  '+ str(bus)))
    exit()

def combineValues(MSB,LSB):
  value = (MSB << 8 ) + LSB
  return value

def setReg(reg , data):
  bus.write_block_data(SI1146ADDR, reg, [data])

def getReg(reg):
  data = bus.read_byte_data(SI1146ADDR,reg)
  return data  

#TODO Check status 

def getStatus():
  RUNNING = 0b00000100
  SUSPEND = 0b00000010
  SLEEP   = 0b00000001
  status = bus.read_byte_data(SI1146ADDR, CHIP_STAT)
  chip.running =  (status & RUNNING)
  return ( (status & RUNNING)/RUNNING  , (status & SUSPEND)/SUSPEND ,(status&SLEEP)/SLEEP)


def sendReset(delay):
  print('Resetting SI114X........')
  setReg(MEAS_RATE0, 0)
  setReg(MEAS_RATE1, 0)
  setReg(IRQ_ENABLE, 0)
  setReg(IRQ_MODE1, 0)
  setReg(IRQ_MODE2, 0)
  setReg(INT_CFG, 0)
  setReg(IRQ_STATUS,0xFF)
  bus.write_byte_data(SI1146ADDR, COMMAND, COM_RESET)
  sleep(delay)
  sysKey(COM_HW_KEY)
  sleep(delay)

def sendCommand(command):
  while (getResponse() > 0):
    bus.write_byte_data(SI1146ADDR, COMMAND, 0x00)
    sleep(0.1)
  while (getResponse() == 0):
    bus.write_byte_data(SI1146ADDR, COMMAND, command)
  print('OK')
  return  

def setParam(param,data):  
  while (getResponse() > 0):
    bus.write_byte_data(SI1146ADDR, COMMAND, 0x00)
    sleep(0.1)

  setReg(PARAM_WR,data)
  sleep(0.2)
  print(COM_PARAM_SET|param)
  bus.write_byte_data(SI1146ADDR, COMMAND,(COM_PARAM_SET|param))
  if (getReg(PARAM_RD)==data):
    print(getReg(PARAM_RD),data,COM_PARAM_SET | param)
    print('PARAM OK')
    return
  print(getReg(PARAM_RD), data, COM_PARAM_SET|param , getResponse())
 # dumpRam()

  #exit()

def getResponse():
  response = bus.read_byte_data(SI1146ADDR,RESPONSE)
  return response

def setRunning():
  bus.write_byte_data(SI1146ADDR, HW_KEY , COM_HW_KEY)
  return

def readSensor( sensor):
  lsb = getReg(sensor-1)
  msb = getReg(sensor)
  data = combineValues(msb,lsb)
  return data

def dumpRam(): 
  for location in range(0,20):
    sendCommand(COM_PARAM_QUERY+location)
    data = getReg(PARAM_RD)
    print (location ,' = ' ,data)


def sysKey(key):
  setReg(HW_KEY, key)
  return 

def si114xSetup():
  #Setup UVindex Measurement Coefficients
  setReg(UCOEF0, 0x29)
  setReg(UCOEF1, 0x89)
  setReg(UCOEF2, 0x02)
  setReg(UCOEF3, 0x00)

  #enable UV sensor
  setParam(PARAM_CHLIST, PARAM_CHLIST_ENUV | PARAM_CHLIST_ENALSIR | PARAM_CHLIST_ENALSVIS | PARAM_CHLIST_ENPS1)
  #enable interrupt on every sample
  setReg(INT_CFG, INT_CFG_INTOE)  
  setReg(IRQ_ENABLE, IRQ_EN_ALSEVERYSAMPLE);  

#/****************************** Prox Sense 1 */

#  // program LED current
  setReg(PSLED_21, 0x03)# // 20mA for LED 1 only
  setParam(PARAM_PS1ADCMUX,PARAM_ADCMUX_LARGEIR);

#  // prox sensor #1 uses LED #1
  setParam(PARAM_PSLED12SEL, PARAM_PSLED12SEL_PS1LED1);
#  // fastest clocks, clock div 1
  setParam(PARAM_PSADCGAIN, 0);
#  // take 511 clocks to measure
  setParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
#  // in prox mode, high range
  setParam(PARAM_PSADCMISC, PARAM_PSADCMISC_RANGE | PARAM_PSADCMISC_PSMODE);

  setParam(PARAM_ALSIRADCMUX, PARAM_ADCMUX_SMALLIR);  
#  // fastest clocks, clock div 1
  setParam(PARAM_ALSIRADCGAIN, 0);
#  // take 511 clocks to measure
  setParam(PARAM_ALSIRADCOUNTER, PARAM_ADCCOUNTER_511CLK);
#  // in high range mode
  setParam(PARAM_ALSIRADCMISC, PARAM_ALSIRADCMISC_RANGE);



  #fastest clocks, clock div 1
  setParam(PARAM_ALSVISADCGAIN, 0);
  #take 511 clocks to measure
  setParam(PARAM_ALSVISADCOUNTER, PARAM_ADCCOUNTER_511CLK);
  # in high range mode (not normal signal)
  setParam(PARAM_ALSVISADCMISC, PARAM_ALSVISADCMISC_VISRANGE);


#/************************/

  # measurement rate for auto
  regSet(MEAS_RATE0, 0xFF); #// 255 * 31.25uS = 8ms
  
  # auto run
  regSet(COMMAND, COM_PSALS_AUTO);

    

init()
#sendReset(0.5)
#sysKey(0x17)

si114xSetup()

cemment = '''
print(getStatus())
setParam(0x01, 0xF7)


print(getResponse(), ' Commands Sent')

setParam(0x05, 0b0001000)
setParam(0x05, 0b0001000)
sleep(0.1)
setReg(MEAS_RATE0, 0xFF)
#setReg(MEAS_RATE1, 0x03)
setReg(PS_LED21, 0xFF)
#setReg(PS_LED3,0x03)

setParam(1, 0b00000000)
setParam(2, 0b00000111)

sendCommand(COM_PSALS_AUTO)

setRunning()
print(getStatus())
print(chip.running)

dumpRam()
'''
while 1: 
  print (readSensor(PS2_DATA1),readSensor(PS1_DATA1),readSensor(ALS_VIS_DATA1))
  print (getResponse())
  print(getStatus())  
  sleep(0.1)

