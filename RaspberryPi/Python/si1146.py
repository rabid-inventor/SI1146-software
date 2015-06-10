SI1146ADDR = 	0x60  #Devices Default i2c address

#Registers
PART_ID = 		0x00
REV_ID = 		0x01
SEQ_ID = 		0x02 
INT_CFG = 		0x03
IRQ_ENABLE = 	0x04 
HW_KEY = 		0x07
MEAS_RATE0 = 	0x08
MEAS_RATE1 = 	0x09
PS_LED21 = 		0x0F
PS_LED3 = 		0x10
UCOEF0 = 		0x13
UCOEF1 = 		0x14
UCOEF2 = 		0x15
UCOEF3 = 		0x16
PARAM_WR = 		0x17
COMMAND =  		0x18 
RESPONSE = 		0x20
IRQ_STATUS =  	0x21 
ALS_VIS_DATA0 = 0x22
ALS_VIS_DATA1 = 0x23
ALS_IR_DATA0  = 0x24 
ALS_IR_DATA1 = 	0x25
PS1_DATA0 = 	0x26
PS1_DATA1 = 	0x27
PS2_DATA0 = 	0x28
PS2_DATA1 = 	0x29
PS3_DATA0  = 	0x2A 
PS3_DATA1 = 	0x2B

AUX_DATA0_UVINDEX0 = 0x2C
AUX_DATA1_UVINDEX1 = 0x2D
PARAM_RD = 		0x2E
CHIP_STAT = 	0x30
ANA_IN_KEY =  	0x3B

#Commands for entering into command Resegister

COM_ALS_FORCE = 	0b00000110 #Forces a single ALS measurement
COM_PSAL_FORCE =	0b00000111 #Forces a single PS and ALS measurement
COM_PS_PAUSE = 		0b00001001  #Pauses autonomous PS
COM_ALS_PAUSE = 	0b00001010  #Pauses autonomous ALS
COM_PSAL_PAUSE =	0b00001011  #Pauses PS and ALS
COM_PS_AUTO = 		0b00001101   #Starts/Restarts an autonomous PS Loop
COM_ALS_AUTO =  	0b00001110   #Starts/Restarts an autonomous ALS Loop
COM_PSALS_AUTO = 	0b00001111   #Starts/Restarts autonomous ALS and PS loop

COM_HW_KEY = 0x17 #Key to set Hardware from Sleep to Active mode.

#Hardware Ids

SI1146 = 	0b01000110

class chipStat():
  def __init__(self):
    self.running = 0 
	self.suspend = 0
	self.sleep = 0
    
  

chip = chipStat()

import smbus
from  time import sleep as sleep


def init(bus):
  bus = smbus.SMBus(bus)
  if (bus.read_byte_data(SI1146ADDR, PART_ID) == SI1146):
    print((' SI1146 Detected on I2C BUS ' + char(bus)))
  elif(): 
    print(('SI1146 Not found on I2C BUS  '+ char(bus)))
    exit()



#TODO Check status 

def getStatus():
  RUNNING = 0b00000100
  SUSPEND = 0b00000010
  SLEEP   = 0b00000001
  status = bus.read_byte_data(SI1146ADDR, CHIP_STAT)
  return ( (status & RUNNING) , (status & SUSPEND) ,(status&SLEEP))
  
  













init(1)
