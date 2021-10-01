Sends VDM to Mac Mini upon connection, to go into DFU mode. FUSB302B is connected to AVR using I2C and GPIO.

The library it taken from https://github.com/ReclaimerLabs/FUSB302/blob/master/FUSB302.cpp

AVR    FUSB302B  
SDA <> SDA  
SCL <> SCL  
VBUS <> D4  
GND <> GND  
INT <> D12  
+3.3v <> VPU and VDD  
