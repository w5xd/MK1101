 This is a Win32 program that controls the AC0 and AC1 outputs on the FT232H chip made by ftdichip.com 
and used in
the MK-1103 and MK-1104. Those keyers have a solid state relay circuit that routes the TXD and RXD lines
from the FT232H normally to the RS232 level converter and the back panel Rig connector.
If AC0 is held low with AC1 high, the relay latches such that the arduino's TXD/RXD are on the COM port such that
its diagnostics can be run from through the FT232H. The Arduino can also have its program uploaded
with the latch in that state.

The FT232H has EEPROM settings that determine its power up state for AC0 and AC1 (among other settings.)
The EEPROM can be programmed using the FT-PROG program from ftdi: 
<a href='https://ftdichip.com/utilities/#ft_prog'>https://ftdichip.com/utilities/#ft_prog</a>
The MK-1103 and MK-1104 are specified to be programmed with EEPROM setting Drive_0 for AC1 and Drive_1 for
AC0, which routes the serial UART to the RS232 back panel Rig connector.

There are alternatives. Programming the EEPROM for AC1 high and AC0 low puts the FT232H and solid state
relay in diagnostic mode at power up (and therefore disables the Rig connector.)

The power up reset circuit on pin 1 of U20A holds the pin low long
enough to override the EEPROM settings. That means you may use ft_prog to program the FT232H EEPROM for
 AC0 to Drive_1 and AC1 to Tristate which sets
the EEPROM to leave the latch unchanged. That makes this program useful. Invoking this program for that EEPROM
setup sets or clears the latch:
  Diagnostics COMn on    sets the latch for diagnostics
  Diagnostics COMn off   sets the latch for RS232 on the Rig connector.

