#include <Windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include "ftd2xx.h"
#include <iostream>
#include <functional>
#include <vector>
#include <memory>
#include <algorithm>

/* This program controls the AC0 and AC1 outputs on the FT232H chip made by ftdichip.com and used in
* the MK-1103 and MK-1104. Those keyers have a solid state relay circuit on the TXD and RXD lines
* from the FT232H that are normally routed to the RS232 level converter and the back panel Rig connector.
* If AC0 is held low with AC1 high, the relay latches so that the arduino is on the COM port such that
* its diagnostics can be run from through the FT232H. The Arduino can also have its program uploaded
* with the latch in that state.
* 
* The FT232H has EEPROM settings that determine its power up state for AC0 and AC1 (among other settings.)
* The EEPROM can be programmed using the FT-PROG program from ftdi: https://ftdichip.com/utilities/#ft_prog
* The MK-1103 and MK-1104 are specified to be programmed with EEPROM setting Drive_0 for AC1 and Drive_1 for
* AC0, which routes the serial UART to the RS232 back panel Rig connector.
* 
* There are alternatives. Programming the EEPROM for AC1 high and AC0 low puts the FT232H and solid state
* relay in diagnostic mode at power up (and therefore disables the Rig connector.)
* 
* The power up reset circuit on pin 1 of U20A unfortunately doesn't hold the pin low long
* enough to override the EEPROM settings. But programming the EEPROM for AC0 to Drive_1 and AC1 to Tristate sets
* the EEPROM to leave the latch unchanged. That makes this program useful. Invoking this program for that EEPROM
* setup sets or clears the latch:
*   Diagnostics COMn on    sets the latch for diagnostics
*   Diagnostics COMn off   sets the latch for RS232 on the Rig connector.
*/

typedef std::function<bool(FT_HANDLE, const std::string&)> iterFcn_t; // function OWNS FT_HANDLE
static void iterateDeviceInfoList(const iterFcn_t& fcn)
{   // use the ftd2xx.dll, auto-magically installed by the FT232H USB device drivers, to iterate
    // through all the ftdi chips on this machine.
    DWORD nc;
    FT_STATUS stat = FT_CreateDeviceInfoList(&nc);
    if ((FT_OK == stat) && nc > 0)
    {
        std::vector<FT_DEVICE_LIST_INFO_NODE> nodes(nc);
        stat = FT_GetDeviceInfoList(&nodes[0], &nc);
        for (int idx = 0; idx < static_cast<int>(nc); idx++)
        {
            if (nodes[idx].Flags & FT_FLAGS_OPENED)
                continue;
            FT_HANDLE handle;
            stat = FT_OpenEx(&nodes[idx].SerialNumber[0], FT_OPEN_BY_SERIAL_NUMBER, &handle);
            if (stat == FT_OK)
            {
                if (fcn(handle, nodes[idx].SerialNumber))
                {
                    std::cout << "Comm port found" << std::endl;
                    return;
                }
            }
        }
    }
    std::cout << "Comm port not found" << std::endl;
}

namespace {
    const ULONG READ_TIMEOUT_MSEC = 950;
    const ULONG WRITE_TIMEOUT_MSEC = 950;
    const unsigned char LATENCY_MSEC = 2;

    unsigned char ONLY_DRIVE_ZERO = 0x9e;
    unsigned char SET_DATA_HIGH_BYTE = 0x82;
    unsigned char SET_DATA_LOW_BYTE = 0x80;


    FT_STATUS ClearReadQueue(FT_HANDLE handle)
    {
        DWORD dwNumInputBuffer;
        // Purge USB receive buffer ... Get the number of bytes in the FT232H receive buffer and then read them
        FT_STATUS ftStatus = FT_GetQueueStatus(handle, &dwNumInputBuffer);
        while ((ftStatus == FT_OK) && (dwNumInputBuffer > 0))
        {
            DWORD dwNumBytesRead;
            char InputBuffer[16];
            DWORD r = std::min(static_cast<DWORD>(sizeof(InputBuffer)), dwNumInputBuffer);
            ftStatus |= ::FT_Read(handle, &InputBuffer, r, &dwNumBytesRead);
            dwNumInputBuffer -= r;
        }
        return ftStatus;
    }

    bool SyncBadCommand(FT_HANDLE handle, BYTE b)
    {
        DWORD dwNumBytesSent;
        std::vector<BYTE> OutputBuffer;
        OutputBuffer.push_back(b);											// Add an invalid command 
                                                                            // Send off the invalid command
        FT_STATUS ftStatus = FT_Write(handle, &OutputBuffer[0], OutputBuffer.size(), &dwNumBytesSent);

        // Check if the bytes were sent off OK
        if (ftStatus != FT_OK || OutputBuffer.size() != dwNumBytesSent)
            return false;

        // Now read the response from the FT232H. It should return error code 0xFA followed 
        //     by the actual bad command 0xAA
        // Wait for the two bytes to come back 

        DWORD dwNumInputBuffer = 0;
        int ReadTimeoutCounter = 50; // maximum tries

        while (true)
        {
            ftStatus = FT_GetQueueStatus(handle, &dwNumInputBuffer);	// Get number of bytes in the input buffer
            if (ftStatus != FT_OK)
                return false;
            if (dwNumInputBuffer >= 2)
                break;
            if (--ReadTimeoutCounter <= 0)
                return false;
            Sleep(2);
        }

        if (dwNumInputBuffer > 512)
            return false; // there should not be anywhere near this much data
        DWORD dwNumBytesRead;
        std::vector<BYTE> InputBuffer(dwNumInputBuffer);
        ftStatus = FT_Read(handle, &InputBuffer[0], dwNumInputBuffer, &dwNumBytesRead);
        if (ftStatus != FT_OK || dwNumBytesRead != dwNumInputBuffer)
            return false;
        if ((InputBuffer[dwNumBytesRead - 2] != BYTE(0xFA)) || (InputBuffer[dwNumBytesRead - 1] != b))
            return false;
        return true;
    }

    FT_STATUS SetFT232H(FT_HANDLE handle, bool SetDiagnostics)
    {
        FT_STATUS ftStatus = FT_OK;
        ftStatus |= FT_ResetDevice(handle);
        ftStatus |= FT_SetTimeouts(handle, READ_TIMEOUT_MSEC, WRITE_TIMEOUT_MSEC);
        ftStatus |= FT_SetUSBParameters(handle, 0x10000, 0x10000);			// Set USB request transfer sizes
        ftStatus |= FT_SetFlowControl(handle, FT_FLOW_NONE, 0, 0);
        ftStatus |= FT_SetChars(handle, false, 0, false, 0);				// Disable event and error characters

        ftStatus |= FT_SetLatencyTimer(handle, LATENCY_MSEC);
        ftStatus |= FT_SetBitMode(handle, 0x0, 0x00); 					// Reset the mode to whatever is set in EEPROM
        ftStatus |= FT_SetBitMode(handle, 0x0, 0x02);	 					// Enable MPSSE mode.. COM port UART won't work now.
        if (ftStatus != FT_OK)
            return ftStatus; // really can't talk to this device

        // #########################################################################################
        // Synchronise the MPSSE by sending bad command AA to it
        // #########################################################################################
        ftStatus |= ClearReadQueue(handle);
        if (!SyncBadCommand(handle, 0xAA))
            return FT_OTHER_ERROR; // its probably not FT232H
        if (!SyncBadCommand(handle, 0xAB))
            return FT_OTHER_ERROR;

        // #########################################################################################
        // Configure the MPSSE settings
        // #########################################################################################

        std::vector<BYTE> OutputBuffer;

        OutputBuffer.push_back(ONLY_DRIVE_ZERO); 		// Enable the FT232H's drive-zero mode on the lines used for I2C ...
        OutputBuffer.push_back(0);		// ...lower port (AD0, AD1, AD2)
        OutputBuffer.push_back(~0x01);		// ...and  AC0  drive high    }
        OutputBuffer.push_back(SET_DATA_HIGH_BYTE);	// Command to set directions of ACbus and data values for pins set as o/p
        OutputBuffer.push_back(SetDiagnostics ? 2 : 1);	// ONE of AC0 or AC1 set. AC1 is Diagnostics OFF, AC0 is diagnostics ON
        OutputBuffer.push_back(3);   // direction
        DWORD dwNumBytesSent;
        ftStatus = FT_Write(handle, &OutputBuffer[0], OutputBuffer.size(), &dwNumBytesSent);
        SyncBadCommand(handle, 0xAA);
        ftStatus |= FT_SetBitMode(handle, 0, 0); 					// Reset the mode. The FT232H EEPROM settings for AC0/AC1 are invoked
        // don't bother trying to omit this FT_SetBitMode to 0,0 in hopes of keeping AC0/AC1 as latched. The chip won't run its UART that way. 
        // that is, the serial port won't work
        return ftStatus;
    }


    bool ScanFT232(FT_HANDLE handle, int lookingForPort, bool SetDiagnostics)
    {
        LONG portnum = -1;
        bool ret = false;
        FT_GetComPortNumber(handle, &portnum);
        if (portnum == lookingForPort)
        {
            SetFT232H(handle, SetDiagnostics);
            ret = true;
        }
        FT_Close(handle);
        return ret;
    }
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: Diagnostics COM<number> < on | off >" << std::endl;
        return 1;
    }
    std::string com;
    for (const char* p = argv[1]; *p; p += 1)
        com.push_back(static_cast<char>(toupper(*p)));
    int portNum = -1;
    if ((com[0]) == 'C' && com[1] == 'O' && com[2] == 'M')
    {
        com = com.substr(3);
        portNum = atoi(com.c_str());
    }
    if (portNum <= 0)
    {
        std::cerr << "First argument, " << argv[1] << " must be COMn where n is port number" << std::endl;
        return 1;
    }
    bool SetDiagnostics;
    std::string val;
    for (const char* p = argv[2]; *p; p += 1)
        val.push_back(static_cast<char>(toupper(*p)));
    
    if (val == "ON")
        SetDiagnostics = true;
    else if (val == "OFF")
        SetDiagnostics = false;
    else
    {
        std::cerr << "Second argument, " << argv[2] << ", must be ON or OFF" << std::endl;
        return 1;
    }

    iterateDeviceInfoList(std::bind(&ScanFT232, std::placeholders::_1, portNum, SetDiagnostics));
    return 0;
}

