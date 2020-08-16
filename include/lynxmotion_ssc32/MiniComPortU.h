//------------------------------------------------------------------------------
// MiniComPort.h : Minimalist COM Port Manager, based on Windows file API
// author : Jean-Paul PETILLON
// Adapted from the following National Instrument publication:
// "Using the Windows SDK APIs for Serial Port Communication in LabWindows™/CVI"
// URL : http://digital.ni.com/public.nsf/allkb/37EC9685EC1C87B4862579D1004095C7
//
// V2.0 : - added a com port enumeration feature (usefull to offer the user a
//          combo-box to select the desired port over the existing ports on the PC
//        - added the prefix "\\\\.\\COM" to fix the issue with opening
//          COM's greater than 9 (source : http://support.microsoft.com/kb/115831)
//
// V3.0 : - modification of ReadIntervalTimeout, ReadTotalTimeoutConstant and of
//          ReadTotalTimeoutMultiplier to reduce waiting time to 0 when reading
//          the Com Port (previous values slowed excessively the calling program)
//          Documentation :
//          https://msdn.microsoft.com/en-us/library/windows/desktop/aa363190%28v=vs.85%29.aspx
//------------------------------------------------------------------------------

#ifndef MiniComPortUH
#define MiniComPortUH

#include "windows.h" // to access windows file API

//------------------------------------------------------------------------------
// set of up to 256 elements (pascal style)
typedef struct {
  unsigned char c[32];
} TSet;

// checks whether an element belongs to a set
bool InSet(unsigned char element, TSet s);

// serial ports enumeration
TSet EnumerateComPorts();
//------------------------------------------------------------------------------

class TMiniComPort
{
private:
	HANDLE hndl;
public:
	// opening
	bool OpenOk(char* portName);
	// settings
	bool SetProtocol(
	  DWORD BaudRate,// 110, ... 9600, 19200, ... 921600 (depending on UART capabilities
	  BYTE ByteSize, // 4, 5, 6, 7 ou 8
	  BYTE StopBits, // ONESTOPBIT, ONE5STOPBITS, TWOSTOPBITS
	  BYTE Parity	 // NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY, SPACEPARITY
	);
	// transmission
	unsigned int Transmit( char* buffer, unsigned int count);
	// reception
	unsigned int Receive( char* buffer, unsigned int count);
	// closure
	void Close();
};
//------------------------------------------------------------------------------
#endif // ifndef MiniComPortUH
