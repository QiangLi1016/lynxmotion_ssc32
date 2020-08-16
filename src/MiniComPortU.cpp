//------------------------------------------------------------------------------
// MiniComPort.cpp : Minimalist COM Port Manager, based on Windows file API
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

#include "lynxmotion_ssc32/MiniComPortU.h"
//#include "MiniComPortU.h"

#include <stdio.h>
//------------------------------------------------------------------------------
bool InSet(unsigned char element, TSet s)
{
  return s.c[element>>3] & (1 << (unsigned char)(element&7));
}
//------------------------------------------------------------------------------
TSet EnumerateComPorts()
{
  HANDLE h;
  TSet r;
  for (int i = 0; i < 32; i++) {
    r.c[i] = 0;
    unsigned char m=1;
    for (int j = 0; j < 8; j++) {
      char portName[16];
      sprintf_s(portName,"\\\\.\\COM%d",8*i+j+1);   
      h = CreateFileA(portName,GENERIC_READ | GENERIC_WRITE,0,0,
              OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
      if (h!=INVALID_HANDLE_VALUE) {
            r.c[i] |= m;
            CloseHandle(h);
      }
      m <<= 1;
    }
  }
  return r;
}
//------------------------------------------------------------------------------
bool TMiniComPort::OpenOk(char* portName)
{
  bool result = true;
  char LocalPortName[16] = "\\\\.\\";
  strcat_s(LocalPortName, portName);
  hndl = CreateFileA(
    LocalPortName,
    GENERIC_READ | GENERIC_WRITE,
    0,
    0,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL,
    0
  );
  if (hndl==INVALID_HANDLE_VALUE) result = false;
  else {
    /* Set Timeouts */
    COMMTIMEOUTS to={0};
    to.ReadIntervalTimeout=MAXDWORD;
    to.ReadTotalTimeoutConstant=0;
    to.ReadTotalTimeoutMultiplier=0;
    to.WriteTotalTimeoutConstant=50;
    to.WriteTotalTimeoutMultiplier=10;
    if(!SetCommTimeouts(hndl, &to)) result = false;
  }
  return result;
}
//------------------------------------------------------------------------------
bool TMiniComPort::SetProtocol(DWORD BaudRate, BYTE ByteSize, BYTE StopBits, BYTE Parity)
{
  bool result = true;

  DCB dcb = {0};
  
  dcb.DCBlength=sizeof(dcb);
  if (!GetCommState(hndl, &dcb)) {
    result = false;
  } else {
    dcb.BaudRate = BaudRate;
    dcb.ByteSize = ByteSize;
    dcb.StopBits = StopBits;
    dcb.Parity = Parity;
  }
  if (!SetCommState(hndl, &dcb)) result = false;

  return result;
}
//------------------------------------------------------------------------------
unsigned int TMiniComPort::Transmit( char* buffer, unsigned int count)
{
  unsigned long result;
  WriteFile(hndl, buffer, count, &result, NULL);
  return result;
}
//------------------------------------------------------------------------------
unsigned int TMiniComPort::Receive( char* buffer, unsigned int count)
{
  unsigned long result;
  ReadFile(hndl, buffer, count, &result, NULL);
  return result;
}
//------------------------------------------------------------------------------
void TMiniComPort::Close()
{
  CloseHandle(hndl);
}
//------------------------------------------------------------------------------
