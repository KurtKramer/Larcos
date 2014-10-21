#pragma once

using namespace System;

namespace LarcosCounterManaged 
{

  /**
   *@class  UmiScannerClock
   *@brief Used byu Scanner file routines to keep track of most currtent buffers.
   *@details 
   *@code
   *****************************************************************************
   ** Used by SscannerFileBuffered to keep track of age if Buffered Frames     *
   ** Every time a buffered frame is accesed it will update its TimeStamp      *
   ** from a global instance of 'UmiScannerClock'                              *
   *****************************************************************************
   *@endcode
  */
  public  ref class UmiScannerClock
  {
  public:
    UmiScannerClock ();

    property  long  Time  {long  get ();   void  set (long _time);}

  private:
    long  time;
  };
}


