/* CameraAcquisitionPleora.cpp -- Represents a single camera frame.
 * Copyright (C) 2011-2013  Kurt Kramer
 * For conditions of distribution and use, see copyright notice in CounterUnManaged.txt
 */
#include "FirstIncludes.h"

#include <errno.h>
#include <istream>
#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#if  defined(KKOS_WINDOWS)
#include <windows.h>
#endif
#include <math.h>
#include "MemoryDebug.h"
using namespace std;


#include <PvDeviceFinderWnd.h>
#include <PvDeviceGEV.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvStreamGEV.h>
//#include <PvStreamRaw.h>
#include <PvSystem.h>

#include "KKBaseTypes.h"
#include "GoalKeeper.h"
#include "OSservices.h"
#include "RunLog.h"
using namespace KKB;


#include "ScannerFile.h"
#include "ScannerFileEntry.h"
using namespace  KKLSC;


#include "CounterVariables.h"
using namespace  CounterBase;


#include "CameraAcquisitionPleora.h"
#include "CameraAcquisition.h"
#include "CameraFrame.h"
#include "CameraFrameBuffer.h"
#include "CameraAcquisitionPleora.h"
#include "CameraParameters.h"
#include "InstallationConfig.h"
#include "CounterManager.h"
using  namespace  CounterUnManaged;


// 2013-10-19,   KAK,  Will learned possible way to get to talk thru camera to get flow data,
//                     Seems bug in firmware where we have to set format to "NoConnect"  then
//                     to "TTL";  this will force the command to propagate thru the rest if the 
//                     camera.
//
// LineFormat  turn to "NoConnect(0)"  then back to "TTL(1):
//  Input to lineSelector  to Line0, followed by NoConnect,  then TTL

// OutPut
// OutputSelector to Line1
// OutputFormat set to "NoConnect"  followed by  "TTL".



CameraAcquisitionPleora::CameraAcquisitionPleora 
                                     (CounterManagerPtr _manager,
                                      const KKStr&            _macAddress,
                                      CameraFrameBufferPtr    _frameBuffer,
                                      FlowMeterTrackerPtr     _flowMeter,
                                      MsgQueuePtr             _msgQueue,
                                      const KKStr&            _threadName
                                     ):
    CameraAcquisition (_manager, _frameBuffer, _flowMeter, _msgQueue, _threadName),

    lDevice                    (NULL),
    lDeviceParams              (NULL),
    lPipeline                  (NULL),
    lStream                    (NULL),
    lStreamParams              (NULL),

    lAcquisitionLinePeriod     (NULL),
    lAnalogGainAbs             (NULL),
    lCameraTemparature         (NULL),
    lDigitalGainRaw            (NULL),
    lExposureTime              (NULL),
    lFrameRate                 (NULL),
    lGain                      (NULL),
    lGainSelector              (NULL),
    lIPAddressParam            (NULL),
    lPayloadSize               (NULL),
    lReadVoltageAndTemperature (NULL),
    lScanRate                  (NULL),
    lSensitivityMode           (NULL),
    lStart                     (NULL),
    lStop                      (NULL),
    lTLLocked                  (NULL),

    oneSecFrameInterval        (41),    /**< Number of frames at 480 Scan Lines per frame 20K scan rate for  1 second interval. */
    twoSecFrameInterval        (82),
    threeSecFrameInterval      (123),
    fiveSecFrameInterval       (205),
    tenSecFrameInterval        (410),
    voltTempReadInterval       (416),   /**< Number of frames at 480 Scan Lines per frame 20K scan rate for 10 second interval.  */

    embeddedFlowMeter          (false),
    generateFlowRateReport     (false),

    // Variables needed to support AutoGain.
    agAnalogGain               (-5.0),
    analogGainMin              (-10.0f),
    analogGainMax              (10.0f),
    agDigitalGain              (1000),
    digitalGainMax             (10000),
    gainTapSel                 (-1)

{
  cameraParams->MacAddress (_macAddress);
  if  (Manager ()->Installation ()->FlowMeterMethod () == InstallationConfig::FlowMeterMethods::Embedded)
    embeddedFlowMeter = true;
  else
    embeddedFlowMeter = false;
}



CameraAcquisitionPleora::CameraAcquisitionPleora 
                                     (CounterManagerPtr _manager,
                                      CameraParametersPtr     _cameraParams,
                                      CameraFrameBufferPtr    _frameBuffer,
                                      FlowMeterTrackerPtr     _flowMeter,
                                      MsgQueuePtr             _msgQueue,
                                      const KKStr&            _threadName
                                     ):
    CameraAcquisition (_manager, _cameraParams, _frameBuffer, _flowMeter, _msgQueue, _threadName),

    lDevice                    (NULL),
    lDeviceParams              (NULL),
    lPipeline                  (NULL),
    lStream                    (NULL),
    lStreamParams              (NULL),

    lAcquisitionLinePeriod     (NULL),
    lAnalogGainAbs             (NULL),
    lCameraTemparature         (NULL),
    lDigitalGainRaw            (NULL),
    lExposureTime              (NULL),
    lFrameRate                 (NULL),
    lGain                      (NULL),
    lGainSelector              (NULL),
    lIPAddressParam            (NULL),
    lPayloadSize               (NULL),
    lReadVoltageAndTemperature (NULL),
    lScanRate                  (NULL),
    lSensitivityMode           (NULL),
    lStart                     (NULL),
    lStop                      (NULL),
    lTLLocked                  (NULL),

    oneSecFrameInterval        (41),    /**< Number of frames at 480 Scan Lines per frame 20K scan rate for  1 second interval. */
    twoSecFrameInterval        (82),
    threeSecFrameInterval      (123),
    fiveSecFrameInterval       (205),
    tenSecFrameInterval        (410),
    voltTempReadInterval       (416),   /**< Number of frames at 480 Scan Lines per frame 20K scan rate for 10 second interval.  */

    embeddedFlowMeter          (false),
    generateFlowRateReport     (false),

    // Variables needed to support AutoGain.
    agAnalogGain               (-5.0),
    analogGainMin              (-10.0f),
    analogGainMax              (10.0f),
    agDigitalGain              (1000),
    digitalGainMax             (10000),
    gainTapSel                 (-1)

{
  if  (Manager ()->Installation ()->FlowMeterMethod () == InstallationConfig::FlowMeterMethods::Embedded)
    embeddedFlowMeter = true;
  else
    embeddedFlowMeter = false;
}



CameraAcquisitionPleora::~CameraAcquisitionPleora ()
{
}



void  CameraAcquisitionPleora::InitializeDeviceParameterAccessVariables ()
{
  lDeviceParams = lDevice->GetParameters();
  lIPAddressParam            = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("IPAddressParam"));
  lTLLocked                  = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("TLParamsLocked"));
  lPayloadSize               = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("PayloadSize"));
  lStart                     = dynamic_cast<PvGenCommand *> (lDeviceParams->Get ("AcquisitionStart"));
  lStop                      = dynamic_cast<PvGenCommand *> (lDeviceParams->Get ("AcquisitionStop"));
  lReadVoltageAndTemperature = dynamic_cast<PvGenCommand *> (lDeviceParams->Get ("ReadVoltageAndTemperature"));
  lScanRate                  = dynamic_cast<PvGenFloat   *> (lDeviceParams->Get ("AcquisitionLineRateAbs"));
  lGainSelector              = dynamic_cast<PvGenEnum    *> (lDeviceParams->Get ("GainSelector"));
  lAnalogGainAbs             = dynamic_cast<PvGenFloat   *> (lDeviceParams->Get ("GainAbs"));
  lDigitalGainRaw            = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("DigitalGainRaw"));

  lSensitivityMode           = dynamic_cast<PvGenEnum    *> (lDeviceParams->Get ("SensitivityMode"));
  lGain                      = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("Gain"));
  lAcquisitionLinePeriod     = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("AcquisitionLinePeriod"));
  lExposureTime              = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("ExposureTime"));
  lCameraTemparature         = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("CameraTemparature"));

  if  (!lCameraTemparature)
    lCameraTemparature = dynamic_cast<PvGenInteger *>(lDeviceParams->Get ("DeviceTemperature"));
}  /* InitializeDeviceParameterAccessVariables */



void  CameraAcquisitionPleora::InitializeStreamParameterAccessVariables ()
{
  lStreamParams = lStream->GetParameters ();
  lFrameRate    = dynamic_cast<PvGenFloat *>  (lStreamParams->Get ("AcquisitionRateAverage"));
}



const char*  CameraAcquisitionPleora::PvGenTypeToCStr (PvGenType  genType)
{
  const char* s = "UnKnown";
  switch  (genType)
  {
  case  PvGenTypeInteger:    s = "Integer";    break;
  case  PvGenTypeEnum:       s = "Enum";       break;
  case  PvGenTypeBoolean:    s = "Boolean";    break;
  case  PvGenTypeString:     s = "String";     break;
  case  PvGenTypeCommand:    s = "Command";    break;
  case  PvGenTypeFloat:      s = "Float";      break;
  case  PvGenTypeRegister:   s = "register";   break;
  case  PvGenTypeUndefined:  s = "Undefined";  break;
  default:
    s = "UnKnown";
  }

  return s;
}  /* PvGenTypeToCStr */



double  CameraAcquisitionPleora::GetPvParameterFloat (PvGenParameterArray*  params,
                                                      const KKStr&          paramName,
                                                      double                defaultValue  /**< Value if parameter not available. */
                                                     )
{
  if  (!params)
    return defaultValue;

  double result = defaultValue;

  PvGenParameter*  param = params->Get (paramName.Str ());
  if  (param)
  {
    PvGenType  pvGenType;
    if  (param->GetType (pvGenType).IsOK ())
    {
      if  (pvGenType == PvGenTypeFloat)
      {
        PvGenFloat*  pvGenFloat = dynamic_cast<PvGenFloat *> (param);
        double  pvFloat;
        if  (pvGenFloat->GetValue (pvFloat).IsOK ())
          result = pvFloat;
      }
    }
    else
    {
      PvString  pvString;
      if  (param->ToString (pvString).IsOK ())
      {
        KKStr  s = pvString.GetAscii ();
        result = s.ToDouble ();
      }
    }
  }

  return result;
}  /* GetPvParameterFloat */



kkint64  CameraAcquisitionPleora::GetPvParameterInteger (PvGenParameterArray*  params,
                                                         const KKStr&          paramName,
                                                         kkint64               defaultValue
                                                        )
{
  if  (!params)
    return defaultValue;

  kkint64  result = defaultValue;

  PvGenParameter*  param = params->Get (paramName.Str ());
  if  (param)
  {
    PvGenType  pvGenType;
    if  (param->GetType (pvGenType).IsOK ())
    {
      if  (pvGenType == PvGenTypeInteger)
      {
        PvGenInteger*  pvGenInteger = dynamic_cast<PvGenInteger *> (param);
        int64_t  pvInt;
        if  (pvGenInteger->GetValue (pvInt).IsOK ())
          result = pvInt;
      }
    }
    else
    {
      PvString  pvString;
      if  (param->ToString (pvString).IsOK ())
      {
        KKStr  s = pvString.GetAscii ();
        result = s.ToInt64 ();
      }
    }
  }

  return result;
}  /* GetPvParameterInteger */



kkint32  CameraAcquisitionPleora::GetPvParameterEnum (PvGenParameterArray*  params,
                                                    const KKStr&          paramName,
                                                    kkint32               defaultValue
                                                   )
{
  if  (!params)
    return defaultValue;

  kkint64  result = defaultValue;

  PvGenParameter*  param = params->Get (paramName.Str ());
  if  (param)
  {
    PvGenType  pvGenType;
    if  (param->GetType (pvGenType).IsOK ())
    {
      if  (pvGenType == PvGenTypeEnum)
      {
        PvGenEnum*  pvGenEnum = dynamic_cast<PvGenEnum *> (param);
        int64_t  pvInt;
        if  (pvGenEnum->GetValue (pvInt).IsOK ())
          result = pvInt;
      }
    }
    else
    {
      PvString  pvString;
      if  (param->ToString (pvString).IsOK ())
      {
        KKStr  s = pvString.GetAscii ();
        result = s.ToInt64 ();
      }
    }
  }

  return (kkint32)result;
}  /* GetPvParameterEnum */



KKStr  CameraAcquisitionPleora::GetPvParameterEnumStr (PvGenParameterArray*  params,
                                                       const KKStr&          paramName,
                                                       const KKStr&          defaultValue
                                                      )
{
  if  (!params)
    return defaultValue;

  KKStr  result = defaultValue;

  PvGenParameter*  param = params->Get (paramName.Str ());
  if  (param)
  {
    PvString  pvString;
    if  (param->ToString (pvString).IsOK ())
     result  = pvString.GetAscii ();
  }

  return  result;
}  /* GetPvParameterEnumStr */



KKStr  CameraAcquisitionPleora::GetPvParameterString (PvGenParameterArray*  params,
                                                      const KKStr&          paramName,
                                                      const KKStr&          defaultValue
                                                     )
{
  if  (!params)
    return defaultValue;

  KKStr  result = defaultValue;

  PvGenParameter*  param = params->Get (paramName.Str ());
  if  (param)
  {
    PvGenType  pvGenType;
    if  (param->GetType (pvGenType).IsOK ())
    {
      if  (pvGenType == PvGenTypeString)
      {
        PvGenString*  pvGenString = dynamic_cast<PvGenString *> (param);
        PvString  pvString;
        if  (pvGenString->GetValue (pvString).IsOK ())
          result = pvString.GetAscii ();
      }
    }
    else
    {
      PvString  pvString;
      if  (param->ToString (pvString).IsOK ())
        result = pvString.GetAscii ();
    }
  }

  return result;
}  /* GetPvParameterString */



void  CameraAcquisitionPleora::SetPvParameterEnum (const KKStr&  paramName,
                                                   const KKStr&  value,
                                                   bool&         successful,
                                                   KKStr&        errMsg
                                                  )
{
  successful = false;
  PvGenEnum*  lParam = dynamic_cast<PvGenEnum*> (lDeviceParams->Get (paramName.Str ()));
  if  (!lParam)
  {
    errMsg = "Parameter name not valid.";
    return;
  }

  PvResult   pvR;
  bool writable = false;
  pvR = lParam->IsWritable (writable);
  if  (!pvR.IsOK ())
  {
    errMsg = "Could not determine if parameter is writable.";
    return;
  }

  if  (!writable)
  {
    errMsg = "Parameter is not writable.";
    return;
  }

  PvString enumStr = value.Str ();
  pvR = lParam->SetValue (enumStr);
  if  (!pvR.IsOK ())
  {
    errMsg = PvResultToStr  (pvR);
    return;
  }

  errMsg = "";
  successful = true;
  return;
}  /* SetPvParameterEnum */



void  CameraAcquisitionPleora::SetPvParameterBool (const KKStr&  paramName,
                                                   bool          value,
                                                   bool&         successful,
                                                   KKStr&        errMsg
                                                  )
{
  successful = false;
  PvGenBoolean*  lParam = dynamic_cast<PvGenBoolean*> (lDeviceParams->Get (paramName.Str ()));
  if  (!lParam)
  {
    errMsg = "Parameter name not valid.";
    return;
  }

  PvResult   pvR;
  bool writable = false;
  pvR = lParam->IsWritable (writable);
  if  (!pvR.IsOK ())
  {
    errMsg = "Could not determine if parameter is writable.";
    return;
  }

  if  (!writable)
  {
    errMsg = "Parameter is not writable.";
    return;
  }

  pvR = lParam->SetValue (value);
  if  (!pvR.IsOK ())
  {
    errMsg = PvResultToStr (pvR);
    return;
  }

  errMsg = "";
  successful = true;
  return;
}  /* SetPvParameterBool */



KKStr  CameraAcquisitionPleora::Int64ToIpAddress (int64_t i)
{
  kkint32 p1 = i % 256;  i /= 256;
  kkint32 p2 = i % 256;  i /= 256;
  kkint32 p3 = i % 256;  i /= 256;
  kkint32 p4 = i % 256;  i /= 256;
  KKStr ipStr = StrFormatInt (p4, "000") + "."  +
                StrFormatInt (p3, "000") + "."  +
                StrFormatInt (p2, "000") + "."  +
                StrFormatInt (p1, "000");

  return  ipStr;
}  /* Int64ToIpAddress */



void  CameraAcquisitionPleora::AddToHeaderField (PvGenParameter*  p)
{
  if  (!p)  return;

  KKStr returnStr (512);
  
  PvString   aName;
  PvGenType  aType;
  
  if  (!p->GetName (aName).IsOK ())
    return;

  KKStr  fieldName = aName.GetAscii ();
  

  KKStr  value = "";

  p->GetType (aType);

  log.Level (40) << "CameraAcquisitionPleora::AddToHeaderField    FieldName[" << fieldName << "]"
                 << "  Type[" << PvGenTypeToCStr (aType) << "]." << endl;

  switch  (aType)
  {
  case  PvGenTypeInteger:
    {
      PvGenInteger *lGenInteger = dynamic_cast<PvGenInteger *>(p);
      int64_t i;
      if  (lGenInteger->GetValue (i).IsOK ())
        value = StrFromInt64 (i);
      else
        value = "***ERROR***";
    }
    break;

  case  PvGenTypeEnum:       
    {
      PvString enumValue;
      PvGenEnum*  pvEnum = dynamic_cast<PvGenEnum *>(p);
      if  (pvEnum->GetValue (enumValue).IsOK ())
        value = enumValue.GetAscii ();
      else
        value = "***ERROR***";
    }
    break;

  case  PvGenTypeBoolean:
    {
      PvGenBoolean  *pvBool = dynamic_cast<PvGenBoolean *>(p);
      bool b = false;
      if  (pvBool->GetValue (b).IsOK ())
        value = (b ? "True" : "False");
      else
        value = "***ERROR***";
    }
    break;

  case  PvGenTypeString:
    {
      PvString  pvString;
      PvGenString  *pStr = dynamic_cast<PvGenString *>(p);
      if  (pStr->GetValue (pvString).IsOK ())
        value = pvString.GetAscii ();
      else
        value = "***ERROR***";
    }
    break;

  case  PvGenTypeCommand:    
    value = "Command";    
    break;

  case  PvGenTypeFloat:  
    {
      double  d;
      PvGenFloat*  pvFloat =  dynamic_cast<PvGenFloat *>(p);
      if  (pvFloat->GetValue (d).IsOK ())
      {
        value = StrFormatDouble (d, "######0.00000");
      }
      else
        value = "***ERROR***";
    }
    break;

  case  PvGenTypeRegister:   
    value = "register";
    break;


  case  PvGenTypeUndefined:  
    value = "Undefined";
    break;

  default:
    value = "UnKnown";
  }

  Manager ()->AddHeaderField (fieldName, value);
  log.Level (10) << "AddToHeaderField\t" << fieldName << "\t" << value << endl;
}  /* AddToHeaderField */



void   CameraAcquisitionPleora::AddPleoraVariableToHeaderField (const KKStr&  varName)
{
  log.Level (40) << "CameraAcquisitionPleora::AddPleoraVariableToHeaderField    VarName[" << varName << "]." << endl;

  if  (lDevice == NULL)
  {
    log.Level (10) << "CameraAcquisitionPleora::AddPleoraVariableToHeaderField  VarName[" << varName << "]     ***  lDevice== NULL   ***." << endl;
    return;
  }

  if  (!lDeviceParams)
    lDeviceParams = lDevice->GetParameters ();

  PvString  pvVarName = varName.Str ();

  PvGenParameter*  lPvGenParam = NULL;
  lPvGenParam = lDeviceParams->Get (pvVarName);

  if  (!lPvGenParam)
  {
    log.Level (10) << "Invalid Pleora Variable Name :" << varName.QuotedStr () << endl;
  }
  else
  {
    AddToHeaderField (lPvGenParam);
  }
}



void   CameraAcquisitionPleora::CameraParametersPopulate (PvGenParameterArray*  params,
                                                          CameraParameters&     cameraParams,
                                                          RunLog&               runLog
                                                         )
{
  runLog.Level (40) << "CameraAcquisitionPleora::CameraParametersPopulate " << ((params == NULL) ? "***  params == NULL  ***" : "") << endl;

  if  (!params)
    return;

  kkint32  acquisitionLinePeriod = 0;
  float  scanRate = 0.0f;
  float  analogGain = 0.0f;

  analogGain = (float)GetPvParameterFloat (params, "GainAbs", -1.0);
  scanRate = (float)GetPvParameterFloat  (params, "AcquisitionLineRateAbs", 0.0);

  cameraParams.DeviceVendorName (GetPvParameterString           (params, "DeviceVendorName", ""));
  cameraParams.DeviceModelName  (GetPvParameterString           (params, "DeviceModelName",  ""));
  cameraParams.DeviceVersion    (GetPvParameterString           (params, "DeviceVersion",    ""));
  cameraParams.DigitalGain      ((kkint32)GetPvParameterInteger (params, "DigitalGainRaw",   -1));
  cameraParams.FrameHeight      ((kkint32)GetPvParameterInteger (params, "Height",            0));
  cameraParams.FrameWidth       ((kkint32)GetPvParameterInteger (params, "Width",             0));
  cameraParams.Model            (GetPvParameterString           (params, "DeviceModelName",  ""));
  cameraParams.SensitivityMode  (GetPvParameterEnumStr          (params, "SensitivityMode",  -1));

  kkint64  ipAddr = GetPvParameterInteger (params, "GevCurrentIPAddress",    0);
  cameraParams.IpAddress (Int64ToIpAddress (ipAddr));

  if  (cameraParams.DeviceModelName ().EqualIgnoreCase ("DiviinaLM1GE"))
  {
    acquisitionLinePeriod = (kkint32)GetPvParameterInteger (params, "AcquisitionLinePeriod", -1);
    analogGain            = (float)GetPvParameterInteger (params, "Gain",                  -1);
  }

  if  ((scanRate <= 0.0f)  &&  (acquisitionLinePeriod > 0))
    scanRate = 1000000.0f / (float)acquisitionLinePeriod;

  else if  ((scanRate > 0.0f)  &&  (acquisitionLinePeriod <= 0))
    acquisitionLinePeriod = (kkint32)(0.5f + 1000000.0f / scanRate);

  cameraParams.AnalogGain (analogGain);
  cameraParams.LinePeriod (acquisitionLinePeriod);
  cameraParams.ScanRate   (scanRate);


  //result->serialNumber     = GetPvParameterString (lDeviceParams, "???????????????");
  //result->macAddress       = GetPvParameterString (lDeviceParams, "DeviceModelName");
  return;
}  /* CameraParametersPopulate */



void  CameraAcquisitionPleora::SetGainTap (kkint64 gainTap)
{
  if  (lGainSelector)
  {
    PvResult  pvr = lGainSelector->SetValue (gainTap);
    if  (!pvr.IsOK ())
    {
      KKStr  msg = "CameraAcquisitionPleora::SetGainTap   ***ERROR***  Setting GainTap to " + StrFormatInt64 (gainTap, "###,##0") + " " + PvResultToStr (pvr);
      log.Level (-1) << msg << endl;
    }

    int64_t  tempGainTap = 0;
    if  (lGainSelector->GetValue (tempGainTap).IsOK ())
      gainTapSel = (kkint32)tempGainTap;
    else
      gainTapSel = -1;
    log.Level (40) << "SetGainTap  GainTap value :" << gainTapSel << endl;
  }
}



KKStr  CameraAcquisitionPleora::PvResultToStr (PvResult&  r)
{
  KKStr   description = r.GetDescription ().GetAscii ();
  kkuint32  code        = r.GetCode ();
  KKStr   codeStr     = r.GetCodeString ().GetAscii ();

  KKStr  s (512);
  s << "Code [" << code << "," << codeStr  << "]";
  if  (!description.Empty ())
    s << " Desc [" << description << "].";

  if  (r.IsFailure ())
    s  << " Failed";

  if  (r.IsPending ())
    s << " Pending";

  if  (r.IsSuccess ())
    s << " Success";

  return  s;
}  /* PvResultToStr */



void  CameraAcquisitionPleora::SetSensitivityMode (const KKStr& requestedSensitivityMode)
{
  if  (!lSensitivityMode)
  {
    log.Level (10) << "CameraAcquisitionPleora::SetSensitivityMode   ***ERROR***    \"lSensitivityMode\"  Not Defined." << endl;
    return;
  }

  PvResult stopResult = lStop->Execute ();
  if  (stopResult.IsOK ())
  {
    log.Level (40) << "SetSensitivityMode  Device Stopped :" << PvResultToStr (stopResult) << endl;
  }
  else
  {
    log.Level (-1) << "SetSensitivityMode  ***ERROR***    Failed to STOP Device :" << PvResultToStr (stopResult) << endl;
  }


  if  (lTLLocked)
    lTLLocked->SetValue (0);

  PvString  str (requestedSensitivityMode.Str ());
  PvResult result = lSensitivityMode->SetValue (str);
  if  (result.IsOK ())
  {
    log.Level (10) << "SetSensitivityMode   Sensitivity Mode set to :" << curSensitivityMode << endl;
  }
  else
  {
    KKStr msg (128);
    msg << "SetSensitivityMode   ***ERROR***  Set SensitivityMode Failure Mode :" << requestedSensitivityMode << "  Result code :" << PvResultToStr (result);
    log.Level (-1) << msg << endl;
  }

  {  
    int64_t  tempSensitivityMode = 0;
    if  (lSensitivityMode->GetValue (tempSensitivityMode).IsOK ())
    {
      switch  (tempSensitivityMode)
      {
      case 0:  curSensitivityMode = "Low";   break;
      case 1:  curSensitivityMode = "High";  break;
      case 2:  curSensitivityMode = "Tall";  break;
      default: curSensitivityMode = "UnKnown";
      }
    }

    log.Level (10) << "SetSensitivityMode  Sensitivity Mode set to :" << curSensitivityMode << endl;
  }

  // Need to get the Pipe Line Running again.
  if  (lTLLocked)
    lTLLocked->SetValue (1);
  PvResult  startResult =  lStart->Execute ();
  if  (startResult.IsOK ())
  {
    log.Level (40) << "SetSensitivityMode   Device start successful :" <<  PvResultToStr (startResult);
  }
  else
  {
    KKStr msg (128);
    msg << "SetSensitivityMode   ***ERROR***   Start command failed :" <<  PvResultToStr (startResult);
    log.Level (-1) << msg << endl;
  }
}   /* SetSensitivityMode */



void  CameraAcquisitionPleora::SetScanRate  (float requestedScanRate)
{
  KKStr  requestedScanRateStr = StrFormatDouble (requestedScanRate, "-ZZZ,ZZ0.0");
  if  (requestedScanRate < 1.0f)
  {
    log.Level (-1) << "CameraAcquisitionPleora::SetScanRate   ***ERROR***  Invalid RequestedScanRate :" << requestedScanRateStr << endl;
    return;
  }

  if  (cameraParams->DeviceModelName ().EqualIgnoreCase ("DiviinaLM1GE"))
  {
    int64_t  acquisitionLinePeriod = (int)(0.5f + 1000000.0f / requestedScanRate);

    if  (acquisitionLinePeriod < 55)
    {
      requestedScanRate = 1000000.0f / 55.0f;
      KKStr newRequestedScanRateStr = StrFormatDouble (requestedScanRate, "-ZZZ,ZZ0.0");
      KKStr warnMsg (128);
      warnMsg  << "CameraAcquisitionPleora::SetScanRate   RequestedScanRate: " << requestedScanRateStr << "  too fast; adjusted to: " << StrFormatDouble (requestedScanRate, "###,##0.00");
      log.Level (20) << warnMsg << endl;
      requestedScanRateStr = newRequestedScanRateStr;
      acquisitionLinePeriod = 55;
    }

    int64_t  exposureTime = acquisitionLinePeriod - 5;

    if  (lExposureTime)
    {
      PvResult  pvr = lExposureTime->SetValue (exposureTime);
      if  (!pvr.IsOK ())
      {
        KKStr  msg = "CameraAcquisitionPleora::SetScanRate   ***ERROR***  Could not set ExposureTime to: " +  PvResultToStr (pvr);
        log.Level (-1) << endl << msg << endl << endl;
      }
      else 
      {
        pvr = lAcquisitionLinePeriod->SetValue (acquisitionLinePeriod);
        if  (!pvr.IsOK ())
        {
          KKStr  msg = "CameraAcquisitionPleora::SetScanRate   ***ERROR***  Could not set AcquisitionLinePeriod to: " + PvResultToStr (pvr);
          log.Level (-1) << endl << msg << endl << endl;
        }
      }
    }
  }
  else
  {
    if  (lScanRate)
    {
      PvResult pvr = lScanRate->SetValue (requestedScanRate);
      if  (!pvr.IsOK ())
      {
        log.Level (-1) << "CameraAcquisitionPleora::SetScanRate   ***ERROR***  Error setting ScanRate :" << requestedScanRate << " :" << PvResultToStr (pvr) << endl;
      }

      double  scanRateD = 0.0;
      if  (lScanRate->GetValue (scanRateD).IsOK ())
      {
        curScanRate = (float)scanRateD;
        cameraParams->ScanRate (curScanRate);
        log.Level (10) << "SetScanRate   ScanRate set to : " << StrFormatDouble (scanRateD, "###,##0.0") << endl;
        Manager ()->ScanRateChanged (curScanRate);
        ComputeFramneIntervals ();
      }
    }
  }
}  /* SetScanRate */



void  CameraAcquisitionPleora::SetAnalogGain  (float requestedAnalogGain)
{
  log.Level (40) << "SetAnalogGain   :" << requestedAnalogGain << endl;
  if  (cameraParams->DeviceModelName ().EqualIgnoreCase ("DiviinaLM1GE"))
  {
    //analogGain = (float)GetPvParameterInteger (params, "Gain", -1);

    int64_t  adjGain = (int)(800.0f * ((requestedAnalogGain + 10.0f) / 20.0f));

    PvResult  pvr = lGain->SetValue (adjGain);
    if  (!pvr.IsOK ())
    {
      KKStr  msg (128);
      msg << "SetAnalogGain   ***ERROR***   Setting 'Gain' to : " << StrFormatInt ((int)adjGain, "##0") << "  Result Code :" << PvResultToStr (pvr);
      log.Level (-1) << msg << endl;
    }
    if  (lGain->GetValue (adjGain).IsOK ())
    {
      curAnalogGain = (20.0f * (float)adjGain / 800.0f) - 10.0f;
      cameraParams->AnalogGain ((float)curAnalogGain);
      log.Level (10) << "SetAnalogGain   Gain set to :" << StrFormatInt ((int)adjGain, "##,##0") << endl;
    }
  }
  else
  {
    if  (lAnalogGainAbs)
    {
      SetGainTap (0);

      PvResult  pvSetResult = lAnalogGainAbs->SetValue (requestedAnalogGain);
      if  (!pvSetResult.IsOK ())
        log.Level (-1) << "SetAnalogGain   ***ERROR***   Error setting 'GainAbs' :" << PvResultToStr (pvSetResult) << endl;

      double  analogGainD = 0.0;
      PvResult pvGetResult = lAnalogGainAbs->GetValue (analogGainD);
      if  (pvGetResult.IsOK ())
      {
        curAnalogGain = (float)analogGainD;
        cameraParams->AnalogGain ((float)analogGainD);
        log.Level (10) << "SetAnalogGain   Analog Gain set to :" << StrFormatDouble (analogGainD, "---,--0.0") << endl;
      }
    }
  }
}  /* SetAnalogGain */



void  CameraAcquisitionPleora::SetDigitalGain (kkint32 requestedDigitalGain)
{
  if  (lDigitalGainRaw)
  {
    SetGainTap (0);

    PvResult digitalGainRawSet = lDigitalGainRaw->SetValue (requestedDigitalGain);

    if  (!digitalGainRawSet.IsOK ())
    {
      KKStr  msg = "SetDigitalGain   ***ERROR***  Setting DigitalGainRaw :" + StrFormatInt (requestedDigitalGain, "###,##0") + "  :" + PvResultToStr (digitalGainRawSet);
      log.Level (-1) << msg << endl;
    }

    int64_t  digitalGainD = 0;
    if  (lDigitalGainRaw->GetValue (digitalGainD).IsOK ())
    {
      curDigitalGain = (kkint32)digitalGainD;
      cameraParams->DigitalGain ((kkint32)digitalGainD);
      log.Level (10) << "SetDigitalGain   Digital Gain : " << StrFormatInt64 (digitalGainD, "###,##0") << endl;
    }
  }
}  /* SetDigitalGain */



void  CameraAcquisitionPleora::ApplyCommandEntries ()
{
  log.Level (10) << "CameraAcquisitionPleora::ApplyCommandEntries" << endl;
  while  (QueueActive ())
    ApplyCommandNextEntry ();
}  /* ApplyCommandEntries */



void  CameraAcquisitionPleora::ApplyCommandNextEntry ()
{
  if  (!QueueActive ())
    return;

  CommandEntryPtr  cmd = GetNextCommandEntry ();
  if  (cmd)
  {
    switch  (cmd->CommandType ())
    {
    case  CommandEntry::EntryTypes::Null:
      break;

    case  CommandEntry::EntryTypes::AnalogGain:
      SetAnalogGain (cmd->AnalogGain ());
      break;

    case  CommandEntry::EntryTypes::DigitalGain:
      SetDigitalGain (cmd->DigitalGain ());
      break;

    case  CommandEntry::EntryTypes::ScanRate:
      SetScanRate (cmd->ScanRate ());
      break;

    case  CommandEntry::EntryTypes::SensitivityMode:
      SetSensitivityMode (cmd->SensitivityMode ());
      break;
    }

    delete  cmd;
    cmd = NULL;
  }
}  /* ApplyCommandNextEntry */



void  CameraAcquisitionPleora::SetCameraParametersForFlowMeter ()
{
  //KKKK  I want to move this code to a point where the variables are writable.
  bool  successful = false;
  KKStr  errMsg;

  SetPvParameterEnum ("PLC_Q1_Variable0", "One", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q1_Variable0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_Q1_Operator0", "Or", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q1_Operator0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_Q1_Variable1", "One", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q1_Variable1'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("OutputSelector", "Line1", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'OutputSelector'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("OutputFormat", "NoConnect", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'OutputFormat' to 'NoConnect'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("OutputFormat", "TTL", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'OutputFormat' to 'TTL'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("LineSelector", "Line0", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'LineSelector' to 'Line0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("LineFormat", "NoConnect", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'LineFormat' to 'NoConnect'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("LineFormat", "TTL", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'LineFormat' to 'TTL'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("CounterSelector", "Counter1", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'CounterSelector' to 'Counter1'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_I0", "Line0", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_I0' to 'Line0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_Q17_Variable0", "PLC_I0", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q17_Variable0' to 'PLC_I0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_Q17_Operator0", "Or", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q17_Variable0' to 'PLC_I0'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("PLC_Q17_Variable1", "Zero", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'PLC_Q17_Variable1' to 'Zero'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("GevTimestampCounterSelector", "Counter1", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'GevTimestampCounterSelector' to 'Counter1'." << endl
                   << errMsg << endl;
  }

  SetPvParameterBool ("GrbCh0AcqCfgIncludeMetadataInImage", true, successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'GrbCh0AcqCfgIncludeMetadataInImage' to 'True'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("GrbCh0MetadataSourceSelection", "GevTimestampValueLow", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'GrbCh0MetadataSourceSelection' to 'GevTimestampValueLow'." << endl
                   << errMsg << endl;
  }

  SetPvParameterEnum ("CounterEventSource", "PLC_Q17_RisingEdge", successful, errMsg);
  if  (!successful)
  {
    log.Level (-1) << "SetCameraParametersForFlowMeter    ***ERROR***  setting 'CounterEventSource' to 'PLC_Q17_RisingEdge'." << endl
                   << errMsg << endl;
  }
}  /* SetCameraParametersForFlowMeter */



void  CameraAcquisitionPleora::ConnectToCamera (bool&  connectionSuccessful)
{
  log.Level (10) << "ConnectToCamera" << endl;
  connectionSuccessful = false;
  StartStatus (StartStatusType::Connecting, "");

  Manager ()->AddSecondaryMsg ("Connecting to Camera.");

  if  (lPipeline  ||  lStream  ||  lDevice)
  {
    log.Level (10) << "ConnectToCamera  Previous connection not cleaned up need to call 'DisconnectFromCamera'." << endl;
    DisconnectFromCamera ();
    log.Level (40) << "ConnectToCamera    back from 'DisconnectFromCamera' can now proceed to connect to camera." << endl;
  }

  // Connect to the GEV Device
  lDevice = new PvDeviceGEV ();

  KKStr  status;
  status << "Connecting to Mac Address: " << MacAddress ();
  log.Level (10) << status << endl;

  PvString  pcMacAddress = MacAddress ().Str ();

  log.Level (40) << "ConnectToCamera   Establishing connection to :" << MacAddress () << endl;
  PvResult  pvConnectResult = lDevice->Connect (pcMacAddress);
  if   (!pvConnectResult.IsOK ())
  {
    status = "";
    status << "Unable to connect to Mac-Address: " << MacAddress ();
    log.Level (-1) << "ConnectToCamera   ***ERROR*** Trying to connect :" << PvResultToStr (pvConnectResult) << endl
                   << "   " << status << endl;
    StartStatus (StartStatusType::ConnectionFailed, status);

    PvString  pvIpAddress               = cameraParams->IpAddress  ().Str ();
    PvString  pvMacAddress              = cameraParams->MacAddress ().Str ();
    PvString  pvInterfaceSubnetMask     = cameraParams->InterfaceSubnetMask ().Str ();
    PvString  pvInterfaceDefaultGateway = cameraParams->InterfaceDefaultGateway ().Str ();

    log.Level (40) << "ConnectToCamera  Attempting to configure IP address of Mac :" << cameraParams->MacAddress () << "  to IP :" << cameraParams->IpAddress () << endl;

    PvResult pvr = PvDeviceGEV::SetIPConfiguration (pvMacAddress, 
                                                    pvIpAddress, 
                                                    pvInterfaceSubnetMask, 
                                                    pvInterfaceDefaultGateway
                                                   );
    if  (!pvr.IsOK ())
    {
      log.Level (-1) << "ConnectToCamera  ***ERROR***  Setting IP Address  " << PvResultToStr (pvr) << endl;
    }
    else
    {
      log.Level (40) << "ConnectToCamera  IP Address successfully configured  will now try connecting again." << endl;
      pvConnectResult = lDevice->Connect (pvMacAddress);
    }

    if  (pvConnectResult.IsOK ())
    {
      log.Level (30) << "ConnectToCamera   Connection successful :" << PvResultToStr (pvConnectResult) << endl;
    }
    else
    {
      log.Level (30) << "Failed to connect after setting IP :" << PvResultToStr (pvConnectResult) << endl;
      delete lDevice;
      lDevice = NULL;
      return;
    }
  }

  status = "";
  status << "Successfully connected to MacAddress: " << MacAddress ();
  log.Level (10) << status << endl;

  //PrintDeviceParameters ();

  // Get device parameters needed to control streaming
  InitializeDeviceParameterAccessVariables ();

  if  (embeddedFlowMeter)
    SetCameraParametersForFlowMeter ();

  CameraParametersPopulate (lDeviceParams, *cameraParams, log);
  Manager ()->FrameHeightWidthChanged (cameraParams->FrameHeight (), cameraParams->FrameWidth ());
  curScanRate    = cameraParams->ScanRate ();
  curAnalogGain  = cameraParams->AnalogGain ();
  curDigitalGain = cameraParams->DigitalGain ();

  if  ((requestedScanRate > 1)  &&  (requestedScanRate < 36000))
    AddScanRateCommand (requestedScanRate);

  if  ((requestedAnalogGain >= analogGainMin)  &&  (requestedAnalogGain <= analogGainMax))
    AddAnalogGainCommand (requestedAnalogGain);

  if  ((requestedDigitalGain >= 0)  &&  (requestedDigitalGain <= digitalGainMax))
    AddDigitalGainCommand (requestedDigitalGain);

  analogGainMin = -7.0f;
  digitalGainMax = 10000;
  if  (lAnalogGainAbs)
  {
    // Retrieve Min-Max analogGain values.
    analogGainMin = -7.0f;
    double  min = 0.0;
  
    PvResult  getMinResult = lAnalogGainAbs->GetMin (min);
    if  (getMinResult.IsOK ())
    {
      log.Level (50) << "ConnectToCamera  AnalogGainAbs::Min :" << min << endl;
      if  (min < -1000.0f)
        analogGainMin = -1000.0f;
      else
        analogGainMin =  (float)min;
    }
    else
    {
      log.Level (40) << "ConnectToCamera   ***ERROR***  AnalogGainAbs.GetMin() :" << PvResultToStr (getMinResult) << endl;
    }

    double  max = 0.0;
    PvResult getMaxResult = lAnalogGainAbs->GetMax (max);
    if  (getMaxResult.IsOK ())
    {
      log.Level (50) << "ConnectToCamera   AnalogGainAbs::Max :" << max << endl;
      if  (max > 1000.0f)
        analogGainMax = 1000.0f;
      else
        analogGainMax = (float)max;
    }
    else
    {
      log.Level (40) << "ConnectToCamera   *** ERROR ***  AnalogGainAbs.GetMin()[" << PvResultToStr (getMaxResult) << "]." << endl;
    }
  }

  // Negotiate streaming packet size
  log.Level (20) << "ConnectToCamera   Negotiating Streaming Packet Size." << endl;
  PvResult  negotiateResult = lDevice-> NegotiatePacketSize ();
  if  (!negotiateResult.IsOK ())
    log.Level (-1) << "ConnectToCamera  ***ERROR***   NegotiatePacketSize failed :" << PvResultToStr (negotiateResult) << endl;
  else
    log.Level (40) << "ConnectToCamera  NegotiatePacketSize Successful :" << PvResultToStr (negotiateResult) << endl;

  log.Level (20) << "ConnectToCamera   Opening stream to device." << endl;
  lStream = new PvStreamGEV ();
  PvResult  streamOpenResult = lStream->Open (IpAddress ().Str ());
  if  (!streamOpenResult.IsOK ())
  { 
    log.Level (-1) << "ConnectToCamera   ***ERROR***    Unable to open Camera Stream :" << PvResultToStr (streamOpenResult) << endl;
    delete  lStream;
    lStream = NULL;
    return;
  }
  else
  {
    log.Level (40) << "ConnectToCamera  Camera Stream Opened :" << PvResultToStr (streamOpenResult) << endl;
  }


  lPipeline = new PvPipeline (lStream);

  int64_t payloadSize = 0;
  
  PvResult getPayloadSizeResult = lPayloadSize->GetValue (payloadSize);
  if  (!getPayloadSizeResult.IsOK ())
    log.Level (-1) << "ConnectToCamera   ***ERROR***  Error retrieving PayLoadSize :" << PvResultToStr (getPayloadSizeResult) << endl;
  else
    log.Level (40) << "ConnectToCamera   PayLoadSize Retrieved :" << (kkint64)payloadSize << " :" << PvResultToStr (getPayloadSizeResult) << endl;


  lPipeline->SetBufferSize (static_cast<uint32_t> (payloadSize));

  PvResult  bufferCountResult = lPipeline->SetBufferCount (64); // Increase for high frame rate without missing block IDs
  if  (bufferCountResult.IsOK ())
    log.Level (40) << "ConnectToCamera   BufferCount successfully set to :" << 64 << " :" << PvResultToStr (bufferCountResult) << endl;
  else
    log.Level (-1) << "ConnectToCamera   ***ERROR***   Setting BufferCount :" << PvResultToStr (bufferCountResult) << endl;
 
  // Have to set the Device IP destination to the Stream

  PvString localIPAddress = lStream->GetLocalIPAddress ();
  uint16_t localPort      = lStream->GetLocalPort ();

  PvResult streamDestResult = lDevice->SetStreamDestination (localIPAddress, localPort);
  if  (streamDestResult.IsOK ())
    log.Level (40) << "ConnectToCamera   Device StreamDestination set to :" << localIPAddress.GetAscii () << " : " << localPort << endl;
  else
    log.Level (-1) << "ConnectToCamera   ***ERROR***   Setting Device-Stream-Destination :" << PvResultToStr (streamDestResult) << endl;

  // TLParamsLocked is optional but when present, it MUST be set to 1 before sending the AcquisitionStart command
  if  (lTLLocked != NULL)
  {
    log.Level (40) << "ConnectToCamera   Setting TLParamsLocked to 1" << endl;
    PvResult tlParamsLockedResult = lTLLocked->SetValue (1);
    if  (tlParamsLockedResult.IsOK ())
      log.Level (40) << "ConnectToCamera  'tlParamsLocked' successfully set to '1' :" << PvResultToStr (tlParamsLockedResult) << endl;
    else
      log.Level (-1) << "ConnectToCamera  ***ERROR***  Failure setting 'tlParamsLocked' to 1 :" << PvResultToStr (tlParamsLockedResult) << endl;
  }

  // IMPORTANT: the pipeline needs to be "armed", or started before we instruct the device to send us images
  log.Level (20) << "ConnectToCamera   Starting pipeline" << endl;
  PvResult pipeLineStartResult = lPipeline->Start ();
  if  (pipeLineStartResult.IsOK ())
    log.Level (40) << "ConnectToCamera   Pipeline Started Successfully." << endl;
  else
    log.Level (-1) << "ConnectToCamera   ***ERROR*** Starting Pipeline :" << PvResultToStr (pipeLineStartResult) << endl;
 
  // Get stream parameters/stats
  InitializeStreamParameterAccessVariables ();

  log.Level (40) << "ConnectToCamera   Retrieving  'GevTimestampControlReset' parameter." << endl;
  PvGenCommand *lResetTimestamp = dynamic_cast<PvGenCommand *>(lDeviceParams->Get ("GevTimestampControlReset"));
  if  (!lResetTimestamp)
  {
    log.Level (40) << "ConnectToCamera   Parameter 'GevTimestampControlReset'  not retrieved." << endl;
  }
  else
  {
    log.Level (40) << "ConnectToCamera  Executing 'GevTimestampControlReset'." << endl;
    PvResult timestampControlResetResult = lResetTimestamp->Execute ();
    if  (timestampControlResetResult.IsOK ())
      log.Level (40) << "ConnectToCamera  'GevTimestampControlReset' successfully executed :" << PvResultToStr (timestampControlResetResult) << endl;
    else
      log.Level (-1) << "ConnectToCamera  ***ERROR***  'GevTimestampControlReset' execute failed :" << PvResultToStr (timestampControlResetResult) << endl;
  }


  {
    // Setting "MaximumResendGroupSize" to 30;  this helps to reduce if not eliminate 
    // "2819,TOO_MANY_CONSECUTIVE_RESENDS"  errors when retrieving camera frame buffers.
    PvGenInteger*  maxResendGroupSize = dynamic_cast<PvGenInteger *> (lStreamParams->Get ("MaximumResendGroupSize"));
    if  (maxResendGroupSize)
    {
      PvResult maxResendGroupSizeResult;

      int64_t  curValue = 0;
      maxResendGroupSizeResult = maxResendGroupSize->GetValue (curValue);
      if  (!maxResendGroupSizeResult.IsOK ())
      {
        log.Level (-1) << "ConnectToCamera   ***ERROR***   Unable to get current 'MaximumResendGroupSize' value." << endl
                       << "                  " << PvResultToStr (maxResendGroupSizeResult) << endl;
        curValue = 0;
      }
       
      if  (curValue >= 30)
      {
        log.Level (10) << "ConnectToCamera   MaximumResendGroupSize: " << (kkint64)curValue << "  Not going to override." << endl;
      }
      else
      {
        maxResendGroupSizeResult = maxResendGroupSize->SetValue (30);
        if  (!maxResendGroupSizeResult.IsOK ())
        {
          log.Level (-1) << "ConnectToCamera   ***ERROR***   Unable to set 'MaximumResendGroupSize' to 30" << endl
                         << "                  " << PvResultToStr (maxResendGroupSizeResult) << endl;
        }
        else
        {
          log.Level (10) << "ConnectToCamera   MaximumResendGroupSize: Set to 30." << endl;
        }
      }
    }
  }

  if  (Manager ()->OperatingMode () == CounterManager::CounterOperatingModes::Advanced)
    Manager ()->AddSecondaryMsg ("Setting Camera Parameters.");
  ApplyCommandEntries ();

  // The pipeline is already "armed", we just have to tell the device to start sending us images
  log.Level (10) << "ConnectToCamera   Sending Start command to device." << endl;
  PvResult deviveStartResult = lStart->Execute();
  if  (deviveStartResult.IsOK ())
    log.Level (40) << "ConnectToCamera   Start command successful :" << PvResultToStr (deviveStartResult) << endl;
  else
    log.Level (-1) << "ConnectToCamera   ***ERROR***   Start Command failed :" << PvResultToStr (deviveStartResult) << endl;

  if  (embeddedFlowMeter)
  {
    AddPleoraVariableToHeaderField ("PLC_Q1_Variable0");
    AddPleoraVariableToHeaderField ("PLC_Q1_Operator0");
    AddPleoraVariableToHeaderField ("PLC_Q1_Variable1");
    AddPleoraVariableToHeaderField ("LineSelector");
    AddPleoraVariableToHeaderField ("LineFormat");
    AddPleoraVariableToHeaderField ("OutputSelector");
    AddPleoraVariableToHeaderField ("OutputFormat");
    AddPleoraVariableToHeaderField ("CounterSelector");
    AddPleoraVariableToHeaderField ("PLC_I0");
    AddPleoraVariableToHeaderField ("PLC_Q17_Variable0");
    AddPleoraVariableToHeaderField ("PLC_Q17_Operator0");
    AddPleoraVariableToHeaderField ("PLC_Q17_Variable1");
    AddPleoraVariableToHeaderField ("GevTimestampCounterSelector");
    AddPleoraVariableToHeaderField ("GrbCh0AcqCfgIncludeMetadataInImage");
    AddPleoraVariableToHeaderField ("GrbCh0MetadataInsertionMode");
  }

  CameraParametersAddToHeaderFields ();

  connectionSuccessful = true;

  Manager ()->AddSecondaryMsg ("Camera Connection Established!");

  log.Level (40) << "CameraAcquisitionPleora::ConnectToCamera    Exiting." << endl;

  return;
}  /* ConnectToCamera */



void  CameraAcquisitionPleora::DisconnectFromCamera ()
{
  log.Level (10) << endl << "DisconnectFromCamera" << endl;
  Manager ()->AddSecondaryMsg ("Disconnecting from Camera.");
  if  (lStop)
  {
    log.Level (20) << "DisconnectFromCamera   Executing Stop Command." << endl;
    PvResult stopResult = lStop->Execute();
    log.Level (40) << "DisconnectFromCamera   Stop Command Results :" << this->PvResultToStr (stopResult) << endl;
  }

  // If present reset TLParamsLocked to 0. Must be done AFTER the 
  // streaming has been stopped
  if  (lTLLocked != NULL)
  {
    log.Level (20) << "DisconnectFromCamera   Resetting TLParamsLocked to 0." << endl;
    PvResult lockResult = lTLLocked->SetValue (0);
    log.Level (40) << "DisconnectFromCamera   Resetting Results :" << PvResultToStr (lockResult) << endl;
  }
  
  if  (lPipeline)
  {
    // We stop the pipeline - letting the object lapse out of 
    // scope would have had the destructor do the same, but we do it anyway

    log.Level (20) << "DisconnectFromCamera   Stopping pipeline." <<endl;
    PvResult pipeLineResult = lPipeline->Stop();
    log.Level (40) << "DisconnectFromCamera   Stopping Results :" <<  PvResultToStr (pipeLineResult) << endl;
    delete  lPipeline;
    lPipeline = NULL;
  }

  if  (lStream)
  {
    // Now close the stream. Also optional but nice to have
    log.Level (20) << "DisconnectFromCamera   Closing Stream." << endl;
    PvResult streamResult = lStream->Close();
    log.Level (40) << "DisconnectFromCamera   Closing Stream Result :" << PvResultToStr (streamResult) << endl;
    delete  lStream;
    lStream = NULL;
  }

  if  (lDevice)
  {
    // Finally disconnect the device. Optional, still nice to have
    log.Level (20) << "DisconnectFromCamera    Disconnecting From Device." << endl;
    PvResult deviceResult = lDevice->Disconnect();
    log.Level (40) << "DisconnectFromCamera    Disconnecting From Device :" << PvResultToStr (deviceResult) << endl;
    delete  lDevice;
    lDevice = NULL;
  }

  lStreamParams              = NULL;
  lFrameRate                 = NULL;
  lCameraTemparature         = NULL;
  lTLLocked                  = NULL;
  lPayloadSize               = NULL;
  lStart                     = NULL;
  lReadVoltageAndTemperature = NULL;
  lStop                      = NULL;
  lScanRate                  = NULL;
  lGainSelector              = NULL;
  lAnalogGainAbs             = NULL;
  lDigitalGainRaw            = NULL;
  lGain                      = NULL;
  lExposureTime              = NULL;
  lAcquisitionLinePeriod     = NULL;

  Manager ()->AddSecondaryMsg ("Disconnected From Camera.");

  log.Level (40) << "CameraAcquisitionPleora::DisconnectFromCamera    Exiting." << endl;

  return;
}  /* DisconnectFromCamera */



void  CameraAcquisitionPleora::ComputeFramneIntervals ()
{
  if  (curScanRate <= 1.0f)
    return;

  // Compute the number of frames that occur in a 5 second period.
  oneSecFrameInterval   = (kkint32)(0.5f + curScanRate *  1.0f / (float)FrameHeight ());
  twoSecFrameInterval   = oneSecFrameInterval *  2;
  threeSecFrameInterval = oneSecFrameInterval *  3;
  fiveSecFrameInterval  = oneSecFrameInterval *  5;
  tenSecFrameInterval   = oneSecFrameInterval * 10;
  voltTempReadInterval  = oneSecFrameInterval * 10;
}



void  CameraAcquisitionPleora::Run ()
{
  Status (ThreadStatus::Running);

  log.Level (10) << "CameraAcquisitionPleora::Run" << endl;

  KKStr  status (128);

  StartStatus (StartStatusType::Connecting, "Connecting to Camera.");

  bool  connectionSuccessful = false;
  ConnectToCamera (connectionSuccessful);
  if  (!connectionSuccessful)
  {
    StartStatus (StartStatusType::ConnectionFailed, "Connection FAILED!!!.");
    Manager ()->AddSecondaryMsg ("");
    status = "";
    status << "Unable to connect to Mac-Address: " << MacAddress ();
    log.Level (10) << status << endl;
    DisconnectFromCamera ();
    Status (ThreadStatus::Stopping);
    Crashed (true);
    return;
  }

  kkint32  loopCount = 0;
  kkint32  whosTurn = 0;  /**< Will be used to determine who's turn to talk to camera;  FrameRate == 0;  CameraTemperature == 1 */

  lastBlockId = 0;

  totalLostPackets = 0;

  StartStatus (StartStatusType::Connected, "Camera Connected.");

  ComputeFramneIntervals ();

  ofstream* flowRate = NULL;
  kkint32  flowRateScanLineCount = 0;
  kkint32  lastFlowRateCount = -1;
  if  (generateFlowRateReport)
  {
    DateTime dt = KKB::osGetLocalDateTime ();
    KKStr  flowMeterName = "C:\\Temp\\FlowMeter_" + dt.Date ().YYYYMMDD () + "-" + dt.Time ().HHMMSS () + ".txt";
    flowRate = new ofstream (flowMeterName.Str ());
    *flowRate << "Frame"  << "\t" << "Scan"  << "\t"  << ""    << "\t"  << ""    << "\t"  << ""    << "\t"  << ""    << "\t"  << ""    << "\t"  << "Total" << endl;
    *flowRate << "number" << "\t" << "Line"  << "\t"  << "P0"  << "\t"  << "P1"  << "\t"  << "P2"  << "\t"  << "P3"  << "\t"  << "P4"  << "\t"  << "Count" << endl;
  }

  // Acquire images until the user instructs us to stop
  log.Level (10) << "CameraAcquisitionPleora::Run   Image Acquisition Loop Started." << endl;
  while  (!TerminateFlag ()  &&  !ShutdownFlag ())
  {
    if  (!lPipeline)
    {
      // We are not communicating to the camera lets try reconnecting.
      StartStatus (StartStatusType::Disconnected, "Camera Disconnected !!!  Attempting to reconnect.");

      log.Level (-1) << endl << "CameraAcquisitionPleora::Run    Will Sleep for 2 Seconds." << endl << endl;

      osSleepMiliSecs (1000);
      ConnectToCamera (connectionSuccessful);
      if  (connectionSuccessful)
      {
        StartStatus (StartStatusType::Connected, "Camera Connected.");
        if  (Manager ()->OperatingMode () == CounterManager::CounterOperatingModes::Advanced)
          Manager ()->AddSecondaryMsg ("Re-connected to Camera.");

        log.Level (-1) << endl << "CameraAcquisitionPleora::Run    *** Connection Re-Established  ***" << endl << endl;

        lastBlockId = 0;
      }
      else
      {
        log.Level (-1) << endl << "CameraAcquisitionPleora::Run    ***ERROR***    Camera is not talking to us." << endl << endl;
        DisconnectFromCamera ();
      }
    }
    if  (lPipeline)
    {
      if  (QueueActive ())
        ApplyCommandEntries ();

      // Retrieve next buffer		
      PvBuffer *lBuffer = NULL;
      PvResult  lOperationResult;

      PvResult lResult = lPipeline->RetrieveNextBuffer (&lBuffer, 2000, &lOperationResult);
      if  (!lResult.IsOK ())
      {
        KKStr  kkMsg (100);
        kkMsg << "lPipeline->RetrieveNextBuffer  ***ERROR***   Retrieving NextBuffer: " <<  PvResultToStr (lResult);
        log.Level (-1) << kkMsg << endl;
        StartStatus (StartStatusType::Disconnected, kkMsg);
        if  (Manager ()->OperatingMode () == CounterManager::CounterOperatingModes::Advanced)
          Manager ()->AddSecondaryMsg ("Error Retrieving Frame from Camera.");
        DisconnectFromCamera ();
      }
      else
      {
        kkuint32  pvOutputQueueSize = lPipeline->GetOutputQueueSize ();

        if  (!lOperationResult.IsOK ())
        {
          KKStr  resultStr = PvResultToStr (lOperationResult);
          log.Level (-1) << "lPipeline->RetrieveNextBuffer   ***WARNING***  :" << resultStr << endl
                         << "    pvOutputQueueSize: " << pvOutputQueueSize << endl;
          if  (Manager ()->OperatingMode () == CounterManager::CounterOperatingModes::Advanced)
            Manager ()->AddSecondaryMsg ("Camera Timed Out.");
        }
        else
        {
          const uint8_t*  rawData = lBuffer->GetDataPointer ();

          kkint32 lostPacketCount = lBuffer->GetLostPacketCount ();
          PvImage* i = lBuffer->GetImage ();
          kkuint32  imageSize = i->GetImageSize ();

          kkuint64  blockId = lBuffer->GetBlockID ();
          kkint64   numBlocksSkipped = blockId - (lastBlockId + 1);
          if  (numBlocksSkipped < 0)
            numBlocksSkipped += 65535;

          if  (numBlocksSkipped > 0)
          {
            log.Level (-1) << "CameraAcquisitionPleora::Run   ***ERROR***  'blockId's Skipped   LastBlockId: " << lastBlockId << "   BlockId: " << blockId << "   Approx ScanLine: " << scanLinesRead << endl;
            physicalSeqNumsSkipped += (kkint32)numBlocksSkipped;
          }

          lastBlockId = blockId;

          totalLostPackets += lostPacketCount;

          loopCount++;
          if  ((loopCount % fiveSecFrameInterval) == 0)
          {
            switch  (whosTurn)
            {
            case 0:
              {
                if  (lFrameRate)
                {
                  double  frameRateD = 0.0;
                  lFrameRate->GetValue (frameRateD);
                  frameRate = (float)frameRateD;
                }
              }
              break;

            case 1:
              {
                if  (lCameraTemparature  &&  lReadVoltageAndTemperature)
                {
                  PvResult tempReadResult = lReadVoltageAndTemperature->Execute ();
                  if  (tempReadResult.IsOK ())
                  {
                    int64_t  pvInt = -1;
                    lCameraTemparature->GetValue (pvInt);
                    cameraTemparature = (kkint32)pvInt;
                    log.Level (50) << "CameraAcquisitionPleora::Run   lCameraTemparature :" << cameraTemparature << endl;
                  }
                  else
                  {
                    log.Level (-1) << "CameraAcquisitionPleora::Run   ***ERROR***  executing 'ReadVoltageAndTemperature :" << PvResultToStr (tempReadResult) << endl;
                  }
                }
              }
              break;
            }

            ++whosTurn;
            if  (whosTurn > 1)
              whosTurn = 0;
          }

          if  (embeddedFlowMeter)
          {
            // Flow Counter as beginning of frame.
            kkuint32  tempFlowMeterCounter = rawData[0] + rawData[1] * 256 + (rawData[2] * 256 * 256) + (rawData[3] * 256 * 256 * 256);
            if  (tempFlowMeterCounter != flowMeterCounter)
            {
              // At some point in the future we might want to call "Call-Back" function from here to let other objects 
              // know that the flow meter changed value.
              flowMeterCounter = tempFlowMeterCounter;
              flowMeterCounterScanLine = scanLinesRead;
              FlowMeterCounterUpdate (scanLinesRead, flowMeterCounter);
            }

            if  (generateFlowRateReport)
            {
              // This code was added to help get diagnostic info on the 1ast 4 columns of imagery which
              // are being used to communicate FlowRate meter pulse count.
              const uint8_t* sld = rawData;
              for (int rrr = 0;  rrr < FrameHeight ();  ++rrr)
              {
                kkint32 flowRateCount = sld[0] + sld[1] * 256 + (sld[2] * 256 * 256) + (sld[3] * 256 * 256 * 256);
                if  (flowRateCount != lastFlowRateCount)
                {
                  *flowRate << physicalFramesRead    << "\t" 
                            << flowRateScanLineCount << "\t"
                            << (int)(sld[0])         << "\t" 
                            << (int)(sld[1])         << "\t" 
                            << (int)(sld[2])         << "\t" 
                            << (int)(sld[3])         << "\t" 
                            << flowRateCount
                            << endl;
                  lastFlowRateCount = flowRateCount;
                }

                sld += FrameWidth ();
                ++flowRateScanLineCount;
              }
            }
          }

          ++physicalFramesRead;
          scanLinesRead += FrameHeight ();

          float flowRate = 0.0f;
          float flowRateRatio = -1.0f;
          flowMeter->GetFlowRateTrend (flowRate, flowRateRatio);
          frameBuffer->AddFrame ((uchar*)rawData, imageSize, FrameHeight (), FrameWidth (), flowMeterCounter, flowMeterCounterScanLine, flowRateRatio, flowRate);

          PvResult releaseBufferResult = lPipeline->ReleaseBuffer (lBuffer);
          if  (!releaseBufferResult.IsOK ())
            log.Level (-1) << "CameraAcquisitionPleora::Run   ***ERROR***  ReleaseBuffer failed :" << PvResultToStr (releaseBufferResult) << endl;
        }
      }
    }
  }

  DisconnectFromCamera ();

  delete  flowRate;
  flowRate = NULL;

  Manager ()->AddSecondaryMsg ("Cameras Acquisition Stopped.");

  StartStatus (StartStatusType::Disconnected, "Camera Stopped.");
  Status (ThreadStatus::Stopping);

  log.Level (10) << "CameraAcquisitionPleora::Run   Exiting   TerminateFlag: " << TerminateFlag () <<  "  ShutdownFlag: " << ShutdownFlag () << endl;

  return;
}  /* Run */



CameraParametersListPtr  CameraAcquisitionPleora::GetCameraList (MsgQueuePtr  _msgQueue)
{
  const PvDeviceInfoGEV*  pvDeviceInfoGEV = NULL;

  RunLog  runLog (_msgQueue);
  runLog.SetLevel (CounterVariables::DebugLevel ());
  runLog.Level (40) << "CameraAcquisitionPleora::GetCameraList" << endl;

  PvSystem  pvSystem;
  PvResult  pvResult = pvSystem.Find ();
  if  (!pvResult.IsOK ())
  {
    runLog.Level (10) << "CameraAcquisitionPleora::GetCameraList   ***ERROR***   Pleora drivers not installed: " << PvResultToStr (pvResult) << endl;
    return  NULL;
  }

  CameraParametersListPtr  cameras = new CameraParametersList (true);

  uint32_t  interfaceCount = pvSystem.GetInterfaceCount ();

  for (uint32_t interfaceIdx = 0; interfaceIdx < interfaceCount;  ++interfaceIdx)
  {
    const PvInterface*   pvInterface = pvSystem.GetInterface (interfaceIdx);
    if  (pvInterface == NULL)
      continue;

    PvInterfaceType type = pvInterface->GetType ();
    if  (type != PvInterfaceTypeNetworkAdapter)
      continue;

    const PvNetworkAdapter* pvNetAdapter = dynamic_cast<const PvNetworkAdapter *>(pvInterface);

    KKStr  interfaceDescription =  pvNetAdapter->GetDescription ().GetAscii ();
    KKStr  interfaceId          =  pvNetAdapter->GetUniqueID    ().GetAscii ();
    KKStr  interfaceIpAddress   =  pvNetAdapter->GetIPAddress   ().GetAscii ();
    KKStr  interfaceMacAddress  =  pvNetAdapter->GetMACAddress  ().GetAscii ();
    KKStr  interfaceSubnetMask  =  pvNetAdapter->GetSubnetMask  ().GetAscii ();

    uint32_t  deviceCount = pvInterface->GetDeviceCount ();

    runLog.Level (20) << "GetCameraList  InterfaceDescription :" << interfaceDescription << endl;
    runLog.Level (20) << "GetCameraList  DeviceCount :" << deviceCount << endl;

    for  (uint32_t  deviceIdx = 0;  deviceIdx < deviceCount; ++deviceIdx)
    {
      const PvDeviceInfo*  pvDeviceInfo = pvNetAdapter->GetDeviceInfo (deviceIdx);
      pvDeviceInfoGEV = dynamic_cast<const PvDeviceInfoGEV*>(pvDeviceInfo);
      if  (pvDeviceInfoGEV == NULL)
        continue;

      KKStr  deviceId         = pvDeviceInfoGEV->GetUniqueID         ().GetAscii ();
      KKStr  macAddress       = pvDeviceInfoGEV->GetMACAddress       ().GetAscii ();
      KKStr  serialNum        = pvDeviceInfoGEV->GetSerialNumber     ().GetAscii ();
      KKStr  model            = pvDeviceInfoGEV->GetModelName        ().GetAscii ();
      KKStr  manufacturerInfo = pvDeviceInfoGEV->GetManufacturerInfo ().GetAscii ();
      KKStr  deviceIpAddress  = pvDeviceInfoGEV->GetIPAddress        ().GetAscii ();

      runLog.Level (20) << "GetCameraList   DeviceID :" << deviceId << "  macAddress :" << macAddress << endl;

      {
        kkint32  numTries = 0;
        CameraParametersPtr  cp = NULL;
        while  ((numTries < 3)  &&  (cp == NULL))
        {
          delete  cp;  cp = NULL;
          cp = GetCameraParameters (macAddress, _msgQueue, runLog);
          if  ((cp == NULL)                    ||
               (cp->FrameHeight () <= 0)    ||
               (cp->FrameWidth  () <= 0)    ||
               (cp->ScanRate    () <= 0)
              )
          {
            runLog.Level (-1) << "GetCameraList   ***ERROR***   Failed to retrieve camera parameters." << endl;
            // For some reason we did not retrieve all the parameters;  we need to try again.
            delete  cp;
            cp = NULL;
          }
          ++numTries;
        }

        if  (cp)
          cameras->PushOnBack (cp);
      }
    }
  }

  return cameras;
}  /* GetCameraList */



CameraParametersPtr  CameraAcquisitionPleora::GetDeviceInfo (const KKStr& _keyValue,
                                                             char         _keyType, /* 'S' = Serial Number, 'M' = MAC Address */
                                                             RunLog&      _runLog
                                                            )
{
  _runLog.Level (30) << "CameraAcquisitionPleora::GetDeviceInfo  KeyValue[" << _keyValue << "]." << endl;

  PvSystem  pvSystem;
  PvResult  pvResult = pvSystem.Find ();
  if  (!pvResult.IsOK ())
  {
    _runLog.Level (-1) << "CameraAcquisitionPleora::GetDeviceInfo   ***ERROR***   Pleora drivers not installed :" << PvResultToStr (pvResult) << endl;
    return  NULL;
  }

  _runLog.Level (40) << "CameraAcquisitionPleora::GetDeviceInfo   'pvSystem.Find'  successful." << endl;

  CameraParametersPtr  cp = NULL;

  uint32_t  interfaceCount = pvSystem.GetInterfaceCount ();

  for  (uint32_t interfaceIdx = 0;  (interfaceIdx < interfaceCount)  &&  (cp == NULL);  ++interfaceIdx)
  {
    const PvInterface*   pvInterface = pvSystem.GetInterface (interfaceIdx);
    if  (pvInterface == NULL)
      continue;

    const PvNetworkAdapter* pvNetAdapter = dynamic_cast<const PvNetworkAdapter*>(pvInterface);
    if  (!pvNetAdapter)
      continue;

    PvString  interfaceDescription = pvNetAdapter->GetDescription ();

    _runLog.Level (40) << "GetDeviceInfo   Checking Interface[" << interfaceDescription.GetAscii () << "]" << endl;

    PvString  interfaceId             = pvNetAdapter->GetUniqueID ();
    KKStr     interfaceIpAddress      = pvNetAdapter->GetIPAddress ().GetAscii ();
    KKStr     interfaceMacAddress     = pvNetAdapter->GetMACAddress ().GetAscii ();
    PvString  interfaceSubnetMask     = pvNetAdapter->GetSubnetMask ();
    PvString  interfaceDefaultGateway = pvNetAdapter->GetDefaultGateway ();

    uint32_t  deviceCount = pvInterface->GetDeviceCount ();

    _runLog.Level (40) << "GetDeviceInfo   InterfaceDescription :" << interfaceDescription.GetAscii () << endl;
    _runLog.Level (40) << "GetDeviceInfo   Device Count :" << deviceCount << endl;

    uint32_t  deviceIdx = 0;
    while  ((deviceIdx < deviceCount)  &&  (cp == NULL))
    {
      const PvDeviceInfo* pvDeviceInfo = pvInterface->GetDeviceInfo (deviceIdx);

      pvDeviceInfo = pvInterface->GetDeviceInfo (deviceIdx);
      if  (pvDeviceInfo == NULL)
        continue;

      const PvDeviceInfoGEV*  pvDeviceInfoGEV = dynamic_cast<const PvDeviceInfoGEV*> (pvDeviceInfo);
      if  (!pvDeviceInfoGEV)
        continue;

      PvString  deviceId         = pvDeviceInfoGEV->GetUniqueID ();
      KKStr     deviceIpAddress  = pvDeviceInfoGEV->GetIPAddress ().GetAscii ();
      KKStr     deviceMacAddress = pvDeviceInfoGEV->GetMACAddress ().GetAscii ();

      _runLog.Level (40) << "GetDeviceInfo   deviceMacAddress :" << deviceMacAddress << "  deviceIpAddress :" << deviceIpAddress << endl;
      _runLog.Level (40) << "GetDeviceInfo   DeviceID: " << + deviceId.GetAscii () << endl;

      KKStr  keyValue = "";
      if  (_keyType == 'M')
        keyValue = deviceMacAddress;
      else
        keyValue = pvDeviceInfoGEV->GetSerialNumber ().GetAscii ();

      if  (_keyValue.EqualIgnoreCase (keyValue))
      {
        _runLog.Level (40) << "GetDeviceInfo   Device Found  deviceId :" << deviceId.GetAscii () << endl;

        PvDeviceGEV* pvDevice = new PvDeviceGEV ();
        PvString  pvMacAddress = deviceMacAddress.Str ();
        PvResult  pvDeviveConnectResult = pvDevice->Connect (pvMacAddress);

        if  (!pvDeviveConnectResult.IsOK ())
        {
          _runLog.Level (40) << "GetDeviceInfo   Connect Failed :" << PvResultToStr (pvDeviveConnectResult)  << endl;

          auto  lastDot = interfaceIpAddress.LocateLastOccurrence ('.');
          if  (lastDot  &&  (lastDot.value () > 0))
          {
            KKStr  newIpAddress = interfaceIpAddress.SubStrPart (0, lastDot.value ()) + "122";
            PvString  pvNewIpAddress = newIpAddress.Str ();

            _runLog.Level (20) << "GetDeviceInfo   Attempting to set IP Address :" << newIpAddress << endl;

            PvResult  pvr = PvDeviceGEV::SetIPConfiguration (pvMacAddress, pvNewIpAddress, interfaceSubnetMask, interfaceDefaultGateway);
            if  (!pvr.IsOK ())
            {
              _runLog.Level (10) << "GetDeviceInfo   Error Setting IP Address :" << PvResultToStr (pvr) << endl;
            }
            else
            {
              _runLog.Level (40) << "GetDeviceInfo   IP-Address successfully set." << endl;
              pvDeviveConnectResult = pvDevice->Connect (pvMacAddress);
              _runLog.Level (10) << "GetDeviceInfo   Connection attempt after IP Address set :" << PvResultToStr (pvDeviveConnectResult) << endl;
            }
          }
        }
        if  (pvDeviveConnectResult.IsOK ())
        {
          delete  cp;
          cp = NULL;

          if  (false)
          {
            PvGenParameterArray *lLinkParams = pvDevice->GetCommunicationParameters ();
            PrintDeviceParameters ("CommunicationParameters", lLinkParams);
            lLinkParams   = NULL;
          }

          PvGenParameterArray *lDeviceParams = pvDevice->GetParameters();
          if  (false)
          {
            PrintDeviceParameters ("ParmsGen", lDeviceParams);
          }

          cp = new CameraParameters ();
          cp->IpAddress (deviceIpAddress);
          cp->MacAddress (deviceMacAddress);
          cp->InterfaceDefaultGateway (interfaceDefaultGateway.GetAscii ());
          cp->InterfaceSubnetMask     (interfaceSubnetMask.GetAscii ());
          CameraParametersPopulate (lDeviceParams, *cp, _runLog);
          lDeviceParams = NULL;

          _runLog.Level (40) << "GetDeviceInfo   Camera Parameters retrieved." << endl;

          pvDevice->Disconnect ();
          delete  pvDevice;
          pvDevice = NULL;

          if  ((cp->FrameHeight () <= 0)  ||  (cp->FrameWidth () <= 0)  ||  (cp->ScanRate () <= 0))
          {
            delete  cp;
            cp = NULL;
          }
        }

        delete  pvDevice;
        pvDevice = NULL;
      }
      ++deviceIdx;
    }
  }

  return cp;
}  /* GetDeviceInfo */



CameraParametersPtr  CameraAcquisitionPleora::GetCameraParameters (const KKStr&  _macAddress,
                                                                   MsgQueuePtr   _msgQueue,
                                                                   RunLog&       _runLog
                                                                  )
{
  _runLog.Level (40) << "CameraAcquisitionPleora::GetCameraParameters    MacAddress[" << _macAddress << "]." << endl;
  CameraParametersPtr  result = NULL;
  PvString macAddress (_macAddress.Str ());

  PvDeviceGEVPtr  lDevice = new PvDeviceGEV ();

  PvResult  pvResult = lDevice->Connect (macAddress);
  if  (!pvResult.IsOK ())
  {
    _runLog.Level (10) << "CameraAcquisitionPleora::GetCameraParameters  ***ERROR***  Unable to connect to [" << _macAddress << "]." << endl;
    _runLog.Level (10) << "    Error " << PvResultToStr (pvResult) << endl;
  }

  if  (pvResult.IsOK ())
  {
    _runLog.Level (40) << "GetCameraParameters  Connected to MacAddress[" << _macAddress << "]." << endl;

    if  (false)
    {
      PvGenParameterArray *lLinkParams = lDevice->GetCommunicationParameters ();
      PrintDeviceParameters ("CommunicationParameters", lLinkParams);
    }

    PvGenParameterArray *lDeviceParams = lDevice->GetParameters();
    //PrintDeviceParameters ("ParmsGen", lDeviceParams);

    result = new CameraParameters ();
    result->MacAddress (_macAddress);

    if  (!lDeviceParams)
    {
      _runLog.Level (10) << "CameraAcquisitionPleora::GetCameraParameters   ***ERROR***  Unable to 'GetGenParameters' for camera [" << _macAddress << "]" << endl;
    }
    else
    {
      CameraParametersPopulate (lDeviceParams, *result, _runLog);
      _runLog.Level (50) << "GetCameraParameters  Camera Parameters Retrieved." << endl;
      lDeviceParams = NULL;
    }
  }

  delete  lDevice;
  lDevice = NULL;

  return  result;
}  /* GetCameraParameters */



CameraAcquisitionPleoraPtr  CameraAcquisitionPleora::CreateFromSerialNumber 
           (CounterManagerPtr _manager,
            const KKStr             _serialNumber,
            CameraFrameBufferPtr    _frameBuffer,
            FlowMeterTrackerPtr     _flowMeter,
            MsgQueuePtr             _msgQueue,
            const KKStr&            _threadName
           )
{
  RunLog  runLog (_msgQueue);
  runLog.SetLevel (CounterVariables::DebugLevel ());

  runLog.Level (20) << "CameraAcquisitionPleora::CreateFromSerialNumber   serialNumber :" << _serialNumber << endl;

  CameraParametersPtr cameraParams = GetDeviceInfo (_serialNumber, 'S', runLog);
  if  (!cameraParams)
    return NULL;

  KKStr macAddress = cameraParams->MacAddress ();
  delete  cameraParams;
  cameraParams = NULL;

  if  (macAddress.Empty ())
    return NULL;

  return new  CameraAcquisitionPleora (_manager, 
                                       macAddress,
                                       _frameBuffer,
                                       _flowMeter,
                                       _msgQueue,
                                       _threadName
                                     );
}  /* CreateFromSerialNumber */



CameraAcquisitionPleoraPtr  CameraAcquisitionPleora::CreateFromMacAddress 
           (CounterManagerPtr _manager,
            const KKStr&            _macAddress,
            CameraFrameBufferPtr    _frameBuffer,
            FlowMeterTrackerPtr     _flowMeter,
            MsgQueuePtr             _msgQueue,
            const KKStr&            _threadName
           )
{
  RunLog  runLog (_msgQueue);
  runLog.SetLevel (CounterVariables::DebugLevel ());

  runLog.Level (20) << "CameraAcquisitionPleora::CreateFromMacAddress   MacAddress :" << _macAddress << endl;
  return new  CameraAcquisitionPleora (_manager,
                                       _macAddress,
                                       _frameBuffer,
                                       _flowMeter,
                                       _msgQueue,
                                       _threadName
                                     );
}  /* CreateFromMacAddress */



CameraAcquisitionPleoraPtr  CameraAcquisitionPleora::CreateFromCameraParameters
               (CounterManagerPtr _manager,
                CameraParametersPtr     _cameraParameters,
                CameraFrameBufferPtr    _frameBuffer,
                FlowMeterTrackerPtr     _flowMeter,
                MsgQueuePtr             _msgQueue,
                const KKStr&            _threadName
               )
{
  RunLog  runLog (_msgQueue);
  runLog.SetLevel (CounterVariables::DebugLevel ());

  runLog.Level (20) << "CameraAcquisitionPleora::CreateFromCameraParameters   MacAddress :" << _cameraParameters->MacAddress () << endl;

  CameraAcquisitionPleoraPtr  acquisition =  new CameraAcquisitionPleora (_manager, _cameraParameters, _frameBuffer, _flowMeter, _msgQueue, _threadName);
  return  acquisition;
}



KKStr  CameraAcquisitionPleora::PromptForCameraMacAddress ()
{
  // Create a GEV Device finder dialog
  PvDeviceFinderWnd  lDeviceFinderWnd;

  // Prompt the user to select a GEV Device
  lDeviceFinderWnd.ShowModal();

  // Get the connectivity information for the selected GEV Device
  const PvDeviceInfo* deviceInfo = lDeviceFinderWnd.GetSelected();

  // If no device is selected, abort
  if  (deviceInfo == NULL)
    return "";

  PvDeviceInfoType   type = deviceInfo->GetType ();
  if  (type != PvDeviceInfoTypeGEV)
  {
    // We curently only handle GEV devices.
    return "";
  }

  const PvDeviceInfoGEV* deviceInfoGEV = dynamic_cast<const PvDeviceInfoGEV*>(deviceInfo);
  KKStr  macAddress = deviceInfoGEV->GetMACAddress ().GetAscii ();
  return  macAddress;
}  /* PromptForCameraMacAddress */



KKStr  CameraAcquisitionPleora::GetPvGenParameterDesc (PvGenParameter*  p,
                                                       bool             tabDelStr
                                                      )
{
  if  (!p)  return "";

  KKStr returnStr (512);
  
  PvString   aName;
  PvString   aDescription;
  PvGenType  aType;

  PvResult  getNameResult = p->GetName (aName);
  cout << "getNameResult  Name : " << aName.GetAscii () << "  Result: " << PvResultToStr (getNameResult) << endl;

  PvResult  getDescResult = p->GetDescription (aDescription);
  cout << "getDescResult  : " << PvResultToStr (getDescResult) << endl;

  PvResult  getTypeResult = p->GetType (aType);
  cout << "getTypeResult  : " << PvResultToStr (getTypeResult) << endl;

  KKStr  category = "";
  PvString  pvCategoryStr;
  PvResult  getCatResult = p->GetCategory (pvCategoryStr);
  cout << "getCatResult  : " << PvResultToStr (getCatResult) << endl;

  if  (getCatResult.IsOK ())
     category = pvCategoryStr.GetAscii ();

  KKStr  toolTipStr = "";
  PvString pvToolTipStr;
  PvResult pvr = p->GetToolTip (pvToolTipStr);
  if  (pvr.IsOK ())
    toolTipStr = pvToolTipStr.GetAscii ();

  bool  isAvailable = p->IsAvailable ();
  KKStr  s;

  switch  (aType)
  {
  case  PvGenTypeInteger:
    {
      PvGenInteger *pvGenInt = dynamic_cast<PvGenInteger *>(p);
      int64_t i;
      if  (pvGenInt->GetValue (i).IsOK ())
        s = (kkint32)i;
      else
        s = "***Error***";
    }
  break;

  case  PvGenTypeEnum:       
    {
      int64_t  enumValue = 0;
      PvGenEnum*  pvEnum = dynamic_cast<PvGenEnum *>(p);
      if  (pvEnum->GetValue (enumValue).IsOK ())
        s = (kkint32)enumValue;
      else
        s = "***ERROR***";

      int64_t  numEntries = 0;
      if  (pvEnum->GetEntriesCount (numEntries).IsOK ())
      {
        KKStr  entryNames = "";
        for  (int xx = 0;  xx < numEntries;  ++xx)
        {
          if  (xx > 0)
            entryNames << ", ";
          const PvGenEnumEntry*  enumEntry;
          if  (pvEnum->GetEntryByIndex (xx, &enumEntry))
          {
            PvString  enumEntryName;
            if  (enumEntry->GetName (enumEntryName))
              entryNames << xx << "-" << enumEntryName.GetAscii ();
          }
        }

        s << "(" << entryNames << ")";
      }
    }
    break;

  case  PvGenTypeBoolean:
    {
      PvGenBoolean  *pvBool = dynamic_cast<PvGenBoolean *>(p);
      bool b = false;
      if  (pvBool->GetValue (b).IsOK ())
        s = (b ? "True" : "False");
      else
        s = "***ERROR***";
    }
    break;

  case  PvGenTypeString:
    {
      s = "";
      PvString  pvString;
      PvGenString  *pStr = dynamic_cast<PvGenString *>(p);
      if  (pStr->GetValue (pvString).IsOK ())
        s = pvString.GetAscii ();
      else
        s = "***Error***";
    }
    break;

  case  PvGenTypeCommand:    
    s = "Command";    
    break;

  case  PvGenTypeFloat:  
    {
      double  d;
      PvGenFloat*  pvFloat =  dynamic_cast<PvGenFloat *>(p);
      if  (pvFloat->GetValue (d).IsOK ())
      {
        s = StrFormatDouble (d, "######0.00000");
      }
      else
        s = "***ERROR***";
    }
    break;

  case  PvGenTypeRegister:   
    s = "register";
    break;

  case  PvGenTypeUndefined:  
    s = "Undefined";
    break;

  default:
    s = "UnKnown";
  }

  bool  readable = false;
  bool  writable = false;
  p->IsReadable (readable);
  p->IsWritable (writable);

  if  (tabDelStr)
  {
    returnStr  
       << PvGenTypeToCStr (aType)             << "\t"
       << aName.GetAscii ()                   << "\t" 
       << category                            << "\t"
       << (readable ? "Readable" : "")        << "\t"
       << (writable ? "Writable" : "")        << "\t"
       << (isAvailable ? "Available" : "Not") << "\t"
       << s                                   << "\t"
       << aDescription.GetAscii ()            << "\t"
       << toolTipStr
       << endl;
  }
  else
  {
    returnStr << "GenType     [" << PvGenTypeToCStr (aType)          << "]" << endl
              << "Name        [" << aName.GetAscii ()                << "]" << endl
              << "Category    [" << category                         << "]" << endl
              << "Readable    [" << (readable    ? "Readable"  : "") << "]" << endl
              << "Writable    [" << (writable    ? "Writable"  : "") << "]" << endl
              << "Available   [" << (isAvailable ? "Available" : "") << "]" << endl
              << "Value       [" << s                                << "]" << endl
              << "Description [" << aDescription.GetAscii ()         << "]" << endl
              << "ToolTip     [" << toolTipStr                       << "]" << endl;
  }
  return  returnStr;
}  /* GetPvGenParameterDesc */



void  CameraAcquisitionPleora::PrintDeviceParameters (const KKStr&          subName,
                                                      PvGenParameterArray*  lDeviceParams
                                                     )
{
  if  (!lDeviceParams)
    return;

  KKStr  fileName = osAddSlash (CounterVariables::TempDir ()) + "CameraParametrs_" + subName + ".txt";

  ofstream  outFile (fileName.Str ());

  PvGenInteger*  lTLLocked  = dynamic_cast<PvGenInteger *> (lDeviceParams->Get ("TLParamsLocked"));
  if  (lTLLocked)
    lTLLocked->SetValue (1);

  kkint32 numberOfParameters = lDeviceParams->GetCount ();
  for  (kkint32 zed = 0;  zed < numberOfParameters;  ++zed)
  {
    KKStr  s = "";

    PvGenParameter*  p = lDeviceParams->Get (zed);
    if  (!p)
      continue;

    KKStr  msg = GetPvGenParameterDesc (p, false);

    outFile  << msg << endl;
    osSleepMiliSecs (10);
  }

  if  (lTLLocked)
    lTLLocked->SetValue (0);

  outFile.close ();
}  /* PrintDeviceParameters */



void  CameraAcquisitionPleora::ResetCounts ()
{
  CameraAcquisition::ResetCounts ();
}



/******************************************************************************/
/*                             Auto gain Procedure                            */
/******************************************************************************/


kkuint16 CameraAcquisitionPleora::CameraHighValue ()
{
  VectorUcharPtr  scanLine = Manager ()->LastFrameHighValuesScanLine ();
  if  (!scanLine)
    return  false;

  kkuint16  highPoint = 0;

  // We start from the 4th column because the first 4 columns may be carrying the "Flow-Meter" counter value.
  kkuint16  x = 4;
  for  (x = 4;  x < scanLine->size ();  ++x)
  {
    uchar v = (*scanLine)[x];
    if  (v > highPoint)
      highPoint = v;
  }

  delete  scanLine;
  scanLine = NULL;

  return  highPoint;
}  /* CameraHighValue */



bool  CameraAcquisitionPleora::CameraSomePixelsSaturated (float percentThreshold)
{
  kkuint16  numWithHighPointTH = (kkuint16)(0.5f + (float)(FrameWidth ()) * percentThreshold);

  VectorUcharPtr  scanLine = Manager ()->LastFrameHighValuesScanLine ();
  if  (!scanLine)
    return  false;

  kkuint16  highPoint = 0;
  kkuint16  x = 0;
  kkuint16 numWithHighPoint = 0;

  // KAK 2013-10-08
  // I added a offset to beginning and end of scan-line after there was an issue with Auto Gain that was
  // caused by the Flow-Meter count being put into the first 4 pixels in the column. This would cause the 
  // Auto Gain to fail thinking all settings were saturated. Decided to offset by 30 firm both ends.
  kkuint16  firstCol = 30;
  kkuint16  lastCol = (kkuint16)scanLine->size () - 30;

  for  (x = firstCol;  x < lastCol;  ++x)
  {
    uchar v = (*scanLine)[x];
    if  (v > highPoint)
    {
      highPoint = v;
      numWithHighPoint = 1;
    }
    else if  (v == highPoint)
    {
      ++numWithHighPoint;
    }
  }

  delete  scanLine;
  scanLine = NULL;

  return  numWithHighPoint >= numWithHighPointTH;
}  /* CameraSomePixelsSaturated */



kkuint16 CameraAcquisitionPleora::CameraHighValueOf3Frames ()
{
  WaitForOneFrame ();
  kkuint16  hv1 = CameraHighValue ();

  WaitForOneFrame ();
  kkuint16  hv2 = CameraHighValue ();

  WaitForOneFrame ();
  kkuint16  hv3 = CameraHighValue ();

  return  (Max (hv1, hv2, hv3));
}  /* CameraHighValueOf3Frames */



bool  CameraAcquisitionPleora::DigitalGainIsMaxed ()
{
  bool  digitalGainIsMaxed = false;

  VectorUcharPtr  scanLine = Manager ()->LastFrameAverageScanLine ();
  if  (!scanLine)
    return  false;

  kkuint16  numPixelsGreaterThan253 = 0;

  // We start from the 4th column because the first 4 columns may be carrying the "Flow-Meter" counter value.
  for  (kkuint32 x = 4;  x < scanLine->size ();  ++x)
  {
    uchar v = (*scanLine)[x];
    if  (v > 253)
      ++numPixelsGreaterThan253;
  }

  kkuint16  threshold = (kkuint16)(0.5f + scanLine->size () / 40.0f);
  if  (numPixelsGreaterThan253 > threshold)
    digitalGainIsMaxed = true;
  else
    digitalGainIsMaxed = false;

  delete  scanLine;
  scanLine = NULL;

  return  digitalGainIsMaxed;
}  /* DigitalGainIsMaxed */



void  CameraAcquisitionPleora::WaitForOneFrame ()
{
  kkint64  lastNextFrameSeqNum = frameBuffer->NextFrameSeqNum ();
  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {
    kkint64  diff = frameBuffer->NextFrameSeqNum () - lastNextFrameSeqNum;
    if  (diff < 0)  diff = 0 - diff;
    if  (diff > 1) 
      break;
    osSleepMiliSecs (20);
  }
}  /* WaitForOneFrame */



void  CameraAcquisitionPleora::MakeAnalogGainRequest (float  _gain)
{
  log.Level (50) << "CameraAcquisitionPleora::MakeAnalogGainRequest  Gain [" << _gain << "]." << endl;

  Manager ()->RequestedAnalogGain (_gain);

  int loopCount = 0;

  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {
    if  ((loopCount % 20) == 19)
      Manager ()->RequestedAnalogGain (_gain);

    osSleepMiliSecs (20);
    float  diff = fabs (Manager ()->CurAnalogGain () - _gain);
    if  (diff < 0.01f)
      break;

    if  (RequestedAnalogGain () != _gain)
       Manager ()->RequestedAnalogGain (_gain);

    ++loopCount;
  }
}  /* MakeAnalogGainRequest */



void  CameraAcquisitionPleora::MakeDigitalGainRequest (kkint32  _gain)
{
  log.Level (50) << "CameraAcquisitionPleora::MakeDigitalGainRequest  Gain [" << _gain << "]." << endl;

  Manager ()->RequestedDigitalGain (_gain);
  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {
    osSleepMiliSecs (20);
    kkint32  diff = Manager ()->CurDigitalGain () - _gain;
    if  (diff < 0)
      diff = 0 - diff;
    if  (diff < 1)
      break;

    if  (RequestedDigitalGain () != _gain)
       Manager ()->RequestedDigitalGain (_gain);
  }
}  /* MakeDigitalGainRequest */



void  CameraAcquisitionPleora::RaiseAnalogGainUntilSaturated2 (float  increments)
{
  // AnalogGain saturation occurs when the high values no longer increase as you increase Analog Gain.
  // Because of noise we define Not Growing and not increasing by more than 2%.
  log.Level (40) << "CameraAcquisitionPleora::RaiseAnalogGainUntilSaturated2  increments[" << increments << "]." << endl;

  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {
    float  nextAnalogGain = agAnalogGain + increments;
    if  (nextAnalogGain > analogGainMax)
    {
      log.Level (30) << "RaiseAnalogGainUntilSaturated2   Reached end of Analog-Gain range." << endl;
      break;
    }

    MakeAnalogGainRequest (nextAnalogGain);
    agAnalogGain = nextAnalogGain;
    WaitForOneFrame ();

    // When increments are at 2.0f we want a saturation threshold count of 10% of total number of pixels in a scan line.
    float  th = Max (0.3f, 0.1f * (increments / 2.0f));

    if  (CameraSomePixelsSaturated (th))
    {
      agAnalogGain = agAnalogGain - increments;
      agAnalogGain = max (analogGainMin, agAnalogGain);
      MakeAnalogGainRequest (agAnalogGain);
      WaitForOneFrame ();
      log.Level (30) << "RaiseAnalogGainUntilSaturated2   Camera values have not gained  New analogGain[" << KKB::StrFormatDouble (agAnalogGain, "##0.00") << "]." << endl;
      break;
    }
  }
}  /* RaiseAnalogGainUntilSaturated2 */



void    CameraAcquisitionPleora::SetAnalogGainToFirstHighInRange (float  analogGainStart, 
                                                                  float  analogGainEnd, 
                                                                  float  increment
                                                                 )
{
  log.Level (40) << "SetAnalogGainToFirstHighInRange  AnalogGainStart: " << analogGainStart << "  AnalogGainEnd: " << analogGainEnd << "  Increment: " << increment << endl;

  VectorUint16 highValues;

  kkuint16  allTimeHigh = 0;

  float  nextAnalogGain = analogGainStart;
  while  (nextAnalogGain <= analogGainEnd)
  {
    agAnalogGain = nextAnalogGain;
    MakeAnalogGainRequest (agAnalogGain);
    WaitForOneFrame ();
    //osSleepMiliSecs (1000);
    kkuint16  hv = CameraHighValueOf3Frames ();
    allTimeHigh = Max (hv, allTimeHigh);
    highValues.push_back (hv);
    nextAnalogGain = nextAnalogGain + increment;
  }

  agAnalogGain = analogGainStart;
  for  (kkuint16 x = 0;  x < highValues.size ();  ++x)
  {
    if  (highValues[x] == allTimeHigh)
      break;
    agAnalogGain += increment;
  }

  MakeAnalogGainRequest (agAnalogGain);
}  /* SetAnalogGainToFirstHighInRange */



void  CameraAcquisitionPleora::RaiseDigitalGainUntilSaturated (kkint32  increments)
{
  log.Level (40) << "CameraAcquisitionPleora::RaiseDigitalGainUntilSaturated [" << increments << "]." << endl;
  int loopCount = 0;

  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {
    kkint32  nextDigitalGain = agDigitalGain + increments;
    if  (nextDigitalGain > digitalGainMax)
      break;

    MakeDigitalGainRequest (nextDigitalGain);
    agDigitalGain = nextDigitalGain;
    WaitForOneFrame ();
    if  (DigitalGainIsMaxed ())
    {
      agDigitalGain = Max ((kkint32)0, (agDigitalGain - increments));
      MakeDigitalGainRequest (nextDigitalGain);
      break;
    }
    ++loopCount;
    if  (loopCount > 10)
      break;
  }
}  /* RaiseDigitalGainUntilSaturated */



void  CameraAcquisitionPleora::RaiseDigitalGainUntilAtLeastHighValue (int  minHighValue)
{
  log.Level (40) << "CameraAcquisitionPleora::RaiseDigitalGainUntilAtLeastHighValue   MinHighValue[" << minHighValue << "]." << endl;
  WaitForOneFrame ();
  kkuint16  highValue = CameraHighValue ();
  if  (highValue >= minHighValue)
    return;

  kkint32  bestGuess = agDigitalGain * minHighValue / highValue;
  kkint32  increment = (bestGuess - agDigitalGain) / 3;

  while  ((!TerminateFlag ())  &&  (!ShutdownFlag ()) &&  (highValue < minHighValue))
  {
    if  (agDigitalGain >= digitalGainMax)
      break;
    agDigitalGain += increment;
    agDigitalGain = Min (agDigitalGain, digitalGainMax);
    MakeDigitalGainRequest (agDigitalGain);
    WaitForOneFrame ();
    highValue = CameraHighValue ();
  }

}  /* RaiseDigitalGainUntilAtLeastHighValue */



/*
 *@details
 *  This code was originally implemented for a Dalsa SG-11-02K80.
 *@code
 * Background:
 *   Camera: Dalsa SG-11-02K80
 *   Analog Gain (-10.0f thru 10.0f)
 *   Digital Gain (0 thru 5000)
 *
 *   Preference is to maximize Analog gain over Digital gain.
 *
 *  General Flow.
 *  1) Initialize Analog-Gain to known minimum value and Digital-Gain to 2000.
 *  2) Raise Digital Gain until High-Value in Scan line is at least 80.
 *  3) Raise Analog-Gain by increments of 2.0f until saturated.
 *  4) Raise Digital-Gain until high value in scan line is at least 200.
 *  5) Raise Analog-Gain by increments of 0.2f until saturated.
 *  6) Search for Analog-Gain that is in range of cur value -0.5 to + 1.0 selecting the 1st one
 *     to have "High-Value".
 *
 *    At this point we have the Analog-Gain value we want to live with; we now only need to 
 *    search for the Digital-Gain.
 *  
 *  7) Search for Digital-Gain start with Coarse(increments of a 1000) to fine using increment 
 *     of 100, and then 20.
 *@endcode
 */
void  CameraAcquisitionPleora::PerformAutoGainProcedureOld ()
{
  log.Level (10) << "CameraAcquisitionPleora::PerformAutoGainProcedure" << endl;

  bool sampleBeforeFlatField = Manager ()->SampleLastFrameBeforeFlatField ();
  Manager ()->SampleLastFrameBeforeFlatField (true);

  agAnalogGain  = analogGainMin;
  agDigitalGain = 2000;

  MakeAnalogGainRequest (agAnalogGain);
  MakeDigitalGainRequest (agDigitalGain);

  RaiseDigitalGainUntilAtLeastHighValue (80);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  RaiseAnalogGainUntilSaturated2 (2.0f);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  // Now that we should have a reasonable value for Analog-Gain we will start the Digital Gain at low value again
  agDigitalGain = 1000;
  MakeDigitalGainRequest (agDigitalGain);
  WaitForOneFrame ();

  RaiseDigitalGainUntilAtLeastHighValue (200);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  agDigitalGain = (kkint32)(0.6f * (float)agDigitalGain);
  MakeDigitalGainRequest (agDigitalGain);
  WaitForOneFrame ();

  RaiseAnalogGainUntilSaturated2 (0.5f);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  float  analogGainStart = Max (analogGainMin, (agAnalogGain - 0.5f));
  float  analogGainEnd   = Min (analogGainMax, (agAnalogGain + 0.5f));
  SetAnalogGainToFirstHighInRange (analogGainStart, analogGainEnd, 0.1f);

  RaiseDigitalGainUntilSaturated (1000);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  RaiseDigitalGainUntilSaturated (100);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  RaiseDigitalGainUntilSaturated (20);
  if  ((TerminateFlag ())  ||  (ShutdownFlag ()))
    return;

  Manager ()->SampleLastFrameBeforeFlatField (sampleBeforeFlatField);
}  /* PerformAutoGainProcedure */



void  CameraAcquisitionPleora::AnalogGainBinarySearch (kkuint16 threshholdSaturationValue)
{
  log.Level (20) << "CameraAcquisitionPleora::AnalogGainBinarySearch    " << endl;

  float  low  = Max (-10000.0f, analogGainMin);
  float  high = Min (10000.0f,  analogGainMax);

  float  deltaLowHigh = high - low;

  while  (deltaLowHigh > 0.1f)
  {
    float   mid = (low + high) / 2.0f;
    agAnalogGain = mid;
    MakeAnalogGainRequest (agAnalogGain);
    WaitForOneFrame ();
    kkuint16  cameraHighValue = CameraHighValue ();
    if  (cameraHighValue < threshholdSaturationValue)
      low = mid;
    else
      high = mid;
    deltaLowHigh = high - low;
  }

  agAnalogGain = low;
  MakeAnalogGainRequest (agAnalogGain);
  WaitForOneFrame ();

  log.Level (20) << "CameraAcquisitionPleora::AnalogGainBinarySearch   Done   AnalogGain[" << agAnalogGain << "]" << endl;
}   /* AnalogGainBinarySearch */



void  CameraAcquisitionPleora::DigitalGainBinarySearch ()
{
  log.Level (20) << "CameraAcquisitionPleora::DigitalGainBinarySearch" << endl;

  kkint32  low  = 1;
  kkint32  high = digitalGainMax;
  kkint32  deltaLowHigh = high - low;

  while  (deltaLowHigh > 10)
  {
    kkint32 mid = (low + high) / 2;
    agDigitalGain = mid;
    MakeDigitalGainRequest (agDigitalGain);
    WaitForOneFrame ();

    if  (DigitalGainIsMaxed ())
      high = mid;
    else
      low = mid;

    deltaLowHigh = high - low;
  }

  agDigitalGain = low;
  MakeDigitalGainRequest (agDigitalGain);
  WaitForOneFrame ();

  log.Level (20) << "CameraAcquisitionPleora::DigitalGainBinarySearch   Done   DigitalGain[" << agDigitalGain << "]" << endl;

}   /* DigitalGainBinarySearch */



/**
 *@details   Perform this with Analog-Gain maxed out so that area being imaged is obvious.
 */
void  CameraAcquisitionPleora::PerformAutoCrop ()
{
  log.Level (20) << "CameraAcquisitionPleora::PerformAutoCrop" << endl;
  VectorUcharPtr  scanLine = Manager ()->LastFrameHighValuesScanLine ();
  if  (!scanLine)
  {
    log.Level (-1) << "CameraAcquisitionPleora::PerformAutoCrop    ***ERROR***    No  'LastFrameHighValuesScanLine'  returned." << endl;
    return;
  }

  kkuint16  highPoint = 0;

  // We start from the 4th column because the first 4 columns may be carrying the "Flow-Meter" counter value.
  kkuint16  x = 4;
  for  (x = 4;  x < scanLine->size ();  ++x)
  {
    uchar v = (*scanLine)[x];
    if  (v > highPoint)
      highPoint = v;
  }

  kkuint16  threshhold = highPoint * 1 / 2;
  
  kkint32  midPoint = (kkint32)scanLine->size () / 2;

  kkint32  cropLeft = midPoint;
  while  ((cropLeft > 6)  && 
          (((*scanLine)[cropLeft] > threshhold)  ||  ((*scanLine)[cropLeft - 1] > threshhold))
         )
    --cropLeft;
  cropLeft += 2;

  kkint32  rightSide = (kkint32)scanLine->size ();
  kkint32  cropRight = midPoint;
  while  ((cropRight <  rightSide)  && 
          (((*scanLine)[cropRight] > threshhold)  ||  ((*scanLine)[cropRight - 1] > threshhold))
         )
    ++cropRight;
  cropRight -= 1;

  Manager ()->CropSettingsChanged (cropLeft, cropRight);

  delete  scanLine;
  scanLine = NULL;

  log.Level (50) << "CameraAcquisitionPleora::PerformAutoCrop    Exiting." << endl;

  return;
} /* PerformAutoCrop */



void  CameraAcquisitionPleora::PerformAutoGainProcedure ()
{
  log.Level (10) << "CameraAcquisitionPleora::PerformAutoGainProcedure" << endl;

  bool sampleBeforeFlatField = Manager ()->SampleLastFrameBeforeFlatField ();
  Manager ()->SampleLastFrameBeforeFlatField (true);

  // 1) Set Analog Gain to Max Value for a Fixed Digital-Gain value 
  //    a. The current high Image value will be used as the Threshold-Saturation-Value; we will use this value to search for Analog value to use.
  //    b. Determine Crop Settings.
  //
  // 2) Perform Binary Search for Analog-Gain value that is just below "
  
  agAnalogGain  = Min (10000.0f, analogGainMax);
  agDigitalGain = 2000;

  MakeAnalogGainRequest (agAnalogGain);
  MakeDigitalGainRequest (agDigitalGain);
  WaitForOneFrame ();
  WaitForOneFrame ();
  kkuint16  threshholdSaturationValue = CameraHighValue ();
  if  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
  {

    AnalogGainBinarySearch (threshholdSaturationValue);
    if  ((!TerminateFlag ())  &&  (!ShutdownFlag ()))
    {
      PerformAutoCrop ();
      DigitalGainBinarySearch ();
    }
  }

  Manager ()->SampleLastFrameBeforeFlatField (sampleBeforeFlatField);
}  /* PerformAutoGainProcedure */
