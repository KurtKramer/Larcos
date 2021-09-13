#include "StdAfx.h"
#include "FirstIncludes.h"
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <map>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>
#include "MemoryDebug.h"


#include "KKBaseTypes.h"
#include "..\\KKBase\\GoalKeeper.h"
#include "OSservices.h"
#include "UmiOSservices.h"
using namespace  KKB;

#include "ScannerFile.h"
using namespace  KKLSC;

#include "UmiKKStr.h"
#include "UmiParticleEntry.h"
#include "UmiScannerFileBuffered.h"
using namespace  CounterManaged;


using namespace  System::Windows::Forms;



UmiScannerFileBuffered::UmiScannerFileBuffered (UmiScannerFile^  _scannerFile):
    clock                   (nullptr),
    cropColLeft             (0),
    cropColRight            (2047),
    croppedWidth            (2048),
    dispColsPerScanCol      (1.0f),
    dispRowsPerScanRow      (1.0f),
    frames                  (nullptr),
    particleEntries         (nullptr),
    particleEntriesLoaded   (false),
    pixelsPerDisLine        (2048),
    ratio                   (1.0f),
    scanColPerDispColRatio  (NULL),
    scanColPerDispColTarget (NULL),
    scanLineSize            (2048),
    scannerFile             (nullptr),
    weOwnScannerFile        (false)
{
  weOwnScannerFile = false;
  scannerFile = _scannerFile;
  Initialization ();
}




    
UmiScannerFileBuffered::UmiScannerFileBuffered (String^     _fileName,
                                                UmiRunLog^  _runLog
                                               ):
    clock                   (nullptr),
    cropColLeft             (0),
    cropColRight            (2047),
    croppedWidth            (2048),
    dispColsPerScanCol      (1.0f),
    dispRowsPerScanRow      (1.0f),
    frames                  (nullptr),
    particleEntries         (nullptr),
    particleEntriesLoaded   (false),
    pixelsPerDisLine        (2048),
    ratio                   (1.0f),
    scanColPerDispColRatio  (NULL),
    scanColPerDispColTarget (NULL),
    scanLineSize            (2048),
    scannerFile             (nullptr),
    weOwnScannerFile        (false)
{
  try
  {
    scannerFile = UmiScannerFile::CreateScannerFile (_fileName, _runLog);
  }
  catch  (IOException^  ioException)
  {
    Console::WriteLine("UmiScannerFileBuffered::UmiScannerFileBuffered   ***EXCEPTION***  CReating Scanner File: " + _fileName + ioException->ToString());
    throw;
  }
      
  if  (scannerFile == nullptr)
  {
    throw gcnew Exception ("UmiScannerFileBuffered:   Could not determine format of file[" + _fileName + "]");
  }
      
  weOwnScannerFile = true;
  Initialization ();
}



UmiScannerFileBuffered::!UmiScannerFileBuffered ()
{
  if  (weOwnScannerFile  &&  (scannerFile != nullptr))
  {
    scannerFile->Close ();
    scannerFile = nullptr;
  }

  delete  scanColPerDispColRatio;    scanColPerDispColRatio  = NULL;
  delete  scanColPerDispColTarget;   scanColPerDispColTarget = NULL;
}



UmiScannerFileBuffered::~UmiScannerFileBuffered ()
{
  this->!UmiScannerFileBuffered ();
}


bool   UmiScannerFileBuffered::LoadThreadsRunning::get ()
{
  return  BuildFrameOffsetsRunning ()  ||  (!particleEntriesLoaded);
}



kkint32  UmiScannerFileBuffered::PixelsPerScanLine::get ()
{
  if  (scannerFile == nullptr)
    return  0;
  else
    return scannerFile->PixelsPerScanLine;
}


String^  UmiScannerFileBuffered::FileName::get ()
{
  if  (scannerFile == nullptr)
    return  nullptr;
  else
    return scannerFile->FileName;
}


bool   UmiScannerFileBuffered::FrameIndexBuilt::get ()
{
  if  (scannerFile == nullptr)
    return true;
  else
    return  scannerFile->FrameIndexBuilt;
}



kkint32  UmiScannerFileBuffered::ScanLinesPerFrame::get ()
{
  if  (scannerFile == nullptr)
    return 0;
  else
    return  scannerFile->ScanLinesPerFrame;
}



float  UmiScannerFileBuffered::ScanRate::get ()
{
  if  (scannerFile == nullptr)
    return 0.0f;
  else
    return  scannerFile->ScanRate;
}



bool  UmiScannerFileBuffered::BuildFrameOffsetsRunning ()
{
  if  (scannerFile)
    return scannerFile->BuildFrameOffsetsRunning;
  else
    return false;
}


array<UmiScannerFile::HeaderField^>^  UmiScannerFileBuffered::GetHeaderFields ()
{
  if  (scannerFile == nullptr)
    return nullptr;
  else
    return scannerFile->GetHeaderFields ();
}



String^  UmiScannerFileBuffered::GetValue (String^  key)
{
  if  (scannerFile == nullptr)
    return nullptr;
  else
    return scannerFile->GetValue (key);
}



void  UmiScannerFileBuffered::FrameBytesOffsetsUpdateThreadStart ()
{
  if  (scannerFile != nullptr)
    scannerFile->FrameBytesOffsetsUpdateThreadStart ();
}



void  UmiScannerFileBuffered::ParticleEntryDataLoadThreadStart (UmiRunLog^  runLog)
{
  ParameterizedThreadStart^ ts = gcnew ParameterizedThreadStart (this, &UmiScannerFileBuffered::LoadParticleEntryData);

  System::Threading::Thread^  t = gcnew System::Threading::Thread (ts);
  t->Name = "LoadParticleEntryData";
  t->Start (runLog);
}



void  UmiScannerFileBuffered::FrameBytesOffsetsUpdateThreadTerminate ()
{
  if  (scannerFile != nullptr)
    scannerFile->FrameBytesOffsetsUpdateThreadTerminate ();
}



void  UmiScannerFileBuffered::LoadParticleEntryData (Object^ data)
{
  UmiRunLog^  runLog = (UmiRunLog^)data;
  particleEntriesLoaded = false;
  particleEntries = nullptr;
  if  (scannerFile != nullptr)
    particleEntries = gcnew UmiParticleEntryList (scannerFile->FileName, 
                                                  false,                  // false = Load entire report file.
                                                  runLog
                                                 );

  particleEntriesLoaded = true;
}




void  UmiScannerFileBuffered::Initialization ()
{
  if  (scannerFile == nullptr)
    return;

  int numBuferedFrames = (int)Math::Ceiling ((15000 / scannerFile->ScanLinesPerFrame));
  scanLineSize = scannerFile->PixelsPerScanLine;
  clock = gcnew UmiScannerClock ();
  int x = 0;
  frames = gcnew array<UmiScannerFrame^> (numBuferedFrames);
  
  bool  found = true;
  
  for (x = 0;  x < frames->Length;  x++)
  {
    frames[x] = gcnew UmiScannerFrame (clock, scannerFile->ScanLinesPerFrame, scanLineSize);
    scannerFile->FrameRead (x, frames[x], found);
    if  (!found)
    {
      frames[x]->FrameNum      = -1;
      frames[x]->ScanLineFirst = -1;
      frames[x]->ScanLineLast  = -1;
    }
  }

  SetPixelsPerDisplayLine (scanLineSize, 0, scanLineSize - 1);
}  /* Initialization */





    
void  UmiScannerFileBuffered::SetPixelsPerDisplayLine (kkint32 _pixelsPerDisLine,
                                                       kkint32 _cropColLeft,
                                                       kkint32 _cropColRight
                                                      )
{
  pixelsPerDisLine = _pixelsPerDisLine;
  cropColLeft      = _cropColLeft;
  cropColRight     = _cropColRight;

  CalculateColumnRatios ();
}



void  UmiScannerFileBuffered::CalculateColumnRatios ()
{
  croppedWidth = 1 + cropColRight - cropColLeft;
  ratio = (float)croppedWidth / (float)pixelsPerDisLine;
  dispColsPerScanCol = (float)pixelsPerDisLine / (float)croppedWidth;
  dispRowsPerScanRow = (float)pixelsPerDisLine / (float)croppedWidth;

  delete  scanColPerDispColRatio;
  delete  scanColPerDispColTarget;

  scanColPerDispColRatio  = new float[scanLineSize];
  scanColPerDispColTarget = new Int16[scanLineSize];

  float  targetDispColCur  = 0.0f;
  float  targetDispColNext = 1.0f / ratio;

  Int16    targetDispColCurInt  = 0;
  Int16    targetDispColNextInt = (Int16)Math::Floor (targetDispColNext);
  
  int scanLineCol = 0;

  while  (scanLineCol < cropColLeft)
  {
    scanColPerDispColTarget[scanLineCol] = -1;
    scanColPerDispColRatio[scanLineCol] = 0.0f;
    scanLineCol++;
  }

  while  (scanLineCol <= cropColRight)
  {
    scanColPerDispColTarget[scanLineCol] = (int)Math::Floor (targetDispColCur);

    if  (Math::Floor (targetDispColCur) == Math::Floor (targetDispColNext))
    {
      scanColPerDispColRatio[scanLineCol] = 1.0f;
    }
    else
    {
      scanColPerDispColRatio[scanLineCol] = ((float)targetDispColNextInt - targetDispColCur) / dispColsPerScanCol;
    }

    scanLineCol++;
    targetDispColCur = targetDispColNext;
    targetDispColNext = (float)(scanLineCol - cropColLeft) / ratio;

    targetDispColCurInt  = targetDispColNextInt;
    targetDispColNextInt = (int)Math::Floor (targetDispColNext);
  }

  while  (scanLineCol < scanLineSize)
  {
    scanColPerDispColTarget[scanLineCol] = Int16::MaxValue;
    scanColPerDispColRatio[scanLineCol] = 0.0f;
    scanLineCol++;
  }

}  /* CalculateTargetCols */




UmiScannerFile^  UmiScannerFileBuffered::OpenAppropriateFormat (String^     _fileName,
                                                               UmiRunLog^  _runLog
                                                              )
{
  try
  {
    scannerFile = UmiScannerFile::CreateScannerFile (_fileName, _runLog);
  }
  catch  (...)
  {
    scannerFile = nullptr;
  }
  return  scannerFile;
}  /* OpenAppropriateFormat  */




UmiScannerFrame^  UmiScannerFileBuffered::OldestFrame ()
{
  int  x = 0;
  
  UmiScannerFrame^  oldest = frames[0];
  kkint32           oldestTime = oldest->Time;
  if  (oldest->FrameNum == -1)
    return  oldest;
  
  for  (x = 1;  x < frames->Length;  x++)
  {
    if  (frames[x]->Time < oldestTime)
    {
      oldest = frames[x];
      oldestTime = oldest->Time;
    }
  }
  
  oldest->Time = 0;
  oldest->FrameNum = -1;
  oldest->ScanLineFirst = -1;
  oldest->ScanLineLast = -1;
  
  return  oldest;
}  /* OldestFrame */





UmiScannerFrame^  UmiScannerFileBuffered::GetFrameWithScanLine (kkint32  scanLine)
{
  kkint32  frameNum = (kkint32)(scanLine / scannerFile->ScanLinesPerFrame);
  kkint32  x = 0;
  bool  frameFound = true;
  
  UmiScannerFrame^  f = nullptr;
  
  for  (x = 0;  x < frames->Length;  x++)
  {
    f = frames[x];
    if  (f->FrameNum == frameNum)
      return  f;
  }
  
  f = OldestFrame ();
  
  scannerFile->FrameRead (frameNum, f, frameFound);
  if  (!frameFound)
    f = nullptr;

  return  f;
}  /* GetFrameWithScanLine */
  
  
  
array<array<byte>^ >^  UmiScannerFileBuffered::GetScanLines  (kkint32  scanLineStart,
                                                              kkint32  scanLineEnd
                                                             )
{
  if  (scanLineStart < 0)
    scanLineStart = 0;
  kkint32  numScanLines = scanLineEnd - scanLineStart + 1;
  array<array<byte>^ >^  scanLines = gcnew array<array<byte>^ > (numScanLines);
  int                    scanLinesNextRow = 0;

  for  (int x = 0;  x < numScanLines;  ++x)
    scanLines[x] = nullptr;
  
  kkint32  nextScanLine = scanLineStart;

  while  (nextScanLine <= scanLineEnd)
  {
    UmiScannerFrame^  f = GetFrameWithScanLine (nextScanLine);
    if  (f == nullptr)
      break;
    
    if  (scannerFile->FrameIndexBuilt  &&  (scannerFile->LargestKnownScanLine  < (kkuint32)nextScanLine))
      break;
    
    array<array<byte>^ >^  thisFrame = f->ScanLines;
    kkint32  thisFrameNextRow = 0;
    
    kkint32  thisFrameFirstRow  = nextScanLine - f->ScanLineFirst;
    kkint32  thisFrameLastRow = Math::Min (scanLineEnd - f->ScanLineFirst, (thisFrame->Length - 1));

    thisFrameNextRow = (kkint32)thisFrameFirstRow;

    while (thisFrameNextRow <= thisFrameLastRow)
    {
      scanLines[scanLinesNextRow] = thisFrame[thisFrameNextRow];
      nextScanLine++;
      scanLinesNextRow++;
      thisFrameNextRow++;
    }
  }
  
  if  (scanLinesNextRow == 0)
    scanLines = nullptr;

  else if (scanLinesNextRow < scanLines->Length)  {
    // We did not get all the scan lines we expected;we need to Fill in the missing rows with blank lines.
    while  (scanLinesNextRow < scanLines->Length)  {
      array<byte>^ scanLine = gcnew array<byte> (scanLineSize);
      for (int zed = 0;  zed < scanLineSize;  ++zed)
        scanLine[zed] = 0;
      scanLines[scanLinesNextRow] = scanLine;
      ++scanLinesNextRow;
    }
  }
  
  return  scanLines;
} /* GetScanLines */                               




UmiRaster^   UmiScannerFileBuffered::GetImage (UmiParticleEntry^  pe)
{
  if  (pe == nullptr)
    return  nullptr;

  Int16  height = pe->Height;

  kkint32  startRow = pe->ScannerRow - 1;
  kkint32  endRow   = startRow + (kkint32)height + 2;

  array<array<byte>^ >^ scannerData = GetScanLines (startRow, endRow);

  kkint32  rowsReturned = scannerData->Length;

  if  (rowsReturned < height)
  {
    // Something went very wrong;  we did not get all the rows that we needed!  Why?
    height = rowsReturned;
  }


  array<byte, 2>^ rasterData = gcnew array<byte, 2> (height, pe->Width);

  for  (kkint32 row = 0;  row < height;  ++row)
  {
    for  (kkint32 col = 0, srcCol = pe->ScannerCol;  col < pe->Width;  ++col, ++srcCol)
      rasterData[row, col] = scannerData[row][srcCol];
  }

  UmiRaster^  r = gcnew UmiRaster (rasterData);
  return  r;
}  /* GetImage */
  
  
  
  
void  UmiScannerFileBuffered::UpdateDisplayRow (array<byte>^      scanLine,
                                                array<kkuint16>^  displayRow,
                                                double            scanLineFrac      /**< Percentage of this row that should contribute to the displayRow. */
                                               )
{
  if  (scanLine == nullptr)
    return;
  
  Int16  scanCol = cropColLeft;
  Int16  dispCol = 0;

  while  (scanCol <= cropColRight)
  {
    byte  pixelValue = scanLine[scanCol];
    if  (pixelValue > 0)
    {
      dispCol = scanColPerDispColTarget [scanCol];
      if  (scanColPerDispColRatio [scanCol] == 1.0f)
      {
        displayRow[dispCol] += (byte)(scanLineFrac * (float)(pixelValue));
      }
      else
      {
        displayRow[dispCol] += (byte)(scanLineFrac * scanColPerDispColRatio [scanCol] * (float)(pixelValue));
        float leftOver = 1.0f - scanColPerDispColRatio [scanCol];
        displayRow[dispCol + 1] += (byte)(scanLineFrac * leftOver * (float)(pixelValue));
      }
    }

    scanCol++;
  }
}  /* UpdateDisplayRow */



UmiParticleEntryList^  UmiScannerFileBuffered::GetParticlesForScanLineRange (kkint32  displayRowStart,
                                                                            kkint32  displayRowEnd
                                                                           )
{
  if  (particleEntries == nullptr)
    return  nullptr;

  kkint32  startScanRow = (kkint32)Math::Floor ((double)displayRowStart     * (double)ratio);
  kkint32  endScanRow   = (kkint32)Math::Floor ((double)(displayRowEnd + 1) * (double)ratio);

  return  particleEntries->GetParticlesForScanLineRange (startScanRow, endScanRow);
}




void  UmiScannerFileBuffered::GetDisplayRows (kkint32               displayRowStart,
                                              kkint32               displayRowEnd,
                                              array<array<byte>^>^  raster
                                             )
{
  kkint32 x = 0;
  kkint32 y = 0;

  double  startScanRow = displayRowStart     * (double)ratio;
  double  endScanRow   = (displayRowEnd + 1) * (double)ratio;

  kkint32  startScanRowInt = (kkint32)Math::Floor (startScanRow);
  kkint32  endScanRowInt   = (kkint32)Math::Floor (endScanRow);

  kkint32  numDisplayRows = displayRowEnd - displayRowStart + 1;

  array<array<byte>^>^   scanRows = GetScanLines  (startScanRowInt, endScanRowInt);

  //kak   When weOwnScannerFile resize from Max Screen when displaying from UnManagedInterface Commander weOwnScannerFile sometimes Get no Data WHY?
  if  (scanRows == nullptr)
    return;

  array<array<kkuint16>^>^ displayRows = gcnew array<array<kkuint16>^> (numDisplayRows);

  for  (x = 0;  x < numDisplayRows;  x++)
    displayRows[x] = gcnew array<ushort> (pixelsPerDisLine);

  kkint32 scanRow = 0;
  double  curDisplayRow  = (float)startScanRowInt / ratio;
  double  nextDisplayRow = curDisplayRow + dispRowsPerScanRow;

  kkint32  curDisplayRowInt  = (kkint32)Math::Floor (curDisplayRow);
  kkint32  nextDisplayRowInt = (kkint32)Math::Floor (nextDisplayRow);

  kkint32  scanRowZeroed = 0;

  for  (scanRow = startScanRowInt;  scanRow <= endScanRowInt;  scanRow++)
  {
    curDisplayRow = (double)scanRow / (double)ratio;
    nextDisplayRow = curDisplayRow + dispRowsPerScanRow;
    curDisplayRowInt  = (kkint32)Math::Floor (curDisplayRow);
    nextDisplayRowInt = (kkint32)Math::Floor (nextDisplayRow);
 
    try
    {
      if  (curDisplayRowInt == nextDisplayRowInt)
      {
        if  (curDisplayRowInt <= displayRowEnd)
          UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[curDisplayRowInt - displayRowStart], 1.0f);
      }
      else
      {
        double  fracNextLogicalRow = (nextDisplayRow - (double)nextDisplayRowInt) / (double)dispRowsPerScanRow;
        double  fracCurLogicalRow  = 1.0 - fracNextLogicalRow;
        if  (curDisplayRowInt >= displayRowStart)
        {
          if  (scanRowZeroed >= scanRows->Length)
             MessageBox::Show ("'scanRowZeroed'  is too large", "UmiScannerFileBuffered::GetDisplayRows");
          
          kkint32  displayRowToUpdate = curDisplayRowInt - displayRowStart;
          if  (displayRowToUpdate < displayRows->Length)
            UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[displayRowToUpdate], fracCurLogicalRow);
        }
        if  (nextDisplayRowInt <= displayRowEnd)
        {
          UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[1 + curDisplayRowInt - displayRowStart], fracNextLogicalRow);
        }
      }
    }
    catch  (Exception^  e)
    {
      MessageBox::Show (e->ToString (), "UmiScannerFileBuffered::GetDisplayRows");
    }

    scanRowZeroed++;
  }
  
  float ratioSquared = ratio * ratio;
  
  for  (x = 0;  x < numDisplayRows;  x++)
  {
    array<ushort>^  oneDisplayRow = displayRows[x];
    array<byte>^    oneRasterRow  = raster[x];  
    for  (y = 0;  y < pixelsPerDisLine;  y++)
      oneRasterRow[y] = (byte)(0.5f + (float)oneDisplayRow[y] / ratioSquared);
  }
  return;
} /* GetDisplayRows */                               
  



UmiRaster^  UmiScannerFileBuffered::GetRasterForParticle (UmiParticleEntry^ pe)
{
  if  (pe == nullptr)
    return nullptr;

  int  peScannerRow  = pe->ScannerRow - 1;
  int  peHeight      = pe->Height + 2;
  int  peScannerCol  = pe->ScannerCol - 1;
  int  peWidth       = pe->Width + 2;

  if  (peScannerRow < 0)
  {
    peHeight = peHeight + peScannerRow;
    peScannerRow = 0;
  }

  Int32  scanLineStart = peScannerRow;
  Int32  scanLineEnd   = peScannerRow + peHeight - 1;

  array<array<byte>^>^ rows = GetScanLines (scanLineStart, scanLineEnd);

  if  (rows == nullptr)
    return nullptr;

  kkint32  particleHeight = peHeight;
  kkint32  particleWidth  = peWidth;

  RasterPtr  particle = new Raster (particleHeight + 2, particleWidth + 2);

  uchar** particleRaster = particle->Green ();

  kkint32  row, col, scannerCol;

  kkint32  particleRow = 1;
  kkint32  particleCol = 1;
  
  for (row = 0;  row < particleHeight;  ++row, ++particleRow)
  {
    array<byte>^  rowData = rows[row];
    if  (rowData == nullptr)
      continue;
    uchar* particleRowData = particleRaster[particleRow];

    scannerCol = peScannerCol;
    col = 0;
    particleCol = 1;
    while (col < particleWidth  &&  scannerCol < rowData->Length)
    {
      particleRowData[particleCol] = rowData[scannerCol];
      ++col;
      ++scannerCol;
      ++particleCol;
    }
  }

  return gcnew UmiRaster (particle);
}  /* GetRasterForParticle */





UmiRaster^  UmiScannerFileBuffered::GetRasterForParticleAsCounted (UmiParticleEntry^        pe,
                                                                   UmiOperatingParameters^  operatingParameters
                                                                  )
{
  String^  fileName = pe->ScannerFileRootName               + "_" +
                      pe->ScannerRow.ToString ("00000000")  + "-" +
                      pe->ScannerCol.ToString ("0000")      +
                      ".bmp";
  UmiRaster^  r = GetRasterForParticle (pe);
  if  (r == nullptr)
    return nullptr;

  r->BackgroundPixelTH = (byte)operatingParameters->BackGroundPixelTH;
  if  (operatingParameters->ErosionStructSize > 0)
  {
    UmiRaster::MaskType  mt = UmiRaster::MaskType::SQUARE3;
    switch  (operatingParameters->ErosionStructSize)
    {
    case  1: mt = UmiRaster::MaskType::SQUARE3;  break;
    case  2: mt = UmiRaster::MaskType::SQUARE5;  break;
    case  3: mt = UmiRaster::MaskType::SQUARE7;  break;
    case  4: mt = UmiRaster::MaskType::SQUARE9;  break;
    case  5: mt = UmiRaster::MaskType::SQUARE11; break;
    }
    r = r->ErodeImage (mt);
  }

  r = r->ConnectedComponent ();
  r = r->StreatchImage (operatingParameters->FlowRateFactor, 1.0f);

  if  (false)
  {
    // This code is useful to make sure that we are re-producing the image for ImageViewer that we are using during LogicalFrameProcessing
    // Feature Computation.
    //
    //String  origFileSubDir = "D:\\Larcos\\ScannerFiles\\2014-02-28_WalkingTreeFarms\\2014-02-28_WTF_006\\WTF_006_20140228-104439_00\\";
    //String  origFileSubDir = "E:\\Larcos\\ScannerFiles\\2014-02-28_WalkingTreeFarms\\2014-02-28_WTF_006\\WTF_006_20140228-104439_00\\";
    //String  origFileSubDir = "E:\\Larcos\\ScannerFiles\\2014-02-28_WalkingTreeFarms\\2014-02-28_WTF_006\\WTF_006_20140228-104439_03\\";
    //String  origFileSubDir = "D:\\Larcos\\ScannerFiles\\2014-02-21_WalkingTreeFarms\\2014-02-21_WTF-_004\\WTF-_004_20140221-104418_00";
    //String  origFileSubDir = "D:\\Larcos\\ScannerFiles\\2014-02-21_WalkingTreeFarms\\2014-02-21_WTF-_004\\WTF-_004_20140221-104418_01";
    String^  origFileSubDir = "D:\\Larcos\\ScannerFiles\\2014-02-21_WalkingTreeFarms\\2014-02-21_WTF-_002\\WTF-_002_20140221-100939_01";

    String^   origFileDir = UmiOSservices::LookForFile (fileName, origFileSubDir);

    if  (!String::IsNullOrEmpty (origFileDir))
    {
      String^  origFileName = UmiOSservices::AddSlash (origFileDir) + fileName;
      UmiRaster^ origImage = gcnew UmiRaster (origFileName);
      origImage->BackgroundPixelTH = (byte)operatingParameters->BackGroundPixelTH;

      // Look for differences
      UmiRaster^ deltaRaster = r->FindMagnitudeDifferences (origImage);
      if  (deltaRaster->CalcWeightedArea () > 0.0f)
      {
        String^  rootName = UmiOSservices::GetRootName (fileName);
        String^  saveNameOrig = "C:\\Temp\\AuditorImageFiles\\"   + rootName + "_Orig.bmp";
        origImage->Save (saveNameOrig);

        String^  auditorName = "C:\\Temp\\AuditorImageFiles\\"    + rootName + "_Auditor.bmp";
        r->Save (auditorName);

        String^  deltaName = "C:\\Temp\\AuditorImageFiles\\"      + rootName + "_Delta.bmp";
        deltaRaster->Save (deltaName);

        String^  deltaColorName = "C:\\Temp\\AuditorImageFiles\\" + rootName + "_DeltaColor.bmp";
        deltaRaster->CreateColor ()->Save (deltaColorName);
      }
    }
  }

  return r;
}  /* GetRasterForParticleAsCounted */







UmiRaster^  UmiScannerFileBuffered::GetThumbNailForParticle (UmiParticleEntry^        pe,
                                                             UmiOperatingParameters^  operatingParameters,
                                                             kkint32                  maxHeight,
                                                             kkint32                  maxWidth
                                                            )
{
  UmiRaster^  r = GetRasterForParticleAsCounted (pe, operatingParameters);

  float  heightReduction = (float)maxHeight / (float)(r->Height);
  float  widthReduction  = (float)maxWidth  / (float)(r->Width);

  float  reduction = Min (heightReduction, widthReduction);
  if  (reduction < 1.0f)
    r = r->ReduceByFactor (reduction);

  return  r;
}  /* GetThumbNailForParticle */





void  UmiScannerFileBuffered::GetDisplayRowsScanColsByScanRows  (kkint32               displayRowStart,
                                                                 kkint32               displayRowEnd,
                                                                 array<array<byte>^>^  raster
                                                                )
{
  kkint32 x = 0;
  kkint32 y = 0;

  double  startScanRow = displayRowStart     * (double)ratio;
  double  endScanRow   = (displayRowEnd + 1) * (double)ratio;

  kkint32  startScanRowInt = (kkint32)Math::Floor (startScanRow);
  kkint32  endScanRowInt   = (kkint32)Math::Floor (endScanRow);

  kkint32  numDisplayRows = displayRowEnd - displayRowStart + 1;

  array<array<byte>^>^   scanRows = GetScanLines  (startScanRowInt, endScanRowInt);

  //kak   When weOwnScannerFile resize from Max Screen when displaying from UnManagedInterface Commander weOwnScannerFile sometimes Get no Data WHY?
  if  (scanRows == nullptr)
    return;

  array<array<kkuint16>^>^ displayRows = gcnew array<array<kkuint16>^> (numDisplayRows);

  for  (x = 0;  x < numDisplayRows;  x++)
    displayRows[x] = gcnew array<ushort> (pixelsPerDisLine);

  kkint32 scanRow = 0;
  double  curDisplayRow  = (float)startScanRowInt / ratio;
  double  nextDisplayRow = curDisplayRow + dispRowsPerScanRow;

  kkint32  curDisplayRowInt  = (kkint32)Math::Floor (curDisplayRow);
  kkint32  nextDisplayRowInt = (kkint32)Math::Floor (nextDisplayRow);

  kkint32  scanRowZeroed = 0;

  for  (scanRow = startScanRowInt;  scanRow <= endScanRowInt;  scanRow++)
  {
    curDisplayRow = (double)scanRow / (double)ratio;
    nextDisplayRow = curDisplayRow + dispRowsPerScanRow;
    curDisplayRowInt  = (kkint32)Math::Floor (curDisplayRow);
    nextDisplayRowInt = (kkint32)Math::Floor (nextDisplayRow);
 
    try
    {
      if  (curDisplayRowInt == nextDisplayRowInt)
      {
        if  (curDisplayRowInt <= displayRowEnd)
          UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[curDisplayRowInt - displayRowStart], 1.0f);
      }
      else
      {
        double  fracNextLogicalRow = (nextDisplayRow - (double)nextDisplayRowInt) / (double)dispRowsPerScanRow;
        double  fracCurLogicalRow  = 1.0 - fracNextLogicalRow;
        if  (curDisplayRowInt >= displayRowStart)
        {
          if  (scanRowZeroed >= scanRows->Length)
             MessageBox::Show ("'scanRowZeroed'  is too large", "UmiScannerFileBuffered::GetDisplayRows");
          
          kkint32  displayRowToUpdate = curDisplayRowInt - displayRowStart;
          if  (displayRowToUpdate < displayRows->Length)
            UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[displayRowToUpdate], fracCurLogicalRow);
        }
        if  (nextDisplayRowInt <= displayRowEnd)
        {
          UpdateDisplayRow (scanRows[scanRowZeroed], displayRows[1 + curDisplayRowInt - displayRowStart], fracNextLogicalRow);
        }
      }
    }
    catch  (Exception^  e)
    {
      MessageBox::Show (e->ToString (), "UmiScannerFileBuffered::GetDisplayRows");
    }

    scanRowZeroed++;
  }
  
  float ratioSquared = ratio * ratio;

  // Each Column in 'raster' represents a ScanRow
  for  (kkint32  scanRow = 0;  scanRow < numDisplayRows;  ++scanRow)
  {
    array<ushort>^  oneDisplayRow = displayRows[scanRow];
    for  (kkint32 scanCol = 0;  scanCol < pixelsPerDisLine;  ++scanCol)
      raster[scanCol][scanRow] = (byte)(0.5f + (float)oneDisplayRow[scanCol] / ratioSquared);
  }
  return;
} /* GetDisplayRowsScanColsByScanRows */                               





kkint32  UmiScannerFileBuffered::ScanColumnFromDisplayCol (kkint32 displayColumn)
{
  kkint32  scanCol = (kkint32)(0.5f + (float)displayColumn / (float)pixelsPerDisLine * (float)croppedWidth) + cropColLeft;
  return  scanCol;
}



void  UmiScannerFileBuffered::AddStartPoint (kkint32  _scanLineNum)
{
  scannerFile->AddStartPoint (_scanLineNum);
}



void  UmiScannerFileBuffered::AddStopPoint (kkint32  _scanLineNum)
{
  scannerFile->AddStopPoint (_scanLineNum);
}


void  UmiScannerFileBuffered::StartStopPointDelete (kkint32 _scanLineNum)
{
  scannerFile->StartStopPointDelete (_scanLineNum);
}


UmiStartStopPoint^  UmiScannerFileBuffered::StartStopPointNearestEntry (kkint32 _scanLineNum)
{
  return  scannerFile->StartStopPointNearestEntry (_scanLineNum);
}


UmiStartStopPoint^  UmiScannerFileBuffered::StartStopPointPrevEntry (kkint32 _scanLineNum)
{
  return  scannerFile->StartStopPointPrevEntry (_scanLineNum);
}


UmiStartStopPoint^  UmiScannerFileBuffered::StartStopPointSuccEntry (kkint32 _scanLineNum)
{
  return  scannerFile->StartStopPointSuccEntry (_scanLineNum);
}



UmiStartStopPointList^  UmiScannerFileBuffered::StartStopPoints ()
{
  return  scannerFile->StartStopPoints ();
}


const StartStopPointList&  UmiScannerFileBuffered::StartStopPointsUnManaged ()
{
  return  scannerFile->StartStopPointsUnManaged ();
}



array<float>^  UmiScannerFileBuffered::RecordRateByTimeIntervals (int intervalSecs)
{
  if  (scannerFile == nullptr)
    return nullptr;
  else 
    return scannerFile->RecordRateByTimeIntervals (intervalSecs);
}






