﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
//using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using System.Threading;

using KKManagedRoutines;
using LarcosCounterManaged;



namespace LarcosCounter
{
  public partial class LarcosCounter : Form
  {
    private  LarcosCounterManagerWrapper  cameraManager = null;

    private  LarcosCounterState  curState = LarcosCounterState._NULL;

    private  String  configFileName = UmiOSservices.AddSlash (UmiVariables.ConfigurationDir ()) + "LarcosCounter.txt";

    private  String  cameraSerialNum = "";

    private  UmiGoalKeeper  cameraManagerGoalKeeper = new UmiGoalKeeper ("CameraManager");

    private  bool resetControls = false;

    private  int  lastHeight = 0;
    private  int  lastWidth  = 0;

    private  int  startUpHeight = 0;
    private  int  startUpWidth = 0;

    private  int  frameProcessorsCount = 0;

    private  bool  runLogTimerCallCountingStop = false;
    private  bool  runLogTimerExitApplication  = false;

    private  StatusSnapshotManaged.DataFieldIdx  throughPutField = StatusSnapshotManaged.DataFieldIdx.Count;
    private  String                              secondaryToDisplay = "Capacity";

    private  TabPage         statsTabPage  = null;   ///< Points to the Tab page that contains Statistics; this way will be able to add and remove page as needed */
    private  TabPage         dialogTabPage = null;
    private  TabPage         defaultOperatingParametersPage = null;

    private  CameraPreView   cameraPreviewWindow = null;

    private  ToolTip         toolTip = null;

    private  UmiRunLog       runLog = null;  


    public  LarcosCounter ()
    {
      SplashScreen sc = new SplashScreen ();
      sc.Show (this);

      InitializeComponent();

      statsTabPage                   = TabDisplayed.TabPages["statsTab"];
      dialogTabPage                  = TabDisplayed.TabPages["dialogPage"];
      defaultOperatingParametersPage = TabDisplayed.TabPages["DefaultOperatingParametersTab"];

      lastHeight = Height;
      lastWidth  = Width;

      cameraManager = new LarcosCounterManagerWrapper ();
      cameraManager.Initialize ();

      runLog = cameraManager.GetUmiRunLogInstance ();
      runLog.WriteLn (10, "LarcosCounter  Loading.");

      throughPutField = cameraManager.ThroughPutField ();
      secondaryToDisplay = "Capacity";

      PopulateThroughPutDataToDisplayChoices ();
      PopulateSecondaryToDisplay ();

      LoadConfiguration ();

      SetEnableDisable ();

      liveVideoFrame.MouseClick += new MouseEventHandler (MonitorLiveVideoMouseClick);
      SetUpLiveVideoFrameContectMenu ();
      cameraManager.SetLiveVideoDimensions (liveVideoFrame.Height, liveVideoFrame.Width);

      UpdateLarcosStatusDisplayNotRunning ();

      UpdateThrouputChartData ();

      int  numCpus = Environment.ProcessorCount;

      if  ((startUpHeight > 0)  &&  (startUpWidth > 0))
      {
        Height = startUpHeight;
        Width  = startUpWidth;
      }

      cameraLine.SetCameraManager (cameraManager);

      SetUpToolTips ();
    }



    private void  LarcosCounter_Load (object sender, EventArgs e)
    {
      Text = "Larcos Counter      ver: " + cameraManager.LarcosVersionNumber ();
      if  (cameraManager.OperatingMode () == LarcosOperatingModes.Advanced)
      {
        SampleBeforeFlatField.Visible    = true;
        SampleBeforeFlatField.Enabled    = true;
        ThroughPutDataToDiosplay.Visible = true;
        ThroughPutDataToDiosplay.Enabled = true;
      }
      else
      {
        SampleBeforeFlatField.Visible    = false;
        SampleBeforeFlatField.Enabled    = false;
        ThroughPutDataToDiosplay.Visible = false;
        ThroughPutDataToDiosplay.Enabled = false;
      }
      SetUpFlowMeterIncluded ();
    }


    
    private  void  SetUpToolTips ()
    {
      toolTip = new ToolTip ();
      toolTip.SetToolTip (ConnectToCameraButton,         "Press this to connect to camera.");
      toolTip.SetToolTip (RecordButton,                  "Start a new recording and Counting session.");
      toolTip.SetToolTip (AuditButton,                   "Reviews previously recorded session allowing user to select sections to include or exclude as part of count; and then produce finale report.");

      toolTip.SetToolTip (SampleBeforeFlatField,         "Select whather to display live video before or after Flat-Field-Correction.");

      // Run Time Parameters.
      toolTip.SetToolTip (ErosionStructSize,             "Specify how vigoruisly to Clean-Up imagery before counting; In verty clean water no larger that 1; Ib very dirty water may need 3 or larger.");
      toolTip.SetToolTip (MinSizeThreshold,              "The minimum number of pixels that a particle must consist of before beconsidered for counting.");
      toolTip.SetToolTip (BackGroundPixelTH,             "The minimum intensity after Flat-Field-Correction that a Pixle must be to be considered forground; pixels with smaller value will be ignored.");
      toolTip.SetToolTip (RequestedScanRate,             "Scan Rate (Lines-Per-Second) that camera is to record at.");
      toolTip.SetToolTip (FlowRateFactor,                "Factor that is used to compensate for vwelocity of water.");
      toolTip.SetToolTip (FileFormat,                    "Format that Scanner Data is to be recoded to disk with; this selection controls how many levels of Gray-Scale will be saved; typical selection will be '3Bit'.");
      toolTip.SetToolTip (SnapshotInterval2,             "Interval in seconds that Through-Put chart is to display.");
      toolTip.SetToolTip (FlatFieldEnabled,              "If not turned on Scanber Imagery will be recorded to disk as recorded by camera; typicacly use in cunjunction with the 'Simple' file format.");
      toolTip.SetToolTip (DataIsToBeRecorded,            "Typically Checked; if not set then NO Scanner Imagery will be recorded to disk; this means there would be no way to Play-Back or Audit count.");
      toolTip.SetToolTip (DataIsToBeCounted,             "Typically Checked; indicates wheather to count Shrimp.");
      toolTip.SetToolTip (SaveParticleImages,            "NOT Typically Checked;  When selected will write individual images for each particle encountered, where images will be aranged by sub-directory indicating classification;  you would do this during a Play-Back session.");
      toolTip.SetToolTip (PlayingBackRealTime,           "NOT Typically Checked;  When selected the playback will try to match the speed of the camera during the original recoding session.");
      toolTip.SetToolTip (ThroughPutDataToDiosplay,      "Select the data that you wish to Chart.");
      toolTip.SetToolTip (SecondaryToDisplay,            "Select  'Capacity'  or  'Flow Rate'  to display on secondary axis.");
      toolTip.SetToolTip (CameraTemparature,             "Camera Temperature.");
      toolTip.SetToolTip (FlowRate,                      "Flow rate in Meters/Second");
      toolTip.SetToolTip (Count,                         "The current Shrimp Count.");
      toolTip.SetToolTip (ModifyDefaultParametersButton, "Brings up dialog box that will allow you to modify Default Operating Parameters.");
    }


    private  void   SetUpFlowMeterIncluded ()
    {
      if  (cameraManager.EmbeddedFlowMeter ())
      {
        SecondaryToDisplay.Visible = true;
        SecondaryToDisplay.Enabled = true;
      }
      else
      {
        secondaryToDisplay = "Capacity";
        SecondaryToDisplay.SelectedItem = secondaryToDisplay;
        SecondaryToDisplay.Visible = false;
        SecondaryToDisplay.Enabled = false;
      }
    }



    private  void  PopulateThroughPutDataToDisplayChoices ()
    {
      ThroughPutDataToDiosplay.DataSource = StatusSnapshotManaged.DataFieldIdxNames ();
    }


    private  void  PopulateSecondaryToDisplay ()
    {
      String[]  choices = {"Capacity", "Flow Rate"};
      SecondaryToDisplay.DataSource = choices;
    }
       
    
    
    private  void  SetUpLiveVideoFrameContectMenu ()
    {
      ContextMenu  liveVideoContextMenu = new ContextMenu ();

      int  cropLeft  = cameraManager.CropLeft ();
      int  cropRight = cameraManager.CropRight ();
      int  rightMostCol = cameraManager.FrameWidthInPixels () - 1;
      
      liveVideoContextMenu.MenuItems.Add ("Set LeftCrop", new EventHandler (SetLeftCrop));
      if  (cropLeft > 0)
        liveVideoContextMenu.MenuItems.Add ("Release LeftCrop", new EventHandler (ReleaseLeftCrop));
          
      liveVideoContextMenu.MenuItems.Add ("Set RightCrop", new EventHandler (SetRightCrop));
      if  (cropRight < rightMostCol)
        liveVideoContextMenu.MenuItems.Add ("Release RightCrop", new EventHandler (ReleaseRightCrop));

      if  (cameraManager.OperatingMode () == LarcosOperatingModes.Advanced)
        liveVideoContextMenu.MenuItems.Add ("Display Temporal Camera-Line", new EventHandler (DisplayTemporalCameraLine));
      
      liveVideoFrame.ContextMenu = liveVideoContextMenu;
    }  /* SetUpLiveVideoFrameContectMenu */


    int  liveVideoMouseCol = 0;



    private  void  MonitorLiveVideoMouseClick (object  sender, MouseEventArgs mea)
    {
      if  (mea.Button == MouseButtons.Right)
        liveVideoMouseCol = mea.X;
    }


    private  void  ReleaseLeftCrop (object sender, System.EventArgs e)
    {
      CropLeft.Text = "0";
      cameraManager.CropLeft (0);
      SetUpLiveVideoFrameContectMenu ();
    } /* ReleaseLeftCrop */




    private  void  SetLeftCrop (object sender, System.EventArgs e)
    {
      int  width = liveVideoFrame.Width;
      if  (liveVideoFrame.Image != null)
        width = liveVideoFrame.Image.Width;
      if  (width < 1)
        return;

      int  cropLeft  =  cameraManager.CropLeft  ();
      int  cropRight =  cameraManager.CropRight ();
      int  cropWidth = 1 + cropRight - cropLeft;
      int  newCropLeft = (cropWidth * liveVideoMouseCol / width) + cropLeft;
      if  (newCropLeft > (cropRight - 100))
      {
        MessageBox.Show ("The Left-Crop can not be greater than the right crop.");
        return;
      }

      cameraManager.CropLeft (newCropLeft);
      CropLeft.Text = newCropLeft.ToString ();

      SetUpLiveVideoFrameContectMenu ();
    }


    private  void  ReleaseRightCrop (object sender, System.EventArgs e)
    {
      int  cropRight = cameraManager.FrameWidthInPixels () - 1;
      cameraManager.CropRight (cropRight);
      CropRight.Text = cropRight.ToString ();
      SetUpLiveVideoFrameContectMenu ();
    } /* ReleaseLeftCrop */


    private  void  DisplayTemporalCameraLine (object sender, System.EventArgs e)
    {
      UmiRaster  lastFrame = cameraManager.SnapShotLatestFrame ();
      if  (lastFrame == null)
        return;

      byte[][] rasterData = lastFrame.BuildRasterArray2 ();
      if  ((rasterData == null)  ||  (rasterData.Length < 1))
        return;

      int  width = rasterData[0].Length;

      int  cropLeft  =  cameraManager.CropLeft  ();
      int  cropRight =  cameraManager.CropRight ();
      int  cropWidth = 1 + cropRight - cropLeft;
      int  col = (cropWidth * liveVideoMouseCol / width);
      if  (col >= width)
        col = width - 1;

      TemporalDisplay  td = new TemporalDisplay (rasterData, col, 0, width - 1, false);
      td.ShowDialog (this);
    }




    private  void  SetRightCrop (object sender, System.EventArgs e)
    {
      int  width = liveVideoFrame.Width;
      if  (liveVideoFrame.Image != null)
        width = liveVideoFrame.Image.Width;
      if  (width < 1)
        return;

      int cropLeft  =  cameraManager.CropLeft  ();
      int cropRight =  cameraManager.CropRight ();

      int  cropWidth = 1 + cropRight - cropLeft;
      int  newCropRight = (cropWidth * liveVideoMouseCol / width) + cropLeft;
      if  (newCropRight < (cropLeft + 100))
      {
        MessageBox.Show ("The Left-Crop can not be greater than the right crop.");
        return;
      }

      cameraManager.CropRight (newCropRight);
      CropRight.Text = newCropRight.ToString ();
      SetUpLiveVideoFrameContectMenu ();
    } /* SetLeftCrop */



    private bool  loadConfigurationInProgress = true;

    private  void  LoadConfiguration ()
    {
      loadConfigurationInProgress  = true;

      StreamReader f = null;
      try  {f = new StreamReader (configFileName);}
      catch (Exception)  {return;}

      if  (f == null)
      {
        loadConfigurationInProgress  = false;
        return;
      }

      string txtLine = f.ReadLine ();
      while (txtLine != null)
      {
        if  ((txtLine.Length > 1)  &&  (txtLine.Substring (0, 2) != "//"))
        {
          String[]  fields = txtLine.Split ('\t');
          if  (fields.Length > 1)
          {
            String  rowType = fields[0].ToLower ().Trim ();
            String  fieldValue = fields[1];

            if  (rowType == "height")
              startUpHeight = UmiKKStr.StrToInt (fieldValue);

            else if  (rowType == "width")
              startUpWidth  = UmiKKStr.StrToInt (fieldValue);

            else if  (rowType == "endoffile")
              break;
          }
        }
        
        txtLine = f.ReadLine();
      }

      f.Close();

      SampleBeforeFlatField.Checked = cameraManager.SampleLastFrameBeforeFlatField ();

      UpdateDefaultOperatingParametersTab ();

      ThroughPutDataToDiosplay.Text = StatusSnapshotManaged.DataFieldIdxToStr (throughPutField);
      SecondaryToDisplay.Text = secondaryToDisplay;

      CropLeft.Text               = cameraManager.CropLeft ().ToString ();
      CropRight.Text              = cameraManager.CropRight ().ToString ();
      PlayingBackRealTime.Checked = cameraManager.PlayingBackRealTime ();

      SnapshotInterval2.Value = (decimal)cameraManager.SnapshotInterval ();

      cameraManager.SetLiveVideoDimensions (liveVideoFrame.Height, liveVideoFrame.Width);

      UpdateControlsForCurrentOperatingMode ();

      loadConfigurationInProgress  = false;
    }  /* LoadConfiguration */



    private  void  UpdateDefaultOperatingParametersTab ()
    {
      UmiOperatingParameters  defaultOperatingParameters = cameraManager.GetDefaultOperatingParameters ();

      FileFormat.Text            = defaultOperatingParameters.DestScannerFileFormatStr;
      MinSizeThreshold.Text      = defaultOperatingParameters.MinSizeThreshold.ToString ();
      ErosionStructSize.Value    = (decimal)defaultOperatingParameters.ErosionStructSize;
      BackGroundPixelTH.Value    = (decimal)defaultOperatingParameters.BackGroundPixelTH;
      FlowRateFactor.Value       = (decimal)defaultOperatingParameters.FlowRateFactor;
      RequestedScanRate.Value    = (decimal)defaultOperatingParameters.RequestedScanRate;

      DataIsToBeCounted.Text     = defaultOperatingParameters.DataIsToBeCounted  ? "Yes" : "No";
      FlatFieldEnabled.Text      = defaultOperatingParameters.FlatFieldEnabled   ? "Yes" : "No";
      DataIsToBeRecorded.Text    = defaultOperatingParameters.DataIsToBeRecorded ? "Yes" : "No";
      SaveParticleImages.Text    = defaultOperatingParameters.SaveParticleImages ? "Yes" : "No";


      float  curAnalogGain = 0.0f;
      int    curDigitalGain = 0;
      bool   autoGainRunning = false;
      cameraManager.GetGainSettings (ref autoGainRunning, ref curAnalogGain, ref curDigitalGain);
     
      AnalogGain.Text  = curAnalogGain.ToString ("-#,##0.0");
      DigitalGain.Text = curDigitalGain.ToString ("##,##0");
    }  /* UpdateDefaultOperatingParametersTab */



    public  void  SaveConfiguration ()
    {
      if  (loadConfigurationInProgress)
        return;

      StreamWriter w = null;
      try  {w = new StreamWriter (configFileName);}
      catch (Exception e)
      {
        MessageBox.Show ("Exception Writing File Viewer Configuration.  [" + e.ToString() + "]");
        return;
      }
      
      w.WriteLine ("// LarcosCounter Configuration");
      w.WriteLine ("// DateTime" + "\t" + DateTime.Now.ToString());
      w.WriteLine ("//");
      w.WriteLine ("//");
      w.WriteLine ("Height" + "\t" + Height.ToString ()); 
      w.WriteLine ("Width"  + "\t" + Width.ToString ()); 
      w.WriteLine ("EndOfFile");
      w.Close();
      return;
    }  /* SaveConfiguration */



    void  DisableAllControls ()
    {
      ConnectToCameraButton.Enabled         = false;
      AuditButton.Enabled                   = false;
      RecordButton.Enabled                  = false;
      ModifyDefaultParametersButton.Enabled = false;
      SnapshotInterval2.Enabled             = false;
      PlayingBackRealTime.Enabled           = false;
    }


    void  SetEnableDisable ()
    {
      DisableAllControls ();
      switch  (curState)
      {
        case  LarcosCounterState._NULL:
        case  LarcosCounterState.Stopped:
        {
          ConnectToCameraButton.Enabled          = true;
          AuditButton.Enabled                    = true;
          PlayBackProgress.Visible               = false;
          AuditButton.Enabled                    = true;
          ModifyDefaultParametersButton.Enabled  = true;
          RecordButton.Text                      = "Start New Session";
          SnapshotInterval2.Enabled              = true;
          PlayingBackRealTime.Enabled            = true;
          break;
        }

        case  LarcosCounterState.Stopping:
        {
          // Until we stopped all the counting threads there is nothing the user can do.
          break;
        }

        case  LarcosCounterState.Connected:
        {
          AuditButton.Enabled                   = true;
          PlayBackProgress.Visible              = false;
          RecordButton.Text                     = "Start New Session";
          RecordButton.Enabled                  = true;
          ModifyDefaultParametersButton.Enabled = true;
          SnapshotInterval2.Enabled             = true;
          PlayingBackRealTime.Enabled           = true;

          break;
        }

        case  LarcosCounterState.Connecting:
        {
          // Until the connection attempt is done there is nothing that the user can do.
          break;
        }

        case  LarcosCounterState.Starting:
        case  LarcosCounterState.BuildingClassifier:
        {
          RecordButton.Text = "Stop";
          RecordButton.Enabled = true;
          break;
        }

        case  LarcosCounterState.PlayingBack:
        case  LarcosCounterState.Running:
        {
          FlowRateFactor.Enabled      = true;
          MinSizeThreshold.Enabled    = true;
          RecordButton.Text           = "Stop";
          RecordButton.Enabled        = true;
          SnapshotInterval2.Enabled   = true;
          PlayingBackRealTime.Enabled = true;

          if  (curState == LarcosCounterState.PlayingBack)
          {
            PlayBackProgress.Enabled = true;
            PlayBackProgress.Visible = true;
          }
          else
          {
            PlayBackProgress.Visible = false;
          }

          break;
        }
      }

      resetControls = false;
    }  /* SetEnableDisable */




    private  void  UpdateControlsForCurrentOperatingMode ()
    {
      if  (cameraManager.OperatingMode () == LarcosOperatingModes.User)
      {
        if  (TabDisplayed.TabPages.Contains (statsTab))
          TabDisplayed.TabPages.Remove (statsTab);

        if  (TabDisplayed.TabPages.Contains (dialogTabPage))
          TabDisplayed.TabPages.Remove (dialogTabPage);

        if  (TabDisplayed.TabPages.Contains (defaultOperatingParametersPage))
          TabDisplayed.TabPages.Remove (defaultOperatingParametersPage);


        if  (cameraPreviewWindow != null)
        {
          cameraPreviewWindow.Close ();
          cameraPreviewWindow = null;
        }
      }
      else
      {
        if  (!TabDisplayed.TabPages.Contains (statsTab))
          TabDisplayed.TabPages.Add (statsTab);

        if  (!TabDisplayed.TabPages.Contains (dialogTabPage))
          TabDisplayed.TabPages.Add (dialogTabPage);

        if  (!TabDisplayed.TabPages.Contains (defaultOperatingParametersPage))
          TabDisplayed.TabPages.Add (defaultOperatingParametersPage);

        if  ((cameraPreviewWindow == null)  ||  (!cameraPreviewWindow.Visible))
          StartCameraPreView ();
      }
    }  /* UpdateControlsForCurrentOperatingMode */



    private  void  RetrieveLoggingMessages ()
    {
      cameraManagerGoalKeeper.StartBlock ();
      if  (cameraManager != null)
      {
        List<String> msgs = cameraManager.GetAllLoggedMsgs ();
        if  (msgs != null)
        {
          foreach (String s in msgs)
          {
            LogMsgs.AppendText (s);
            LogMsgs.AppendText ("\n");
          }
        }
      }
      cameraManagerGoalKeeper.EndBlock ();
    }



    private  void  UpdateStatusFields ()
    {
      if  (cameraManager == null)  
        return;

      LarcosCounterStatsManaged  stats = cameraManager.GetStats ();
      
      LostPackets.Text                     = stats.TotalLostPackets.ToString                ("##,###,##0");
      PhysicalSeqNumsSkipped.Text          = stats.PhysicalSeqNumsSkipped.ToString          ("##,###,##0");
      FrameRate.Text                       = stats.FrameRate.ToString                       ("###,##0.00");
      ScanRate.Text                        = stats.ScanRate.ToString                        ("###,##0.00");
      ScanLinesRead.Text                   = stats.ScanLinesRead.ToString                   ("###,###,##0");
      AnalogGain.Text                      = stats.CurAnalogGain.ToString                   ("#0.0");
      DigitalGain.Text                     = stats.CurDigitalGain.ToString                  ("##,##0");
      Sensitivity.Text                     = stats.CurSensitivity;
      BytesWritten.Text                    = stats.BytesWrittenAllSegs.ToString             ("#,###,###,##0");
      ScanLines.Text                       = stats.ScanLinesWritten.ToString                ("##,###,##0");
      WaitingToWriteToDisk.Text            = stats.WaitingToWriteToDisk.ToString            ("##,###,##0");
      DWSeqNumBreaks.Text                  = stats.DiskWritingSeqNumBreaks.ToString         ("##,###,##0");
      PhysicalFramesRead.Text              = stats.PhysicalFramesRead.ToString              ("##,###,##0");
      PhysicalFramesDropped.Text           = stats.PhysicalFramesDropped.ToString           ("##,###,##0");
      PhysicalFramesWaitingToProcess.Text  = stats.PhysicalFramesWaitingToProcess.ToString  ("##,###,##0");
      PhysicalFramesProcessed.Text         = stats.PhysicalFramesProcessed.ToString         ("##,###,##0");
      LogicalFramesCreated.Text            = stats.LogicalFramesCreated.ToString            ("##,###,##0");
      LogicalFramesDropped.Text            = stats.LogicalFramesDropped.ToString            ("##,###,##0");
      LogicalFramesOnQueue.Text            = stats.LogicalFramesOnQueue.ToString            ("##,###,##0");
      LogicalFramesProcessed.Text          = stats.LogicalFramesProcessed.ToString          ("##,###,##0");
      LogicalFrameProcessorsAvailable.Text = stats.LogicalFrameProcessorsAvailable.ToString ("##0");
      ParticlesExtracted.Text              = stats.ParticlesExtracted.ToString              ("##,###,##0");
      ParticlesWaiting.Text                = stats.ParticlesWaitingProcessing.ToString      ("##,###,##0");
      ParticlesCounted.Text                = stats.ParticlesCounted.ToString                ("##,###,##0");

      String  newCamaraSerialNum = cameraManager.CameraSerialNum ();
      if  (newCamaraSerialNum.CompareTo (cameraSerialNum) != 0)
      {
        cameraSerialNum = newCamaraSerialNum;
        Text = "Count Monitor   Camera[" + cameraSerialNum + "]";
      }

      CropLeft.Text  = stats.CropLeft.ToString ();
      CropRight.Text = stats.CropRight.ToString ();
    } /* UpdateStatusFields */




    private void  UpdatePlayBackProgress ()
    {
      if  (curState == LarcosCounterState.PlayingBack)
      {
        Int64  totalBytesToRead = cameraManager.TotalBytesToRead ();
        Int64  totalBytesRead   = cameraManager.TotalBytesRead ();
             
        int percentDone = 0;
        if  (totalBytesToRead > 0.0)
          percentDone = Math.Min (100, (int)(100.0 * (double)totalBytesRead / (double)totalBytesToRead));

        PlayBackProgress.Value = percentDone;
      }
    }
    



    String    lastSecondaryMsg = "";
    int       lastSecondaryMsgBlanksInARow = 0;
    DateTime  lastSecondaryTimeStamp;

    private  void  UpdateLarcosStatusDisplay ()
    {
      if  (cameraManager == null)
        return;

      String  statusMsg    = null;
      String  color        = null;
      String  secondaryMsg = null;

      float analogGain = 0.0f;
      Int32 digitalGain = 0;

      Int32 cameraTemparature = -1;

      float  flowRate = 1.0f;

      cameraManager.Status (ref statusMsg, 
                            ref color, 
                            ref secondaryMsg,
                            ref analogGain,
                            ref digitalGain,
                            ref cameraTemparature,
                            ref flowRate
                           );

      CameraTemparature.Text = cameraTemparature.ToString ();
      FlowRate.Text = flowRate.ToString ("#0.00") + "m/s";

      if  (String.IsNullOrEmpty (secondaryMsg))
      {
        secondaryMsg = lastSecondaryMsg;
        lastSecondaryMsgBlanksInARow++;
        if  (!String.IsNullOrEmpty (lastSecondaryMsg))
        {
          TimeSpan ts = DateTime.Now - lastSecondaryTimeStamp;
          if  (ts.TotalSeconds > 5.0)
            lastSecondaryMsg = "";
        }
      }
      else
      {
        lastSecondaryMsg = secondaryMsg;
        lastSecondaryMsgBlanksInARow = 0;
        lastSecondaryTimeStamp = DateTime.Now;
      }

      AnalogGain.Text  = analogGain.ToString ("#0.0");
      DigitalGain.Text = digitalGain.ToString ("##,##0");

      if  (statusMsg != null)
        LarcosStatus.Text = statusMsg;

      LarcosStatus2.Text = secondaryMsg;

      if  (color == "Red")
      {
        LarcosStatus.ForeColor = Color.White;
        LarcosStatus.BackColor = Color.Red;
      }

      else if  (color == "Green")
      {
        LarcosStatus.ForeColor = Color.White;
        LarcosStatus.BackColor = Color.Green;
      }

      else if  (color == "Orange")
      {
        LarcosStatus.ForeColor = Color.Black;
        LarcosStatus.BackColor = Color.Orange;
      }

      else
      {
        LarcosStatus.ForeColor = LarcosStatus2.ForeColor;
        LarcosStatus.BackColor = LarcosStatus2.BackColor;
      }

      Count.Text = cameraManager.ParticlesCounted ().ToString ("##,###,##0");
    }  /* UpdateLarcosStatusDisplay*/
    


    private  void  UpdateLarcosStatusDisplayNotRunning ()
    {
      LarcosStatus.Text = "";
      LarcosStatus2.Text = "";
      LarcosStatus.ForeColor = LarcosStatus2.ForeColor;
      LarcosStatus.BackColor = LarcosStatus2.BackColor;
    }


    private  void  RefreshLiveVideo ()
    {
      if  (cameraManager == null)
        return;

      if  (!liveVideoFrame.Visible)
        return;

      Bitmap bm = cameraManager.SnapShotLatestFrame (liveVideoFrame.Height, liveVideoFrame.Width);
      if  (bm != null)
      {
        //bm.RotateFlip (RotateFlipType.Rotate180FlipX);
        liveVideoFrame.SizeMode = PictureBoxSizeMode.CenterImage;
        liveVideoFrame.Image = bm;
      }
    }  /* RefreshLiveVideo */



    private  void  CountingInitiate ()
    {
      int  snapshotInterval = Math.Max (1, (int)(SnapshotInterval2.Value));
      
      SamplingIntervalTimer.Interval = 1000 * snapshotInterval;
      SamplingIntervalTimer.Enabled = true;
      cameraManager.SnapshotInterval (snapshotInterval);

      cameraManager.PlayingBackRealTime (PlayingBackRealTime.Checked);

      frameProcessorsCount = Math.Max (1, cameraManager.FrameProcessorsCount ());

      ThroughputChart.Series[0].LegendText = StatusSnapshotManaged.DataFieldIdxToStr (throughPutField);
      ThroughputChart.Series[0].Points.Clear ();
      ThroughputChart.Series[1].LegendText = secondaryToDisplay;
      ThroughputChart.Series[1].Points.Clear ();
      ThroughputChart.ChartAreas[0].AxisX.ScaleView.Size = ThroughputChart.Width / 50;
    }
    


    private  void  CountingStop ()
    {
      SamplingIntervalTimer.Interval = 1000 * Math.Max (1, (int)SnapshotInterval2.Value);
      SamplingIntervalTimer.Enabled = false;
      runLogTimerCallCountingStop = false;
    }



    //private  void  GenerateFinaleReport ()
    //{
    //  if  (cameraManager == null)
    //    return;
    //
    //  // Because 'FinaleReport' fr is going to run in a modeless window I wanted it tio have its own copy of a UmiRunLog.
    //  FinaleReport  fr = new FinaleReport (cameraManager, cameraManager.LastScannerFileCounted (), cameraManager.GetUmiRunLogInstance ());
    //  fr.Show (this);
    //} /* GenerateFinaleReport */



    private  void   RunAuditFunction ()
    {
      runLog.WriteLn (10, "RunAuditFunction");
      curState = cameraManager.CurState ();

      DisableAllControls ();

      String  sourceScannerFileName = cameraManager.LastScannerFileCounted ();
      Auditor auditor = new Auditor (cameraManager, sourceScannerFileName, runLog);
      auditor.ShowDialog (this);
      if (auditor.RunPlayBack)
      {
        RunPlayBackForScannerFile (sourceScannerFileName);
      }
      else
      {
        SetEnableDisable ();
      }
    }




    private  int                 ticks = 0;
    private  LarcosCounterState  lastState = LarcosCounterState._NULL;

    private void RunLogTimer_Tick(object sender, EventArgs e)
    {
      curState = cameraManager.CurState ();
      if  ((curState != lastState)  ||  resetControls)
      {
        SetEnableDisable ();
        lastState = curState;

        if  (curState == LarcosCounterState.Stopped)
        {
          if  (runLogTimerCallCountingStop)
            CountingStop ();

          else if  (runLogTimerExitApplication)
            Close ();
        }

        if  (cameraManager.GenerateFinaleReport ())
        {
          RunAuditFunction ();
        }
      }

      cameraManagerGoalKeeper.StartBlock ();

      if  ((ticks % 1) == 0)
      {
        RefreshLiveVideo ();

        if  ((ticks % 4) == 0)
        {
          if  (StatsPannel.Visible)
            UpdateStatusFields ();

          UpdateLarcosStatusDisplay ();
          RetrieveLoggingMessages ();

          if  ((ticks % 5) == 0)
          {
            if  (curState == LarcosCounterState.PlayingBack)
              UpdatePlayBackProgress ();
           
            if  ((ticks % 40) == 0)
              GC.Collect ();
          }
        }
      }

      cameraManagerGoalKeeper.EndBlock ();

      ticks++;
    }



    private void LarcosCounter_FormClosed (object sender, FormClosedEventArgs e)
    {
      cameraManager = null;
    }



    private void  ConnectToCameraButton_Click (object sender, EventArgs e)
    {
      runLog.WriteLn  (10, "ConnectToCameraButton_Click.");
      DisableAllControls ();
      cameraManager.ConnectButtonPressed ();
      resetControls = true;
      SetEnableDisable ();
    }



    private void LarcosCounter_FormClosing (object sender, FormClosingEventArgs e)
    {
      curState = cameraManager.CurState ();

      runLog.WriteLn  (10, "LarcosCounter_FormClosing");

      if  (e.CloseReason == CloseReason.UserClosing)
      {
        runLog.WriteLn  (10, "User Exiting Window   CurrentState :" + curState.ToString ());
        if  ((curState == LarcosCounterState.Starting)           ||
             (curState == LarcosCounterState.BuildingClassifier) ||
             (curState == LarcosCounterState.PlayingBack)        ||
             (curState == LarcosCounterState.Running)            ||
             (curState == LarcosCounterState.DroppedFrames)      ||
             (curState == LarcosCounterState.Stopping)
            )
        {
          runLog.WriteLn  (10, "Because we are still processing user required to approve exit of application.");
          DialogResult dr = MessageBox.Show 
                          (this, 
                           "The Counter is running; are you sure that you want to exit ?", 
                           "Counter running",
                           MessageBoxButtons.YesNo
                          );
          if  (dr == DialogResult.No)
          {
            e.Cancel = true;
            runLog.WriteLn  (10, "User canceled clsoing of window/application.");
            return;
          }
          else
          {
            runLog.WriteLn  (10, "User agreed to continue exiting window/application.");
          }
        }
      }

      SaveConfiguration ();
       
      cameraManager.TerminateAllThreads ();
    }



    private  void  StartCameraPreView ()
    {
      if  (cameraPreviewWindow != null)
      {
        cameraPreviewWindow.Close ();
        cameraPreviewWindow = null;
      }

      if  (cameraManager.OperatingMode () == LarcosOperatingModes.Advanced)
      {
        cameraPreviewWindow = new CameraPreView (cameraManager);
        cameraPreviewWindow.Show ();
      }
    }



    private void RecordButton_Click (object sender, EventArgs e)
    {
      runLog.WriteLn  (10, "RecordButton_Click");

      DisableAllControls ();
      curState = cameraManager.CurState ();
   
      SaveConfiguration ();

      if  (this.RecordButton.Text == "Stop")
      {
        runLog.WriteLn  (10, "Stop Button Pressed.");
        String errMsg    = "";
        bool  successful = false;
        cameraManager.StopButtonPressed (false, ref successful, ref errMsg);
        if  (!successful)
          MessageBox.Show (this, errMsg, "Stop Button");

        runLogTimerCallCountingStop = true;

        return;
      }

      else if  (curState == LarcosCounterState.Connected)
      {
        runLog.WriteLn  (10, "Starting new session.");
        bool  successful = false;
        String  errMsg = "";

        UmiSessionParameters    sessionParameter    = cameraManager.GetSessionParameters ();
        UmiOperatingParameters  operatingParameters = cameraManager.GetOperatingParameters ();
        operatingParameters.DataIsToBeCounted = true;
        operatingParameters.DataIsToBeRecorded = true;

        StartRecordingDialog  srd = new StartRecordingDialog (cameraManager, sessionParameter, operatingParameters, false, runLog);
        srd.ShowDialog (this);
        if   (!srd.StartRecording)
        {
          resetControls = true;
          return;
        }

        sessionParameter    = srd.GetSessionParameters ();
        operatingParameters = srd.GetOperatingParameters ();

        CountingInitiate ();

        cameraManager.RecordButtonPressed (sessionParameter,
                                           operatingParameters,
                                           ref successful,
                                           ref errMsg
                                          );
        if  (!successful)
          MessageBox.Show (this, errMsg, "Camera Recording Failed", MessageBoxButtons.OK);
        else
          StartCameraPreView ();
      }

      resetControls = true;
    }  /* RecordButton_Click */



    private  void  RunPlayBackForScannerFile (String  sourceScannerFileName)
    {
      runLog.WriteLn (10, "RunPlayBackForScannerFile   SourceScannerFileName: " + sourceScannerFileName);

      bool  successful = false;
      UmiSessionParameters    playBackSessionParms   = cameraManager.GetSessionParameters ();
      UmiOperatingParameters  playBackOperatingParms = cameraManager.GetOperatingParameters ();

      String  errMsg = "";
      {
        UmiScannerFile sf = UmiScannerFile.CreateScannerFile (sourceScannerFileName, runLog);
        if  (sf == null)
        {
          runLog.WriteLn (10, "RunPlayBackForScannerFile  Error opening Scanner File");
          MessageBox.Show (this, "Invalid Scanner File Selected.", "Play-Back Scanner File.", MessageBoxButtons.OK);
          return;
        }

        playBackSessionParms.UpdateFromScannerFile (sf);
        playBackOperatingParms.UpdateFromScannerFile (sf);

        sf.Close ();
        sf = null;

        UmiParticleEntryList particles = null;
        try
        {
          particles = new UmiParticleEntryList (sourceScannerFileName, 
                                                true,   // true = Load Header Fields only.
                                                runLog
                                               );
        }
        catch  (Exception)
        {
          particles = null;
        }
        if  (particles != null)
        {
          UmiSessionParameters    particlesSessionParms   = particles.GetSessionParameters   ();
          UmiOperatingParameters  particlesOperatingParms = particles.GetOperatingParameters ();

          if  (particlesSessionParms != null)
            playBackSessionParms.Assign (particlesSessionParms);

          if  (particlesOperatingParms != null)
            playBackOperatingParms.Assign (particlesOperatingParms);
        }
      }

      playBackOperatingParms.DataIsToBeCounted  = true;
      playBackOperatingParms.SaveParticleImages = false;
      playBackOperatingParms.DataIsToBeRecorded = false;
      playBackOperatingParms.FlatFieldEnabled   = true;
      if  (playBackOperatingParms.DestScannerFileFormat == UmiScannerFile.ScannerFileFormat.sfSimple)
        playBackOperatingParms.DestScannerFileFormat = UmiScannerFile.ScannerFileFormat.sf4BitEncoded;

      StartRecordingDialog  srd = new StartRecordingDialog (cameraManager, playBackSessionParms, playBackOperatingParms, true, runLog);
      srd.ShowDialog (this);
      if   (srd.StartRecording)
      {
        playBackSessionParms   = srd.GetSessionParameters   ();
        playBackOperatingParms = srd.GetOperatingParameters ();

        CountingInitiate ();

        cameraManager.PlayBackButtonPressed (sourceScannerFileName,
                                             playBackSessionParms,
                                             playBackOperatingParms,
                                             ref successful,
                                             ref errMsg
                                            );
        if  (!successful)
          MessageBox.Show (this, errMsg, "Play-Back Failed to Start", MessageBoxButtons.OK);
        else
          StartCameraPreView ();
      }

      SetEnableDisable ();
    }  /* RunPlayBackForScannerFile */




    private void LogMsgs_VisibleChanged (object sender, EventArgs e)
    {
      if  (LogMsgs.Visible)
        LogMsgs.PerformLayout ();
    }



    private void SampleBeforeFlatField_CheckedChanged (object sender, EventArgs e)
    {
      runLog.WriteLn (10, "SampleBeforeFlatField_CheckedChanged  " + SampleBeforeFlatField.Checked.ToString ());
      cameraManager.SampleLastFrameBeforeFlatField (SampleBeforeFlatField.Checked);
    }



    private void RequestedScanRate_ValueChanged (object sender, EventArgs e)
    {
      runLog.WriteLn (10, "RequestedScanRate_ValueChanged  " + RequestedScanRate.Value.ToString ());
      cameraManager.RequestedScanRate ((float)RequestedScanRate.Value);
    }




    private void LarcosCounter_SizeChanged (object sender, EventArgs e)
    {
      int  deltaHeight = Height - lastHeight;
      int  deltaWidth  = Width  - lastWidth;

      PlayBackProgress.Width += deltaWidth;
      
      FlowRateLabel.Top          += deltaHeight;
      FlowRate.Top               += deltaHeight;
      CameraTemparature.Top      += deltaHeight;
      CameraTemparatureLabel.Top += deltaHeight;
      LarcosStatus.Top           += deltaHeight;
      LarcosStatus2.Top          += deltaHeight;

      Count.Top                  += deltaHeight;
      CountLabel.Top             += deltaHeight;

      ExitButton.Left            += deltaWidth;

      Count.Left                 += deltaWidth;
      CountLabel.Left            += deltaWidth;

      LarcosStatus.Width         += deltaWidth;
      LarcosStatus2.Width        += deltaWidth;

      TabDisplayed.Height        += deltaHeight;
      TabDisplayed.Width         += deltaWidth;

      liveVideoFrame.Height      += deltaHeight;
      liveVideoFrame.Width       += deltaWidth;

      SampleBeforeFlatField.Top  += deltaHeight;
      SampleBeforeFlatField.Left += deltaWidth;

      LogMsgs.Height += deltaHeight;
      LogMsgs.Width  += deltaWidth;

      ThroughPutDataToDiosplay.Top += deltaHeight;
      SecondaryToDisplay.Top += deltaHeight;
      SecondaryToDisplay.Left += deltaWidth;
      ThroughputChart.Height += deltaHeight;
      ThroughputChart.Width  += deltaWidth;
      ThroughputChart.ChartAreas[0].AxisX.ScaleView.Size = ThroughputChart.Width / 50;

      cameraLine.Height += deltaHeight;
      cameraLine.Width  += deltaWidth;

      lastHeight = Height;
      lastWidth  = Width;

      cameraManager.SetLiveVideoDimensions (liveVideoFrame.Height, liveVideoFrame.Width);
    }  /* LarcosCounter_SizeChanged */



    private void ThroughtPutChartAddSnapShot (StatusSnapshotManaged  snapShot)
    {
      if  (snapShot == null)
        return;

      int  x = ThroughputChart.Series[0].Points.Count;

      frameProcessorsCount = Math.Max (1, cameraManager.FrameProcessorsCount ());

      int  timeOffset = snapShot.TimeOffset;
      int  mins = timeOffset / 60;
      int  secs = timeOffset % 60;

      DataPoint  dp = new DataPoint (x, snapShot.GetDataField (throughPutField));
      dp.AxisLabel = mins.ToString ("00") + ":" + secs.ToString ("00");

      if  (snapShot.LogicalFramesDropped > 0)
        dp.Color = Color.Orange;

      if  (snapShot.PhysicalFramesDropped > 0)
        dp.Color = Color.Red;

      ThroughputChart.Series[0].Points.Add (dp);

      if  (secondaryToDisplay == "Capacity")
      {
        float  capacityPercent = snapShot.AvailableCapacity;

        dp = new DataPoint (x, capacityPercent);
        dp.BorderWidth = 3;
        if  (capacityPercent > 0.8f)
          dp.Color = Color.DarkGreen;
        else if  (capacityPercent > 0.5f)
          dp.Color = Color.Orange;
        else
        {
          dp.Color = Color.Red;
          dp.BorderWidth = 6;
        }
      }
      else
      {
        dp = new DataPoint (x, snapShot.FlowRate);
        dp.BorderWidth = 3;
        dp.Color = Color.DarkGreen;
      }

      ThroughputChart.Series[1].Points.Add (dp);
    }



    private void SamplingIntervalTimer_Tick (object sender, EventArgs e)
    {
      StatusSnapshotManaged  snapShot = cameraManager.SnapshotLastEntry ();
      if  (snapShot == null)
        return;
      ThroughtPutChartAddSnapShot (snapShot);
      ThroughputChart.ChartAreas[0].RecalculateAxesScale ();
    }




    private void ThroughPutDataToDisplay_SelectedIndexChanged (object sender, EventArgs e)
    {
      if  (cameraManager.OperatingMode () != LarcosOperatingModes.Advanced)
      {
        throughPutField = StatusSnapshotManaged.DataFieldIdx.Count;
      }
      else
      {
        StatusSnapshotManaged.DataFieldIdx  newThroughPutField = StatusSnapshotManaged.DataFieldIdxFromStr ((String)ThroughPutDataToDiosplay.SelectedItem);
        if  (newThroughPutField != StatusSnapshotManaged.DataFieldIdx.dfiNULL)
        {
          runLog.WriteLn  (10, "ThroughPutDataToDisplay_SelectedIndexChanged   Changed to " + newThroughPutField + ".");
          
          throughPutField = newThroughPutField;
          cameraManager.ThroughPutField (throughPutField);

          UpdateThrouputChartData ();
        }
      }
    }



    private void  SecondaryToDisplay_SelectedIndexChanged (object sender, EventArgs e)
    {
      if  (!cameraManager.EmbeddedFlowMeter ())
      {
        secondaryToDisplay = "Capacity";
        SecondaryToDisplay.SelectedItem = secondaryToDisplay;
      }
      else
      {
        String newSecondaryToDisplay = (String)SecondaryToDisplay.SelectedItem;
        if  ((newSecondaryToDisplay == "Capacity")  ||  (newSecondaryToDisplay == "Flow Rate"))
        {
          runLog.WriteLn  (10, "SecondaryToDisplay_SelectedIndexChanged   Changed to " + newSecondaryToDisplay + ".");
          secondaryToDisplay = newSecondaryToDisplay;
        }
      }

      UpdateThrouputChartData ();
    }



    private  void  UpdateThrouputChartData ()
    {
      ThroughputChart.Series[0].LegendText = (String)ThroughPutDataToDiosplay.SelectedItem;
      ThroughputChart.Series[0].Points.Clear ();
      ThroughputChart.Series[1].LegendText = secondaryToDisplay;
      ThroughputChart.Series[1].Points.Clear ();
      ThroughputChart.ChartAreas[0].AxisX.ScaleView.Size = ThroughputChart.Width / 50;

      StatusSnapshotManagedList  snapShots = cameraManager.SnapshotsRetrieve (DateTime.MinValue, DateTime.MaxValue);
      if  (snapShots != null)
      {
        foreach  (StatusSnapshotManaged snapShot in snapShots)
          ThroughtPutChartAddSnapShot (snapShot);
      }

      ThroughputChart.ChartAreas[0].RecalculateAxesScale ();
    }




    private void AuditButton_Click (object sender, EventArgs e)
    {
      runLog.WriteLn (10, "AuditButton_Click");
      curState = cameraManager.CurState ();
      if  ((curState != LarcosCounterState.Stopped)   &&  (curState != LarcosCounterState.Connected))
      {
        MessageBox.Show (this, "You must be in the Stopped or Connected mode to Audit and existing scanner file.", "Audit a Scanner File", MessageBoxButtons.OK);
        return;
      }

      DisableAllControls ();

      String  sourceScannerFileName = cameraManager.LastScannerFileCounted ();
      OpenFileDialog fd = new OpenFileDialog ();
      fd.FileName = UmiOSservices.GetRootNameWithExtension (sourceScannerFileName);
      fd.InitialDirectory  = UmiOSservices.GetPathPartOfFile (sourceScannerFileName);
      fd.AddExtension = true;
      fd.DefaultExt = "lsc";
      fd.Filter = "Larcos Files (*.lsc)|*.lsc";

      DialogResult dr = fd.ShowDialog (this);
      if  (dr != DialogResult.OK)
      {
        runLog.WriteLn (10, "AuditButton_Click   No File Selected.");
        SetEnableDisable ();
      }
      else
      {
        sourceScannerFileName = fd.FileName;
        runLog.WriteLn (20, "AuditButton_Click   File Selected: " + sourceScannerFileName);
        Auditor auditor = new Auditor (cameraManager, sourceScannerFileName, runLog);
        auditor.ShowDialog (this);
        if (auditor.RunPlayBack)
        {
          runLog.WriteLn (20, "AuditButton_Click   Uaser choose to RunPlayBack: " + sourceScannerFileName);
          RunPlayBackForScannerFile (sourceScannerFileName);
        }
        else
        {
          SetEnableDisable ();
        }
      }
    }



    private void ExitButton_Click (object sender, EventArgs e)
    {
      runLog.WriteLn (10, "ExitButton_Click" + cameraManager.CurState ());
      bool  stopProcessing = false;

      if  (cameraManager.IsRecording ()  ||  cameraManager.IsRunning ())
      {
        DialogResult dr = MessageBox.Show (this, "Currently Recording Data;  are you sure you want to exit ?", "Exit Requested", MessageBoxButtons.YesNo);
        if  (dr == DialogResult.Yes)
          stopProcessing = true;
        else
          return;
      }

      if  (stopProcessing)
      {
        runLog.WriteLn (10, "ExitButton_Click   User requested to stop processing.");
        SaveConfiguration ();
        bool  successful = false;
        String  errMsg = "";
        cameraManager.StopButtonPressed (true, ref successful, ref errMsg);
        runLogTimerExitApplication = true;
      }

      else
      {
        Close ();
      }
    }



    private void ModifyDefaultParametersButton_Click (object sender, EventArgs e)
    {
      UmiOperatingParameters  defaultOperatingParameters = cameraManager.GetDefaultOperatingParameters ();
      AdvancedParameters  operatingParametersScr = new AdvancedParameters (defaultOperatingParameters);
      DialogResult  dr = operatingParametersScr.ShowDialog (this);
      if  (operatingParametersScr.UserAcceptedScreen)
      {
        defaultOperatingParameters = operatingParametersScr.GetOperatingParameters ();
        cameraManager.SetDefaultOperatingParameters (defaultOperatingParameters);
        UpdateDefaultOperatingParametersTab ();
      }
    }



    private void SnapshotInterval2_ValueChanged (object sender, EventArgs e)
    {
      if  (SnapshotInterval2.Value < 1)
      {
        SnapshotInterval2.Value = 1;
        return;
      }

      if  (SnapshotInterval2.Value > 600)
      {
        SnapshotInterval2.Value = 600;
        return;
      }

      cameraManager.SnapshotInterval ((int)SnapshotInterval2.Value);

      if  ((cameraManager.CurState () == LarcosCounterState.Running)  ||
           (cameraManager.CurState () == LarcosCounterState.PlayingBack)
          )
        UpdateThrouputChartData ();
    }



    private void PlayingBackRealTime_CheckedChanged (object sender, EventArgs e)
    {
      cameraManager.PlayingBackRealTime (PlayingBackRealTime.Checked);
    }
 

  }  /* LarcosCounter */

}  /* LarcosCounter Name Space  */

