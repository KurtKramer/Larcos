#if  !defined(_STATUSSNAPSHOTMANAGED_)
#define  _STATUSSNAPSHOTMANAGED_

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::Diagnostics;
using namespace System::Drawing;


#include "CameraParameters.h"
#include "StatusSnapshot.h"
using namespace  LarcosCounterUnManaged;

#include  "CameraParametersManaged.h"

namespace LarcosCounterManaged
{
  public ref class StatusSnapshotManaged
  {
  public:
    StatusSnapshotManaged (const StatusSnapshot&  stats);

    !StatusSnapshotManaged ();

    ~StatusSnapshotManaged ();

    enum  class  DataFieldIdx {dfiNULL                         = StatusSnapshot::dfiNULL,
                               LogicalFrameProcessorsAvailable = StatusSnapshot::dfiLogicalFrameProcessorsAvailable,
                               LogicalFramesOnQueue            = StatusSnapshot::dfiLogicalFramesOnQueue,
                               Count                           = StatusSnapshot::dfiCount,
                               Particles                       = StatusSnapshot::dfiParticles,
                               CpuUsage                        = StatusSnapshot::dfiCpuUsage,
                               AvailableCapacity               = StatusSnapshot::dfiAvailableCapacity,
                               PhysicalFramesDropped           = StatusSnapshot::dfiPhysicalFramesDropped,
                               LogicalFramesDropped            = StatusSnapshot::dfiLogicalFramesDropped,
                               ScanLinesRead                   = StatusSnapshot::dfiScanLinesRead,
                               ScanLinesWritten                = StatusSnapshot::dfiScanLinesWritten,
                               ParticlesWaitingProcessing      = StatusSnapshot::dfiParticlesWaitingProcessing,
                               FlowRate                        = StatusSnapshot::dfiFlowRate,
                               Invalid                         = StatusSnapshot::dfiInvalid
                              };

    property  System::DateTime  TimeStamp            {System::DateTime  get ()  {return timeStamp;}}
    property  kkint32  TimeOffset                      {kkint32  get ()  {return timeOffset;}}
    property  float  LogicalFrameProcessorsAvailable {float  get ()  {return logicalFrameProcessorsAvailable;};}  /**< Number of Fra,eProcessors that are sleeping until available Frame to process. */
    property  float  LogicalFramesOnQueue            {float  get ()  {return logicalFramesOnQueue;};}
    property  kkint32  Count                           {kkint32  get ()  {return count;}}
    property  float  FlowRate                        {float  get ()  {return flowRate;}}
    property  kkint32  Particles                       {kkint32  get ()  {return particles;}}
    property  float  CpuUsage                        {float  get ()  {return cpuUsage;}}
    property  float  AvailableCapacity               {float  get ()  {return availableCapacity;}}
    property  kkint32  ParticlesWaitingProcessing      {kkint32  get ()  {return particlesWaitingProcessing;};}
    property  kkint32  PhysicalFramesDropped           {kkint32  get ()  {return physicalFramesDropped;};}
    property  kkint32  LogicalFramesDropped            {kkint32  get ()  {return logicalFramesDropped;};}         
    property  kkint32  ScanLinesRead                   {kkint32  get ()  {return scanLinesRead;};}
    property  kkint32  ScanLinesWritten                {kkint32  get ()  {return scanLinesWritten;};}

    static
      System::DateTime   DateTimeKKBtoSystem (const  KKB::DateTime&  date);

    static
      KKB::DateTime   DateTimeSystemToKKB (System::DateTime  date);

    float   GetDataField (DataFieldIdx  dataField);

    static  DataFieldIdx  DataFieldIdxFromStr (String^ s);

    static  String^  DataFieldIdxToStr (StatusSnapshotManaged::DataFieldIdx idx);

    static  array<String^>^  DataFieldIdxNames ();

  private:
    void  CleanUpMemory ();

    System::DateTime  timeStamp;                 /**< Time that snapshot was taken.          */
    kkint32   timeOffset;
    float     logicalFrameProcessorsAvailable;
    float     logicalFramesOnQueue;
    kkint32   count;                             /**< Shrimp counted this interval.          */
    kkint32   particles;                         /**< Particles counted       this interval. */
    float     cpuUsage;
    float     availableCapacity;                 /**< Fraction that reflects available processing capacity. */
    kkint32   physicalFramesDropped;             /**< Physical Frames dropped this interval. */
    kkint32   logicalFramesDropped;              /**< Logical Frames dropped  this interval. */
    kkint32   scanLinesRead;                     /**< Scan-Lines read         this interval. */
    kkint32   scanLinesWritten;                  /**< Scan-Lines written      this interval. */
    kkint32   particlesWaitingProcessing;        /**< Number of particles identified by FrameProcessors but have not yet been processed. */
    float     flowRate;
  };  /* StatusSnapshotManaged */





  public ref class StatusSnapshotManagedList: List<StatusSnapshotManaged^>
  {
  public:
    StatusSnapshotManagedList ();

    StatusSnapshotManagedList (const StatusSnapshotList&  snapshots);


  private:
    ~StatusSnapshotManagedList ();

  protected:
    !StatusSnapshotManagedList ();

  };  /* StatusSnapshotManagedList */

}  /* LarcosCounterManaged */

#endif