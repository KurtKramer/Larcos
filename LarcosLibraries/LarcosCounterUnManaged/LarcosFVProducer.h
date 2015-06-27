#if  !defined(_LARCOSFVPRODUCER_)
#define  _LARCOSFVPRODUCER_

#include "KKBaseTypes.h"
#include "Raster.h"
#include "RunLog.h"
using namespace  KKB;

#include "FactoryFVProducer.h"
#include "FeatureVector.h"
#include "FeatureVectorProducer.h"
using namespace  KKMLL;

#include "LarcosFeatureVector.h"


#define _LarcosFVProducer_VersionNum_  336

namespace LarcosCounterUnManaged
{

  class LarcosFVProducer:  public FeatureVectorProducer
  {
  public:
    LarcosFVProducer (FactoryFVProducerPtr  factory);

    virtual ~LarcosFVProducer ();


    virtual  LarcosFeatureVectorPtr  ComputeFeatureVector (const Raster&     srcImage,
                                                           const MLClassPtr  knownClass,
                                                           RasterListPtr     intermediateImages,
                                                           float             priorReductionFactor,
                                                           RunLog&           runLog
                                                          );


    static FileDescPtr  DefineFileDescStatic ();

    /**
     *@brief  Returns the 'type_info' of the Feature Vector that this instance of 'FeatureComputer' creates.
     */
    virtual  const type_info*  FeatureVectorTypeId () const;

    virtual  const type_info*  FeatureVectorListTypeId () const;


    virtual  FeatureVectorListPtr  ManufacturFeatureVectorList (bool     owner,
                                                                RunLog&  runLog
                                                               );

    virtual  kkint16  Version ()  const {return _LarcosFVProducer_VersionNum_;}

    static   kkint32  MaxNumOfFeatures  ()  {return maxNumOfFeatures;}

  protected:
    virtual  FileDescPtr  DefineFileDesc ()  const;


  private:
    void  BinarizeImageByThreshold (uchar          lower,
                                    uchar          upper,
                                    const Raster&  src,
                                    Raster&        dest
                                   );

    void  ReductionByMultiple (kkint32        multiple,
                               const Raster&  srcRaster,
                               Raster&        destRaster
                              );

    void  SaveIntermediateImage (const Raster&  raster, 
                                 const KKStr&   desc,
                                 RasterListPtr  intermediateImages
                                );



    uchar*   workRaster1Area;
    uchar*   workRaster2Area;
    uchar*   workRaster3Area;
    uchar**  workRaster1Rows;
    uchar**  workRaster2Rows;
    uchar**  workRaster3Rows;


    kkint32  totPixsForMorphOps;  /**<  When this instance is created this is the amount of memory each 
                                   * raster work area will be restricted to.  The Height and width will 
                                   * be adjusted such that the resultant dimensions will fit within this
                                   * constraint.
                                   */

    static  kkint16  maxNumOfFeatures;
    static  const    kkint32  SizeThreshold;

    static  const    KKStr  featureNames[];

    static  kkint16  SizeIndex;                   // 0;
    static  kkint16  Moment1Index;                // 1;
    static  kkint16  Moment2Index;                // 2;
    static  kkint16  Moment3Index;                // 3;
    static  kkint16  Moment4Index;                // 4;
    static  kkint16  Moment5Index;                // 5;
    static  kkint16  Moment6Index;                // 6;
    static  kkint16  Moment7Index;                // 7;
    static  kkint16  Moment8Index;                // 8;

    static  kkint16  EdgeSizeIndex;               // 9;
    static  kkint16  EdgeMoment1Index;            // 10;
    static  kkint16  EdgeMoment2Index;            // 11;
    static  kkint16  EdgeMoment3Index;            // 12;
    static  kkint16  EdgeMoment4Index;            // 13;
    static  kkint16  EdgeMoment5Index;            // 14;
    static  kkint16  EdgeMoment6Index;            // 15;
    static  kkint16  EdgeMoment7Index;            // 16;
    static  kkint16  EdgeMoment8Index;            // 17;

    static  kkint16  TransparancyConvexHullIndex; // 18;
    static  kkint16  TransparancyPixelCountIndex; // 19;
    static  kkint16  TransparancyOpen3Index;      // 20;
    static  kkint16  TransparancyOpen5Index;      // 21;
    static  kkint16  TransparancyOpen7Index;      // 22;
    static  kkint16  TransparancyOpen9Index;      // 23;
    static  kkint16  TransparancyClose3Index;     // 24;
    static  kkint16  TransparancyClose5Index;     // 25;
    static  kkint16  TransparancyClose7Index;     // 26;

    static  kkint16  ConvexAreaIndex;             // 27
    static  kkint16  TransparancySizeIndex;       // 28
    static  kkint16  TransparancyWtdIndex;        // 29

    static  kkint16  WeighedMoment0Index;         // 30
    static  kkint16  WeighedMoment1Index;         // 31
    static  kkint16  WeighedMoment2Index;         // 32
    static  kkint16  WeighedMoment3Index;         // 33
    static  kkint16  WeighedMoment4Index;         // 34
    static  kkint16  WeighedMoment5Index;         // 35
    static  kkint16  WeighedMoment6Index;         // 36
    static  kkint16  WeighedMoment7Index;         // 37
    static  kkint16  WeighedMoment8Index;         // 38

    static  kkint16  IntensityHist1Index;         // 39
    static  kkint16  IntensityHist2Index;         // 40
    static  kkint16  IntensityHist3Index;         // 41
    static  kkint16  IntensityHist4Index;         // 42
    static  kkint16  IntensityHist5Index;         // 43
    static  kkint16  IntensityHist6Index;         // 44
    static  kkint16  IntensityHist7Index;         // 45
    static  kkint16  DarkSpotCount0;              // 46
    static  kkint16  DarkSpotCount1;              // 47
    static  kkint16  DarkSpotCount2;              // 48
    static  kkint16  DarkSpotCount3;              // 49
    static  kkint16  DarkSpotCount4;              // 50
    static  kkint16  DarkSpotCount5;              // 51
    static  kkint16  DarkSpotCount6;              // 52
    static  kkint16  DarkSpotCount7;              // 53
    static  kkint16  DarkSpotCount8;              // 54
    static  kkint16  DarkSpotCount9;              // 55

    static FileDescPtr  existingFileDesc;


  };  /* LarcosFVProducer */

  typedef  LarcosFVProducer*  LarcosFVProducerPtr;

#define  _LarcosFVProducer_Defined_



  class  LarcosFVProducerFactory:  public  FactoryFVProducer
  {
    typedef  LarcosFVProducerFactory*  LarcosFVProducerFactoryPtr;

    LarcosFVProducerFactory ();

  protected:
    /**
     *@brief  A Factory can never be deleted until the application terminates; the atexit method will perform the deletes.
     */
    virtual ~LarcosFVProducerFactory ();

  public:

    virtual  FeatureFileIOPtr  DefaultFeatureFileIO ()  const;

    virtual  const type_info*  FeatureVectorTypeId ()  const;

    virtual  const type_info*  FeatureVectorListTypeId ()  const;

    virtual  FileDescPtr  FileDesc ()  const;

    virtual  LarcosFVProducerPtr  ManufactureInstance (RunLog&  runLog);


    /**
     *@brief Manufactures a instance of a 'LarcosFeatureVectorList' class that will own its contents.
     */
    virtual  LarcosFeatureVectorListPtr  ManufacturFeatureVectorList (bool     owner,
                                                                      RunLog&  runLog
                                                                     );

    /**
     *@brief  Returns instance of "LarcosFVProducerFactory"  that is registered with "FactoryFVProducer::RegisterFactory".
     *@details If instance does not exist yet out will create an instance and register it.
     */
    static  LarcosFVProducerFactory*  Factory (RunLog*  runLog);


  private:
    /**
     *@brief  This instance of 'LarcosFVProducerFactory'  will be registered with "FactoryFVProducer::RegisterFactory".
     *@details  It will deleted by the "FactoryFVProducer::FinaleCleanUp" upon application shutdown.
     */
    static  LarcosFVProducerFactory*  factory;

  };  /* LarcosFVProducerFactory */

#define  _LarcosFVProducerFactory_Defined_


}  /* LarcosCounterUnManaged */


#endif
