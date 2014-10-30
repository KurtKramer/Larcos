/* CameraAcquisition.cpp -- Base class for the Acquisition threads.
 * Copyright (C) 2011-2013  Kurt Kramer
 * For conditions of distribution and use, see copyright notice in LarcosCounterUnManaged.txt
 */
#include "FirstIncludes.h"

#include <errno.h>
#include <istream>
#include <iostream>
#include <queue>
#include <vector>
#if  defined(WIN32)
#include <windows.h>
#endif
#include "MemoryDebug.h"
using namespace std;


#include "KKBaseTypes.h"
#include "GlobalGoalKeeper.h"
#include "Raster.h"
#include "RunLog.h"
using namespace KKB;


#include "FactoryFVProducer.h"
#include "FeatureVectorProducer.h"
using namespace  KKMachineLearning;

#include "PostLarvaeFVProducer.h"

#include "FeatureFileIOKK.h"
#include "PostLarvaeFV.h"
using  namespace  LarcosCounterUnManaged;



PostLarvaeFVProducer::PostLarvaeFVProducer (FactoryFVProducerPtr  factory):
    FeatureVectorProducer ("PostLarvaeFV",
                           factory,
                           PostLarvaeFV::PostLarvaeFeaturesFileDesc ()
                          ),
    totPixsForMorphOps (100000)
{
}




PostLarvaeFVProducer::~PostLarvaeFVProducer ()
{
}



PostLarvaeFVPtr  PostLarvaeFVProducer::ComputeFeatureVector (Raster&           image,
                                                             const MLClassPtr  knownClass,
                                                             RasterListPtr     intermediateImages,
                                                             RunLog&           runLog
                                                            )
{
  return new  PostLarvaeFV (image, knownClass, intermediateImages);
}




const type_info*   PostLarvaeFVProducer::FeatureVectorTypeId () const
{
  return  &(typeid (PostLarvaeFV));
}





PostLarvaeFVProducerFactory::PostLarvaeFVProducerFactory ():
   FactoryFVProducer("PostLarvaeFVProducerFactory", "PostLarvaeFV", "Post-Larvae FeatureVector Producer Factory")
{
}



PostLarvaeFVProducerFactory::~PostLarvaeFVProducerFactory ()
{
}



FeatureFileIOPtr  PostLarvaeFVProducerFactory::DefaultFeatureFileIO ()  const
{
  return  FeatureFileIOKK::Driver ();
}



FileDescPtr  PostLarvaeFVProducerFactory::FileDesc ()  const
{
  return  PostLarvaeFV::PostLarvaeFeaturesFileDesc ();
}




PostLarvaeFVProducerPtr  PostLarvaeFVProducerFactory::ManufactureInstance (RunLog&  runLog)
{
  return new PostLarvaeFVProducer (this);
}




PostLarvaeFVListPtr  PostLarvaeFVProducerFactory::ManufacturFeatureVectorList (bool     owner,
                                                                               RunLog&  runLog
                                                                              )
{
  return new PostLarvaeFVList (PostLarvaeFV::PostLarvaeFeaturesFileDesc (), owner, runLog);
}



PostLarvaeFVProducerFactory*  PostLarvaeFVProducerFactory::factory = Factory (NULL);




PostLarvaeFVProducerFactory*  PostLarvaeFVProducerFactory::Factory (RunLog*  runLog)
{
  GlobalGoalKeeper::StartBlock ();

  if  (factory == NULL)
  {
    factory = new PostLarvaeFVProducerFactory ();
    FactoryFVProducer::RegisterFactory (factory, runLog);
  }
  GlobalGoalKeeper::EndBlock ();

  return  factory;
}

