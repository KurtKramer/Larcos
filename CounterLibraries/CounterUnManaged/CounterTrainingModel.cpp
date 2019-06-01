/* CounterTrainingModel.cpp -- Represents a single camera frame.
 * Copyright (C) 2011-2013  Kurt Kramer
 * For conditions of distribution and use, see copyright notice in CounterUnManaged.txt
 */
#include "FirstIncludes.h"

#include <errno.h>
#include <istream>
#include <iostream>
#include <vector>
#include "MemoryDebug.h"
using namespace std;


#include "KKBaseTypes.h"
#include "OSservices.h"
using namespace KKB;

#include "KKMLVariables.h"
using namespace KKMLL;


#include "CounterTrainingModel.h"
using  namespace  CounterUnManaged;



CounterTrainingModel::CounterTrainingModel ():
   subject           (),
   size              (),
   waterQuality      (),
   trainingModelName ()
{
}



CounterTrainingModel::CounterTrainingModel (const CounterTrainingModel&  _entry):

    subject           (_entry.subject),
    size              (_entry.size),
    waterQuality      (_entry.waterQuality),
    trainingModelName (_entry.trainingModelName)
{
}



CounterTrainingModel::CounterTrainingModel (const KKStr&  _subject,
                                          const KKStr&  _size,
                                          const KKStr&  _waterQuality,
                                          const KKStr&  _trainingModelName
                                         ):
    subject           (_subject),
    size              (_size),
    waterQuality      (_waterQuality),
    trainingModelName (_trainingModelName)
{
}



CounterTrainingModel::~CounterTrainingModel ()
{
}



kkMemSize  CounterTrainingModel::MemoryConsumedEstimated ()  const
{
  return  sizeof (*this) + 
          subject.MemoryConsumedEstimated           () +
          size.MemoryConsumedEstimated              () +
          waterQuality.MemoryConsumedEstimated      () +
          trainingModelName.MemoryConsumedEstimated ();
}



CounterTrainingModelList::CounterTrainingModelList (bool _owner):
   KKQueue<CounterTrainingModel> (_owner)
{
}



CounterTrainingModelList::CounterTrainingModelList (const CounterTrainingModelList&  _list):
   KKQueue<CounterTrainingModel> (_list)
{
}



CounterTrainingModelList::~CounterTrainingModelList ()
{
}



CounterTrainingModelListPtr  CounterTrainingModelList::BuildFromTrainigModelDir ()
{
  CounterTrainingModelListPtr  modelList = new CounterTrainingModelList (true);

  KKStr  trainModelDir = KKMLVariables::TrainingModelsDir ();
  KKStrListPtr  modelFileNames = osGetListOfFiles (osAddSlash (trainModelDir) + "*.cfg");
  if (modelFileNames != NULL)
  {
    KKStrList::const_iterator idx;
    for (idx = modelFileNames->begin (); idx != modelFileNames->end (); ++idx)
    {
      KKStrPtr name = *idx;

      KKStr  fileRootName = osGetRootName (*name);
      if (fileRootName.InstancesOfChar ('_') != 2)
        continue;

      KKStr  subject = fileRootName.ExtractToken2 ("_");
      KKStr  size = fileRootName.ExtractToken2 ("_");
      KKStr  waterQuality = fileRootName.ExtractToken2 ("_");

      modelList->PushOnBack (new CounterTrainingModel (subject, size, waterQuality, *name));
    }
  }

  delete  modelFileNames;
  modelFileNames = NULL;
  return  modelList;
}  /* BuildFromTrainigModelDir */



KKStrListPtr  CounterTrainingModelList::GetListOfSubjects ()
{
  map<KKStr,KKStr>  uniqueSubjects;
  map<KKStr,KKStr>::const_iterator  uniqueSubjectsIdx;

  KKStrListPtr uniqueSubjectsList = new KKStrList (true);

  CounterTrainingModelList::const_iterator  idx;
  for  (idx = begin ();  idx != end ();  ++idx)
  {
    CounterTrainingModelPtr ltm = *idx;
    KKStr  subjectLower = ltm->Subject ().ToLower ();
    uniqueSubjectsIdx = uniqueSubjects.find (subjectLower);
    if  (uniqueSubjectsIdx == uniqueSubjects.end ())
    {
      uniqueSubjectsList->PushOnBack (new KKStr (ltm->Subject ()));
      uniqueSubjects.insert (pair<KKStr,KKStr> (subjectLower, ltm->Subject ()));
    }
  }

  return  uniqueSubjectsList;
}  /* GetListOfSubjects */



KKStrListPtr  CounterTrainingModelList::GetListOfSizes (const KKStr&  subject)
{
  map<KKStr,KKStr>  uniqueSizes;
  map<KKStr,KKStr>::const_iterator  uniqueSizesIdx;

  KKStrListPtr uniqueSizesList = new KKStrList (true);

  CounterTrainingModelList::const_iterator  idx;
  for  (idx = begin ();  idx != end ();  ++idx)
  {
    CounterTrainingModelPtr ltm = *idx;
    if  (subject.EqualIgnoreCase (ltm->Subject ()))
    {
      KKStr  sizeLower = ltm->Size ().ToLower ();
      uniqueSizesIdx = uniqueSizes.find (sizeLower);
      if  (uniqueSizesIdx == uniqueSizes.end ())
      {
        uniqueSizesList->PushOnBack (new KKStr (ltm->Size ()));
        uniqueSizes.insert (pair<KKStr,KKStr> (sizeLower, ltm->Size ()));
      }
    }
  }

  return  uniqueSizesList;
}  /* GetListOfSizes */



KKStrListPtr  CounterTrainingModelList::GetListOfWaterQualities (const KKStr&  subject, const KKStr&  size)
{
  KKStrListPtr waterQualityList = new KKStrList (true);

  CounterTrainingModelList::const_iterator  idx;
  for  (idx = begin ();  idx != end ();  ++idx)
  {
    CounterTrainingModelPtr ltm = *idx;
    if  (subject.EqualIgnoreCase (ltm->Subject ())  &&   size.EqualIgnoreCase (ltm->Size ()))
      waterQualityList->PushOnBack (new KKStr (ltm->WaterQuality ()));
  }

  return  waterQualityList;
}  /* GetListOfWaterQualities */



bool  CounterTrainingModelList::InList (const KKStr&  subject, 
                                       const KKStr&  size,  
                                       const KKStr&  waterQuality
                                      )
{
  CounterTrainingModelList::const_iterator  idx;
  for  (idx = begin ();  idx != end ();  ++idx)
  {
    CounterTrainingModelPtr ltm = *idx;
    if  (subject.EqualIgnoreCase      (ltm->Subject      ())  &&   
         size.EqualIgnoreCase         (ltm->Size         ())  &&
         waterQuality.EqualIgnoreCase (ltm->WaterQuality ())
        )
      return  true;
  }

  return  false;
}
