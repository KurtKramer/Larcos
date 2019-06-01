#include "StdAfx.h"
#include "FirstIncludes.h"
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <string>
#include <vector>
#include "MemoryDebug.h"


#include "KKBaseTypes.h"
#include "..\\KKBase\\GoalKeeper.h"
#include "OSservices.h"
using namespace  KKB;


#include "UmiOSservices.h"
#include "UmiKKStr.h"
#include "CounterTrainingModelManaged.h"
using namespace  CounterManaged;



CounterTrainingModelManaged::CounterTrainingModelManaged (CounterTrainingModelPtr  _counterTrainingModel):
   counterTrainingModel (_counterTrainingModel)
 {
   int zed = 100;
 }



 String^  CounterTrainingModelManaged::Subject::get ()
 {
   if  (counterTrainingModel)
     return  UmiKKStr::KKStrToSystenStr (counterTrainingModel->Subject ());
   else
     return  "";
 }



 String^  CounterTrainingModelManaged::Size::get ()
 {
   if  (counterTrainingModel)
     return  UmiKKStr::KKStrToSystenStr (counterTrainingModel->Size ());
   else
     return  "";
 }



 String^  CounterTrainingModelManaged::WaterQuality::get ()
 {
   if  (counterTrainingModel)
     return  UmiKKStr::KKStrToSystenStr (counterTrainingModel->WaterQuality ());
   else
     return  "";
 }



 String^  CounterTrainingModelManaged::TrainingModelName::get ()
 {
   if  (counterTrainingModel)
     return  UmiKKStr::KKStrToSystenStr (counterTrainingModel->TrainingModelName ());
   else
     return  "";
 }


 
CounterTrainingModelPtr  CounterTrainingModelManaged::UnManaged ()
{
  return counterTrainingModel;
}



CounterTrainingModelListManaged::CounterTrainingModelListManaged () :
  counterTrainingModels (new CounterTrainingModelList (true))
{
}



CounterTrainingModelListManaged::CounterTrainingModelListManaged (CounterTrainingModelListPtr  _counterTrainingModels):
   counterTrainingModels (_counterTrainingModels)
{
}



CounterTrainingModelListManaged::!CounterTrainingModelListManaged ()
{
  delete  counterTrainingModels;
  counterTrainingModels = NULL;
}



CounterTrainingModelListManaged::~CounterTrainingModelListManaged ()
{
  this->!CounterTrainingModelListManaged ();
}



CounterTrainingModelListManaged^  CounterTrainingModelListManaged::BuildFromTrainigModelDir ()
{
  CounterTrainingModelListPtr models = CounterTrainingModelList::BuildFromTrainigModelDir ();
  CounterTrainingModelListManaged^  results = gcnew CounterTrainingModelListManaged (models);
  // 'results' now owns 'models' so we do not need to delete it.
  models = NULL;
  return  results;
}



array<String^>^  CounterTrainingModelListManaged::GetListOfSubjects ()
{
  if (counterTrainingModels == NULL)
  {
    return nullptr;
  }

  KKStrListPtr  subjects = counterTrainingModels->GetListOfSubjects ();
  array<String^>^  managedSubjects =  UmiKKStr::KKStrToListToStringArray (subjects);
  delete  subjects;
  subjects = NULL;
  return  managedSubjects;
}



array<String^>^  CounterTrainingModelListManaged::GetListOfSizes (String^ subject)
{
  KKStrListPtr  sizes = counterTrainingModels->GetListOfSizes (UmiKKStr::SystemStringToKKStr (subject));
  array<String^>^  managedSizes =  UmiKKStr::KKStrToListToStringArray (sizes);
  delete  sizes;
  sizes = NULL;
  return  managedSizes;
}



array<String^>^  CounterTrainingModelListManaged::GetListOfWaterQualities (String^ subject, String^ size)
{
  KKStrListPtr  waterQualities = counterTrainingModels->GetListOfWaterQualities (UmiKKStr::SystemStringToKKStr (subject), UmiKKStr::SystemStringToKKStr (size));
  array<String^>^  managedWaterQualities =  UmiKKStr::KKStrToListToStringArray (waterQualities);
  delete  waterQualities;
  waterQualities = NULL;
  return  managedWaterQualities;
}



bool  CounterTrainingModelListManaged::InList (String^ subject, String^ size, String^ waterQuality)
{
  return  counterTrainingModels->InList (UmiKKStr::SystemStringToKKStr (subject), 
                                         UmiKKStr::SystemStringToKKStr (size),
                                         UmiKKStr::SystemStringToKKStr (waterQuality)
                                        );
}
