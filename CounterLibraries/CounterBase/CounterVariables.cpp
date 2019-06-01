#include "FirstIncludes.h"
#include <stdlib.h>
#include <memory>
#include "MemoryDebug.h"
using namespace  std;

#include "KKBaseTypes.h"
#include "OSservices.h"
using namespace  KKB;

#include "Variables.h"
using namespace  KKLSC;

#include "KKMLVariables.h"
using namespace  KKMLL;

#include "CounterVariables.h"
using namespace  CounterBase;



CounterVariables::CounterVariables ()
{
}



CounterVariables::~CounterVariables ()
{
}



KKStr  CounterVariables::counterHomeDir = "";



kkint32  CounterVariables::debugLevel = -1;



KKStr  CounterVariables::CounterHomeDir ()
{
  if  (!counterHomeDir.Empty ())
    return  counterHomeDir;

  // We will first try and get directory from the CounterHomeDir environment variable.
  KKStrPtr homeDir = osGetEnvVariable ("ZoisHomeDir");
  if  (homeDir != NULL)
  {
    counterHomeDir = *homeDir;
    delete  homeDir;
    homeDir = NULL;
  }

  if  (counterHomeDir.Empty ())
  {
    #if  defined(KKOS_WINDOWS)
       counterHomeDir = "C:\\zois";
       cout << endl << "ZoisHomeDir" << "\t" << counterHomeDir << endl << endl;
    #else
       counterHomeDir = "/Larcos";
    #endif
  }

  return  counterHomeDir; 
}  /* CounterHomeDir */



KKStr  CounterVariables::ScannerFilesDefaultDir ()
{
  return osAddSlash (CounterHomeDir ()) + "ScannerFiles";
}  /* ScannerFilesDefaultDir */



KKStr  CounterVariables::TempDir ()
{
  return  osAddSlash (CounterHomeDir ()) + "Temp";
}



void   CounterVariables::SetCounterHomeDir ()
{
  // When this method is called the caller wants to set the Larcos home directory to the default location.
  // so we set 'counterHomeDir' to a empty string and then call the method 'CounterHomeDir' which will initialize
  // 'counterHomeDir' to the default directory.
  counterHomeDir = "";
  CounterHomeDir ();
  KKMLVariables::SetMachineLearninigHomeDir (osAddSlash (CounterHomeDir ()) + "Classifier");
  KKLSC::Variables::SetHomeDir (CounterHomeDir ());
}



void   CounterVariables::SetCounterHomeDir (const KKStr&  _counterHomeDir)
{
  counterHomeDir = _counterHomeDir;
  KKMLVariables::SetMachineLearninigHomeDir (osAddSlash (CounterHomeDir ()) + "Classifier");
  KKLSC::Variables::SetHomeDir (CounterHomeDir ());
}



/* Directory where logging files can be written to. */
KKStr  CounterVariables::LoggingDir ()
{
  return osAddSlash (CounterHomeDir ()) + "LoggingFiles";
}



KKStr  CounterVariables::CmdLineProgDir ()
{
  KKStr  s = osAddSlash (CounterHomeDir ()) + "exe";
  return s;
}



KKStr  CounterVariables::ConfigurationDir ()
{
  KKStr  configDir = osAddSlash (osAddSlash (CounterHomeDir ()) + "Configurations");
  return  configDir;
}



kkint32  CounterVariables::DebugLevel ()
{
  if  (debugLevel < 0)
  {
    KKStrPtr s = osGetEnvVariable ("LarcosDebugLevel");
    if  (s)
    {
      debugLevel = s->ToInt32 ();
      delete  s;
      s = NULL;
    }
    else
    {
      debugLevel  = 10;
    }
  }
  return  debugLevel;
}



KKStr  CounterVariables::DataFilesRootDir ()
{
  KKStr  dataFilesRootDir = osAddSlash (osAddSlash (CounterHomeDir ()) + "Configurations");
  return  dataFilesRootDir;
}



KKStr  CounterVariables::ReportDir ()
{
  KKStr  reportDir = osAddSlash (osAddSlash (CounterHomeDir ()) + "Reports");
  return  reportDir;
}



KKStr  CounterVariables::TrainingLibrariesDir ()
{
  return  KKMLVariables::TrainingLibrariesDir ();
}



KKStr  CounterVariables::TrainingModelsDir ()
{
  return  KKMLVariables::TrainingModelsDir ();
}
