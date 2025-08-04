#include "WaveformReader.h"


/**
 * Displays data about the various streams our port driver is connected to for health-checking purposes
 */
void WaveformReader::statusCheck(void)
{

  for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
  {
    std::string paramIndex = waveform_param_indices[i];
    // extract initialization times in the form of strings
    std::time_t init_time_t = std::chrono::system_clock::to_time_t(initialization_times[i]);
    std::string init_time_str = std::ctime(&init_time_t);
    init_time_str = init_time_str.substr(0, (init_time_str.length() - 1));

    // extract retrieval times in the form of strings
    std::time_t real_time_t = std::chrono::system_clock::to_time_t(retrieval_times[i]);
    std::string real_time_str = std::ctime(&real_time_t);
    real_time_str = real_time_str.substr(0, (real_time_str.length() - 1));

    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "|                                  " << paramIndex << "                                   |" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "| " << std::left << std::setw(25) << "Streaming Status" << " | " << std::right << std::setw(50) << streaming_status[i] << "|" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "| " << std::left << std::setw(25) << "Initialized on" << " | " << std::right << std::setw(50) << ((initialization_status[i] == true)? init_time_str : "N/A")  << "|" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "| " << std::left << std::setw(25) << "Duration (ms)" << " | " << std::right << std::setw(50) << (duration_data[i]).count() << "|" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "| " << std::left << std::setw(25) << "Last retrieved on" << " | " << std::right << std::setw(50) << ((initialization_status[i] == true)? real_time_str : "N/A") << "|" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "| " << std::left << std::setw(25) << "Thread Status" << " | " << std::right << std::setw(50) << thread_status[i] << "|" << std::endl;
    std::cout << "|-------------------------------------------------------------------------------|" << std::endl;
    std::cout << "\n" << std::endl;
  } 
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void waveformStatus(void)
{
  WaveformReader* bayManager = WaveformReader::getPortDriver();
  bayManager->statusCheck();
  return;
}
static const iocshFuncDef statusFuncDef = {"waveformStatus", 0};
static void statusCallFunc(const iocshArgBuf *args)
{
  waveformStatus();
}

void waveformStatusRegister(void)
{
  iocshRegister(&statusFuncDef, statusCallFunc);
}

extern "C" {
  epicsExportRegistrar(waveformStatusRegister);
}