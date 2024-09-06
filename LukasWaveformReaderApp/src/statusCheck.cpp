#include "WaveformReader.h"

//TODO Make this actually useful, tell us what streams are currently connected and meant to be streaming, if they're doing that successfully or not
/**
 * Check the health of any stream our port driver is connected to
 */
void WaveformReader::statusCheck(void)
{
  std::cout << "Status of connected waveforms: " << std::endl;
  for(std::string paramIndex:waveform_param_indices)
  {
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "| " << std::setw(15) << paramIndex << " | " << std::setw(50) << streaming_status_map[paramIndex] << " |" << std::endl;
  }
  std::cout << "------------------------------------------------------------------------" << std::endl;
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void waveformStatus(void)
{
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