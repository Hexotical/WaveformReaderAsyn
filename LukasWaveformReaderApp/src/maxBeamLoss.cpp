#include "WaveformReader.h"

/**
 * Computes and displays the location of the maximum beam loss detected by the monitor
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 */
void WaveformReader::maxBeamLoss(int waveformIndex)
{
  double threshold;
  getDoubleParam(*(threshold_indices[waveformIndex]), &threshold);

  std::string waveform_pvIdentifier = extracted_waveform_param_indices[waveformIndex];
  std::string x_axis_pvIdentifier = extracted_x_axis_waveform_indices[waveformIndex];
  int maxIndex = findMaxIndex(waveformIndex);
  double locationOfMaxIndex = extracted_x_axis_waveform_map[x_axis_pvIdentifier][maxIndex];

  int maxLoss = extracted_waveform_map[waveform_pvIdentifier][maxIndex];

  if (maxLoss < threshold)
  {
    std::cout << "No maximum beam loss was discovered, the data is mostly noise" << std::endl;
    maxLoss = 0;
  }

  else 
  {
    std::cout << "The location of maximum beam loss is " << locationOfMaxIndex << std::endl;
    std::cout << "The value of maximum beam loss is " <<  maxLoss << std::endl;
    setDoubleParam(*(beam_loss_loc_indices[waveformIndex]), locationOfMaxIndex);
    callParamCallbacks();
  }
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void maxBeamLossLocation(int waveformIndex) {
  WaveformReader* bayManager = WaveformReader::getPortDriver();
  bayManager->maxBeamLoss(waveformIndex);
  return;
}

static const iocshArg lossArg0 = {"waveformIndex", iocshArgInt};
static const iocshArg * const lossArgs[] = {&lossArg0};
static const iocshFuncDef lossFuncDef = {"maxBeamLossLocation", 1, lossArgs};
static void lossCallFunc(const iocshArgBuf *args)
{
  maxBeamLossLocation(args[0].ival);
}
void maxBeamLossLocationRegister(void)
{
  iocshRegister(&lossFuncDef, lossCallFunc);
}

extern "C" {
  epicsExportRegistrar(maxBeamLossLocationRegister);
}