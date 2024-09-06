#include "WaveformReader.h"

/**
 * Computes and displays the location of the maximum beam loss detected by the monitor
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 */
void WaveformReader::maxBeamLoss(int waveformIndex)
{
  double startingPosition, endingPosition;
  getDoubleParam(*(start_loc_indices[waveformIndex]), &startingPosition);
  getDoubleParam(*(end_loc_indices[waveformIndex]), &endingPosition);

  double lengthOfMonitor = endingPosition - startingPosition;

  int maxIndex = findMaxIndex(waveformIndex);

  int bufferSize;
  getIntegerParam(number_of_words_index, &bufferSize);
  // size of array is (no of 16-bit words * 2) because 8-bit words are cast to 16-bit words 
  bufferSize *= 2;

  double locationOfMaxIndex = (maxIndex * (lengthOfMonitor / bufferSize)) + startingPosition;
  std::cout << "The location of maximum beam loss is " << locationOfMaxIndex << std::endl;
  setDoubleParam(*(beam_loss_loc_indices[waveformIndex]), locationOfMaxIndex);
  callParamCallbacks();
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void maxBeamLossLocation(int waveformIndex) {
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