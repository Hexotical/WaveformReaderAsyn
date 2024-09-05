#include "WaveformReader.h"

/**
 * Override of asyn port driver default writeUInt32Digital,lets us set hardware by writing to epics records
 */
asynStatus WaveformReader::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  asynStatus status = setUIntDigitalParam(pasynUser->reason, value, mask);
  callParamCallbacks();
  printf("Driver calls write UInt32\n");
  if(pasynUser->reason == waveform_run_index)
  {
    printf("Value of %d\n", value);
    //Do a thing with the firmware here?????? I mean if I'm writing I just need to set the value don't I
    //This is for turning on and off so this should in theory be the TriggerHardwareAutoRearm
    //Essentially just using my ScalVal interface to tell the address to turn on or off, should be that simple according to Jeremy
    _TriggerHwAutoRearm->setVal((int64_t)value);
  }
  if(pasynUser->reason == waveform_init_index)
  {
    _WebInit->execute();
  }

  return status;
}