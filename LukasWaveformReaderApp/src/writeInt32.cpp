#include "WaveformReader.h"

/**
 * Override of asyn port driver default writeInt32,lets us set hardware by writing to epics records
 */
asynStatus WaveformReader::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  printf("Driver calls write Int32\n");
  asynStatus status = setIntegerParam(pasynUser->reason, value);
  callParamCallbacks();

  // execute instructions at hardware addresses based on which parameter of which waveform record
  // pasynUser matches with
  for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
  {
    if(pasynUser->reason == (*(endAddr_indices[i])))
    {
      (*(end_addresses[i]))->setVal((int64_t)value);
      _WebInit->execute();
    }

    if(pasynUser->reason == (*(beginAddr_indices[i])))
    {
      (*(start_addresses[i]))->setVal((int64_t)value);
    }
  }

  if(pasynUser->reason == waveform_buffer_size_index)
  {
    _DataBufferSize->setVal((int64_t)value);
    MAX_BUFFER_SIZE = 4 * value ;
  }

  // if the NO:OF:WORDS record updates
  if(pasynUser->reason == number_of_words_index)
  {
    int number_of_words;
    getIntegerParam(number_of_words_index, &number_of_words);

    /// TO BE TESTED
    number_of_words = ((number_of_words + 4) / 8) * 8;
    ///

    // Calculate the DaqMux Data Buffer Size (N/2)
    // Set the DaqMuxV2/DataBufferSize to N/2 (as this is expressed in 32-bit words)
    int daqMuxBufferSize = (number_of_words / 2);
    setIntegerParam(waveform_buffer_size_index, daqMuxBufferSize);
    callParamCallbacks();

    _DataBufferSize->setVal(daqMuxBufferSize);
    MAX_BUFFER_SIZE = 2 * number_of_words;
    

    int beginAddress, endAddress;

    for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
    {
      // get the beginning address of the stream
      getIntegerParam(*(beginAddr_indices[i]), &beginAddress);
        
      endAddress = beginAddress + (number_of_words * 2);

      setIntegerParam(*(endAddr_indices[i]), endAddress);
      callParamCallbacks();

      // set the value of the hardware
      (*(end_addresses[i]))->setVal(endAddress);
      
    }
    // initialize once for the entire bay
    _WebInit->execute();
    
  }

  return status;
}