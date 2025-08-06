#include "WaveformReader.h"

/**
 * readArray function overrides
 */
asynStatus WaveformReader::readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn)
{
  return asynSuccess;
}

asynStatus WaveformReader::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
  return asynSuccess;
}
