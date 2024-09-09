#include "WaveformReader.h"

WaveformReader& WaveformReader::operator=(const WaveformReader& rhs)
{
  if ((*this).port_driver == nullptr) 
  {
    (*this).port_driver = new WaveformReader(rhs);
  }
  return (*this);
}