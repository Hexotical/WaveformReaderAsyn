#include "WaveformReader.h"


/**
 * Overloaded assignment operator for WaveformReader object
 * 
 * Sets the port_driver pointer variable to rhs only if it is a nullptr
 */
WaveformReader& WaveformReader::operator=(const WaveformReader& rhs)
{
  if ((*this).port_driver == nullptr) 
  {
    (*this).port_driver = new WaveformReader(rhs);
  }
  return (*this);
}