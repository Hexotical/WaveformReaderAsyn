#include "WaveformReader.h"


/**
 * Overloaded assignment operator for WaveformReader object
 * 
 * Sets the port_driver pointer variable to rhs only if it is a nullptr, doesn't create a copy
 */
WaveformReader& WaveformReader::operator=(WaveformReader& rhs)
{
  if ((*this).port_driver == nullptr) 
  {
    (*this).port_driver = &rhs;
  }
  return (*this);
}