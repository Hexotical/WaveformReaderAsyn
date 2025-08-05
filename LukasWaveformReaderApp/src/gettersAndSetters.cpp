#include "WaveformReader.h"

WaveformReader* WaveformReader::getPortDriver()
{
  return WaveformReader::port_driver;
}


/**
 * Sets port_driver to newPortDriver only if it is a nullptr, doesn't create a copy
 */ 
void WaveformReader::setPortDriver(WaveformReader* newPortDriver)
{
  if (WaveformReader::port_driver == nullptr)
  {
    WaveformReader::port_driver = newPortDriver;
  }

}