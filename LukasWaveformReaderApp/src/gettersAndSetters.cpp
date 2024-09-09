#include "WaveformReader.h"

WaveformReader* WaveformReader::getPortDriver()
{
  return WaveformReader::port_driver;
}

void WaveformReader::setPortDriver(WaveformReader* newPortDriver)
{
  if (WaveformReader::port_driver == nullptr)
  {
    WaveformReader::port_driver = newPortDriver;
  }

}