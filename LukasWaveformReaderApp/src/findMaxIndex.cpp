#include "WaveformReader.h"

/**
 * Finds the index of the peak or global maximum value in waveformData
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 * @return the index of the peak value
 */
int WaveformReader::findMaxIndex(int waveformIndex)
{
  int maxIndex = 0;
  std::string pvIdentifier = waveform_param_indices[waveformIndex];

  for (int i = 1; i < MAX_BUFFER_SIZE; i++)
  {
    if (waveform_map[pvIdentifier][i] > waveform_map[pvIdentifier][maxIndex]) 
    {
      maxIndex = i;
    }
  }
  return maxIndex;

}