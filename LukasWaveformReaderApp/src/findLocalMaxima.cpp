#include "WaveformReader.h"

/**
 * Finds the indices of all the local maxima of the waveform data and stores them in local_maxima_indices
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 */
void WaveformReader::findLocalMaxima(int waveformIndex)
{
  std::string pvIdentifier = waveform_param_indices[waveformIndex];
  if (waveform_map[pvIdentifier][0] > waveform_map[pvIdentifier][1]) {local_maxima_indices.push_back(0);}

  for(int i = 1; i < (MAX_BUFFER_SIZE - 1); i++) 
  { 
         
    if ((waveform_map[pvIdentifier][i - 1] < waveform_map[pvIdentifier][i]) && (waveform_map[pvIdentifier][i] > waveform_map[pvIdentifier][i + 1])) 
    {
      local_maxima_indices.push_back(i);
    } 

  }

  if (waveform_map[pvIdentifier][MAX_BUFFER_SIZE - 1] > waveform_map[pvIdentifier][MAX_BUFFER_SIZE - 2]) 
  {
    local_maxima_indices.push_back(MAX_BUFFER_SIZE - 1);
  }
 
}
