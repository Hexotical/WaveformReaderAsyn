#include "WaveformReader.h"

/**
 * Determines the relevant window from the entire waveform data on which data analysis is to be perfomed
 * 
 * @param low the starting point of the window
 * @param high the ending point of the window
 * @param maxIndex the index of the peak value of the waveform data
 * @param LOWER_LIMIT the smallest value of a local maxima that the window will include
 */
void WaveformReader::findRange(int& low, int& high, int maxIndex, const int LOWER_LIMIT, int waveformIndex)
{
  low = maxIndex - 1;
  high = maxIndex + 1;
  std::string pvIdentifier = waveform_param_indices[waveformIndex];
  while ((waveform_map[pvIdentifier][low - 1] <= waveform_map[pvIdentifier][low] || waveform_map[pvIdentifier][low] > LOWER_LIMIT) && (low > 0))
  {
    low--;
  } 
  while ((waveform_map[pvIdentifier][high + 1] <= waveform_map[pvIdentifier][high] || waveform_map[pvIdentifier][high] > LOWER_LIMIT) && (high < (STREAM_MAX_SIZE - 1)))
  {
    high++;
  } 

}