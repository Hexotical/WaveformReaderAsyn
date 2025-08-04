#include "WaveformReader.h"

/**
 * Finds the index of the peak or global maximum value in waveformData
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 * @return the index of the peak value
 */
int WaveformReader::findMaxIndex(int waveformIndex)
{
  int maxIndex = 0, number_of_elements;
  std::string pvIdentifier = extracted_waveform_param_indices[waveformIndex];
  getIntegerParam(*(extracted_elements_indices[waveformIndex]), &number_of_elements);
  std::cout << "The extracted number of elements in findMaxIndex is " << number_of_elements << std::endl;
 
  for (int i = 1; i < number_of_elements; i++)
  {
    if (extracted_waveform_map[pvIdentifier][i] > extracted_waveform_map[pvIdentifier][maxIndex]) 
    {
      maxIndex = i;
    }
  }
  std::cout << "Max index is: " << maxIndex << std::endl;
  return maxIndex;

}