#include "WaveformReader.h"

/**
 * Finds the index of the peak or global maximum value in extracted waveform data
 * 
 * @param waveformIndex index of the waveform, such as 0 for WAVEFORM_0
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