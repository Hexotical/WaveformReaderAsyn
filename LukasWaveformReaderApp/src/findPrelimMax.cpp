#include "WaveformReader.h"

void WaveformReader::findPrelimMax(int waveformIndex)
{
  std::string original_pvIdentifier = waveform_param_indices[waveformIndex];
  int maxVal = waveform_map[original_pvIdentifier][0];

  double threshold = 0;
  getDoubleParam(*(threshold_indices[waveformIndex]), &threshold);

  for (int i = 1; i < MAX_BUFFER_SIZE; i++)
  {
    if (waveform_map[original_pvIdentifier][i] > maxVal) 
    {
      maxVal = waveform_map[original_pvIdentifier][i];
    }
  }

  if (maxVal > threshold)
  {
    std::cout << "Setting extract to TRUE" << std::endl;
    setUIntDigitalParam(*(extract_indices[waveformIndex]), 1, 1);
    callParamCallbacks();
  }
}