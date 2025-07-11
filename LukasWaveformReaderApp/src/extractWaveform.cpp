#include "WaveformReader.h"

void WaveformReader::extractWaveform(int waveformIndex)
{
  double clock_frequency = 0, scaled_input;//, offset, slope;
  double sum = 0;
  double time_ns = 0;
  int offset, slope;

  // might have to change clock_frequency to int because the record type errors on asynFloat64
  getDoubleParam(clk_frequency_index, &clock_frequency);
  getIntegerParam(*(offset_indices[waveformIndex]), &offset);
  getIntegerParam(*(slope_indices[waveformIndex]), &slope);

  std::string original_pvIdentifier = waveform_param_indices[waveformIndex];

  epicsInt16 *refined_waveform = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16));
  epicsInt16 *x_axis_time = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16));  

  /* Calculate the time in ns between each sample */
  time_ns = 1000.0 / (2.0 * clock_frequency);

  /* Then make an array with each element a distance of `timeNs` apart */
  for (int i = 0; i < MAX_BUFFER_SIZE; i++) 
  {
    x_axis_time[i] = sum;
    sum += time_ns;

    scaled_input = waveform_map[original_pvIdentifier][i] + offset;
    refined_waveform[i] = scaled_input * slope;
  }

  //std::string refined_pvIdentifier = refined_waveform_param_indices[waveformIndex];
  //std::string x_axis_pvIdentifier = x_axis_waveform_indices[waveformIndex];
  // assuming all these are doubles, double check this
  double length, z_offset_start, z_offset_end, speed, extraneous_length, actual_length, extraction_start, extraction_end;
  getDoubleParam(*(length_indices[waveformIndex]), &length);
  getDoubleParam(*(z_offset_start_indices[waveformIndex]), &z_offset_start);
  getDoubleParam(*(z_offset_end_indices[waveformIndex]), &z_offset_end);
  getDoubleParam(speed_index, &speed);
  getDoubleParam(*(extraction_start_indices[waveformIndex]), &extraction_start);
  getDoubleParam(*(extraction_end_indices[waveformIndex]), &extraction_end);

  // represents the length of each vertical part of the fiber (because it's symmetric)
  actual_length = z_offset_end - z_offset_start;
  extraneous_length = (length - actual_length) / 2;

  // (epicsInt16 *) first_iteration;
  epicsInt16 *first_iteration = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsInt16 *x_axis_distance = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 

  // WANT TO TRAVERSE ENTIRE WAVEFORM, CHECK IF MAX_BUFFER_SIZE IS THE CORRECT LIMIT OR DO I HAVE TO USE WAVEFORM_SIZE
  // shave off extraneous data, just keep the first iteration of the waveform data
  double total_distance = 0;
  int no_of_elements = 0;
  for (int i = 0; i < MAX_BUFFER_SIZE; i++)
  {
    if (total_distance > length) {break;}
    x_axis_distance[i] = x_axis_time[i] * speed;
    total_distance = x_axis_distance[i];
    first_iteration[i] = refined_waveform[i];
    no_of_elements++;
  }

  epicsInt16 *waveform_data_clipped = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsInt16 *x_axis_clipped = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16));

  // remove data representing vertical parts
  int no_of_valid_elements = 0;
  for (int i = 0; i < no_of_elements; i++)
  {
    if (x_axis_distance[i] > extraneous_length && x_axis_distance[i] < (length - extraneous_length)) 
    {
      waveform_data_clipped[no_of_valid_elements] = first_iteration[i];
      x_axis_clipped[no_of_valid_elements] = x_axis_distance[i] - extraneous_length;
      no_of_valid_elements++;
    }
  }

  epicsInt16 *waveform_data = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsInt16 *x_axis = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  // invert the waveforms because right now the data starts on the right side of the fiber
  for (int i = 0; i < no_of_valid_elements; i++) 
  {
    waveform_data[i] = waveform_data_clipped[no_of_valid_elements - i - 1];
    x_axis[i] = actual_length - x_axis_clipped[no_of_valid_elements - i - 1] + z_offset_start;
  }

  epicsInt16 *extracted_waveform_data = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsInt16 *extracted_x_axis = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  int no_of_extracted_elements = 0;
  // extract relevant part of the waveform based on starting and ending extraction points entered by the user
  for (int i = 0; i < no_of_valid_elements; i++) 
  {
    if (x_axis[i] >= extraction_start && x_axis[i] <= extraction_end)
    {
      extracted_x_axis[no_of_extracted_elements] = x_axis[i];
      extracted_waveform_data[no_of_extracted_elements] = waveform_data[i];
      no_of_extracted_elements++;
    }
  }

  setIntegerParam(*(extracted_elements_indices[waveformIndex]), no_of_extracted_elements);
  callParamCallbacks();

  std::string extracted_pvIdentifier = extracted_waveform_param_indices[waveformIndex];
  //std::string extracted_x_axis_pvIdentifier = extracted_x_axis_waveform_indices[waveformIndex];

  
  for (int i = 0; i < no_of_extracted_elements; i++) 
  {
    //extracted_x_axis_waveform_map[extracted_x_axis_pvIdentifier][i] = extracted_x_axis[i];
    extracted_waveform_map[extracted_pvIdentifier][i] = extracted_waveform_data[i];

    // do I need some kind of array callbacks here?
  }

}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void waveformExtraction(int waveformIndex)
{
  WaveformReader* bayManager = WaveformReader::getPortDriver();
  bayManager->extractWaveform(waveformIndex);
  return;
}

static const iocshArg extractWaveformArg0 = {"waveformIndex", iocshArgInt};
static const iocshArg * const extractWaveformArgs[] = {&extractWaveformArg0};
static const iocshFuncDef extractWaveformFuncDef = {"waveformExtraction", 1, extractWaveformArgs};
static void extractWaveformCallFunc(const iocshArgBuf *args)
{
  waveformExtraction(args[0].ival);
}

void waveformExtractionRegister(void)
{
  iocshRegister(&extractWaveformFuncDef, extractWaveformCallFunc);
}

extern "C" {
  epicsExportRegistrar(waveformExtractionRegister);
}