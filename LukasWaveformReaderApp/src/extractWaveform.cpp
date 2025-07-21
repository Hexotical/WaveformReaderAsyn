#include "WaveformReader.h"

/**
 * Extracts relevant portion of the waveform from the entire buffer by removing extraneous data and clipping the remaining data based on 
 * starting and ending physical locations entered by the user in the extraction_start and extraction_end PVs.
 * Also creates an x_axis that contains physical locations corresponding to the values in the waveform.
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 */
void WaveformReader::extractWaveform(int waveformIndex)
{
  int clock_frequency = 0, scaled_input;
  double sum = 0;
  double time_ns = 0;
  int offset, slope;

  getIntegerParam(clk_frequency_index, &clock_frequency);
  getIntegerParam(*(offset_indices[waveformIndex]), &offset);
  getIntegerParam(*(slope_indices[waveformIndex]), &slope);

  std::string original_pvIdentifier = waveform_param_indices[waveformIndex];

  epicsInt16 *refined_waveform = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16));
  epicsFloat64 *x_axis_time = (epicsFloat64 *)calloc(STREAM_MAX_SIZE, sizeof(epicsFloat64));  

  // Calculate the time in ns between each sample 
  //time_ns = 1000.0 / (2.0 * clock_frequency); // this assumes frequency is in Mhz
  time_ns = 1000000000.0 / (2.0 * clock_frequency); // clock frequency is in Hz

  // Then make an array with each element a distance of `timeNs` apart
  for (int i = 0; i < MAX_BUFFER_SIZE; i++) 
  {
    x_axis_time[i] = sum;
    sum += time_ns;

    scaled_input = waveform_map[original_pvIdentifier][i] + offset;
    refined_waveform[i] = scaled_input * slope;
  }

  double z_offset_start, z_offset_end, fiber_length, speed, extraneous_length, actual_length, extraction_start, extraction_end;
  
  getDoubleParam(*(z_offset_start_indices[waveformIndex]), &z_offset_start);
  getDoubleParam(*(z_offset_end_indices[waveformIndex]), &z_offset_end);
  getDoubleParam(speed_index, &speed);
  getDoubleParam(*(extraction_start_indices[waveformIndex]), &extraction_start);
  getDoubleParam(*(extraction_end_indices[waveformIndex]), &extraction_end);
  getDoubleParam(*(fiber_length_indices[waveformIndex]), &fiber_length);

  epicsInt16 *first_iteration = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsFloat64 *x_axis_distance = (epicsFloat64 *)calloc(STREAM_MAX_SIZE, sizeof(epicsFloat64)); 

  // represents the length of each vertical part of the fiber (because it's symmetric)
  actual_length = z_offset_end - z_offset_start;
  extraneous_length = (fiber_length - actual_length) / 2;

  if (extraneous_length < 0)
  {
    std::cout << "Fiber length cannot be less than the actual length, resetting fiber length equal to actual length." << std::endl;
    fiber_length = actual_length;
    extraneous_length = 0;
  }


  // shave off extraneous data, just keep the first iteration of the waveform data
  double total_distance = 0;
  int no_of_elements = 0;
  for (int i = 0; i < MAX_BUFFER_SIZE; i++)
  {
    if (total_distance > fiber_length) {break;}
    x_axis_distance[i] = x_axis_time[i] * speed;
    total_distance = x_axis_distance[i];
    first_iteration[i] = refined_waveform[i];
    no_of_elements++;
  }

  epicsInt16 *waveform_data_clipped = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsFloat64 *x_axis_clipped = (epicsFloat64 *)calloc(STREAM_MAX_SIZE, sizeof(epicsFloat64));

  // remove data representing vertical parts
  int no_of_valid_elements = 0;
  for (int i = 0; i < no_of_elements; i++)
  {
    if (x_axis_distance[i] > extraneous_length && x_axis_distance[i] <= (fiber_length - extraneous_length)) 
    {
      waveform_data_clipped[no_of_valid_elements] = first_iteration[i];
      x_axis_clipped[no_of_valid_elements] = x_axis_distance[i] - extraneous_length;
      no_of_valid_elements++;
    }
  }

  epicsInt16 *waveform_data = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsFloat64 *x_axis = (epicsFloat64 *)calloc(STREAM_MAX_SIZE, sizeof(epicsFloat64)); 

  // invert the waveforms because right now the data starts on the right side of the fiber
  for (int i = 0; i < no_of_valid_elements; i++) 
  {
    waveform_data[i] = waveform_data_clipped[no_of_valid_elements - i - 1];
    x_axis[i] = actual_length - x_axis_clipped[no_of_valid_elements - i - 1] + z_offset_start;
  }

  if (extraction_start > extraction_end || extraction_start < z_offset_start || extraction_start > z_offset_end
   || extraction_end > z_offset_end || extraction_end < z_offset_start) 
  {
    std::cout << "Extraction position values are out of bounds, resetting them to default values." << std::endl;
    extraction_start = z_offset_start;
    extraction_end = z_offset_end;
  }

  epicsInt16 *extracted_waveform_data = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
  epicsFloat64 *extracted_x_axis = (epicsFloat64 *)calloc(STREAM_MAX_SIZE, sizeof(epicsFloat64)); 
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
  std::string extracted_x_axis_pvIdentifier = extracted_x_axis_waveform_indices[waveformIndex];

  // store the extracted information into PVs
  for (int i = 0; i < no_of_extracted_elements; i++) 
  {
    extracted_x_axis_waveform_map[extracted_x_axis_pvIdentifier][i] = extracted_x_axis[i];
    extracted_waveform_map[extracted_pvIdentifier][i] = extracted_waveform_data[i];
  }

  doCallbacksInt16Array(extracted_waveform_data, no_of_extracted_elements, extracted_param_map[extracted_pvIdentifier], 0);
  doCallbacksFloat64Array(extracted_x_axis, no_of_extracted_elements, x_axis_param_map[extracted_x_axis_pvIdentifier], 0);
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