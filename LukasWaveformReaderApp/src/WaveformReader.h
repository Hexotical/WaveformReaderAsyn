//WaveformReader.h

#include <asynPortDriver.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTimer.h>
#include <epicsTypes.h>

#include <boost/array.hpp>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <array>
#include <map>
#include <mutex>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <math.h>
#include <fftw3.h>
#include <fstream>
#include <pthread.h>

#include <cpsw_api_user.h>
#include <yaml-cpp/yaml.h>
#include <cpsw_api_builder.h>
#include <yamlLoader.h>
#define STREAM_MAX_SIZE 200UL*1024ULL*1024ULL

#define WAVEFORM_RUN_STRING "RUN"
#define NO_OF_WORDS_STRING "NO_OF_WORDS"
#define WAVEFORM_BUFFER_SIZE_STRING "BUFFER_SIZE"
#define WAVEFORM_INITIALIZE_STRING "INITIALIZE"
#define CLK_FREQUENCY_STRING "CLK_FREQUENCY"
#define SPEED_STRING "FIBER_SPEED_OF_LIGHT"

#define REAL 0
#define IMAG 1
#define NUMBER_OF_WAVEFORM_RECORDS 3


class WaveformReader : public asynPortDriver
{
  public:

    WaveformReader(const char *portName, int bayNumber, int bufferSize, int waveformPVs);

    static WaveformReader* getPortDriver();
    static void setPortDriver(WaveformReader* newPortDriver);

    void extractWaveform(int waveformIndex);
    void fft(int waveformIndex);
    void findLocalMaxima(int waveformIndex);
    int findMaxIndex(int waveformIndex);
    void findPrelimMax(int waveformIndex);
    void findRange(int& low, int& high, int maxIndex, const int LOWER_LIMIT, int waveformIndex);
    void healthCheck(void);
    void healthTask(void);
    void maxBeamLoss(int waveformIndex);
    void statusCheck(void);
    WaveformReader& operator=(WaveformReader& rhs);


    void streamTask(const char *stream, std::string pvID);// takes a path to the stream and then a pv identifier for connection
    void streamInit(std::string pv_identifier, std::string stream_path);
    virtual asynStatus readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    //Parameter list indices, should never be written to but need to be read, I really don't want to deal with encapsulation
    //TODO find a way to make these read only without having to write a weird get or set method

    std::vector<std::string> waveform_param_indices; // order matters
    std::vector<std::string> complete_waveform_param_indices;
    std::vector<std::string> complete_z_axis_waveform_indices;
    std::vector<std::string> extracted_waveform_param_indices;
    std::vector<std::string> extracted_z_axis_waveform_indices;
    std::map<std::string, int> pv_param_map; //Identifier of pv to parameter in param list
    std::map<std::string, int> complete_param_map;
    std::map<std::string, int> complete_z_axis_param_map;
    std::map<std::string, int> extracted_param_map;
    std::map<std::string, int> z_axis_param_map;
    std::map<std::string, int> index_map; // map string identifiers to indices 0, 1, and 2, which are used to get waveform-specific data from arrays
    std::map<std::string, std::string> stream_path_map; // map PV identifier to the stream it was initialized with
    std::array<std::string, NUMBER_OF_WAVEFORM_RECORDS> streaming_status; // store the streaming status of the waveforms 
    std::array<std::string, NUMBER_OF_WAVEFORM_RECORDS> thread_status; // store the status of the threads - executing or dead
    std::array<std::chrono::milliseconds, NUMBER_OF_WAVEFORM_RECORDS> duration_data; // store the time it takes to read the stream from the hardware
    std::array<std::chrono::system_clock::time_point, NUMBER_OF_WAVEFORM_RECORDS> initialization_times; // store initialization time of each stream
    std::array<std::chrono::system_clock::time_point, NUMBER_OF_WAVEFORM_RECORDS> retrieval_times; // store time of latest retrieval of each stream
    std::array<std::chrono::steady_clock::time_point, NUMBER_OF_WAVEFORM_RECORDS> interval_times; // store the interval between the last and current retrieval
    std::array<bool, NUMBER_OF_WAVEFORM_RECORDS> initialization_status; // store initialization status (true/false) of each stream 
    std::vector<int> local_maxima_indices; // store indices of local maxima of the waveform data

    //Variables to store indices of records which the asynPortDriver can talk to.
    int waveform_run_index;
    int number_of_words_index;
    int waveform_buffer_size_index;
    int waveform_init_index;
    int MAX_BUFFER_SIZE;
    int clk_frequency_index;
    int speed_index;

    int waveform0_beginAddr_index;
    int waveform0_endAddr_index;
    int waveform0_beam_loss_loc_index;
    int waveform0_beam_loss_val_index;
    int waveform0_threshold_index;
    int waveform0_z_offset_start_index;
    int waveform0_z_offset_end_index;
    int waveform0_fiber_length_index;
    int waveform0_extraction_start_index;
    int waveform0_extraction_end_index;
    int waveform0_extracted_elements_index;
    int waveform0_interval_index;
    int waveform0_offset_index;
    int waveform0_slope_index;
    int waveform0_extract_index;

    int waveform1_beginAddr_index;
    int waveform1_endAddr_index;
    int waveform1_beam_loss_loc_index;
    int waveform1_beam_loss_val_index;
    int waveform1_threshold_index;
    int waveform1_z_offset_start_index;
    int waveform1_z_offset_end_index;
    int waveform1_fiber_length_index;
    int waveform1_extraction_start_index;
    int waveform1_extraction_end_index;
    int waveform1_extracted_elements_index;
    int waveform1_interval_index;
    int waveform1_offset_index;
    int waveform1_slope_index;
    int waveform1_extract_index;
    
    int waveform2_beginAddr_index;
    int waveform2_endAddr_index;
    int waveform2_beam_loss_loc_index;
    int waveform2_beam_loss_val_index;
    int waveform2_threshold_index;
    int waveform2_z_offset_start_index;
    int waveform2_z_offset_end_index;
    int waveform2_fiber_length_index;
    int waveform2_extraction_start_index;
    int waveform2_extraction_end_index;
    int waveform2_extracted_elements_index;
    int waveform2_interval_index;
    int waveform2_offset_index;
    int waveform2_slope_index;
    int waveform2_extract_index;
    
    // the indices of the arrays, 0, 1, and 2, refer to WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2, respectively
    int* beginAddr_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_beginAddr_index, &waveform1_beginAddr_index, &waveform2_beginAddr_index};
    int* endAddr_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_endAddr_index, &waveform1_endAddr_index, &waveform2_endAddr_index};
    int* beam_loss_loc_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_beam_loss_loc_index, &waveform1_beam_loss_loc_index, &waveform2_beam_loss_loc_index};
    int* beam_loss_val_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_beam_loss_val_index, &waveform1_beam_loss_val_index, &waveform2_beam_loss_val_index};
    int* threshold_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_threshold_index, &waveform1_threshold_index, &waveform2_threshold_index};
    int* z_offset_start_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_z_offset_start_index, &waveform1_z_offset_start_index, &waveform2_z_offset_start_index};
    int* z_offset_end_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_z_offset_end_index, &waveform1_z_offset_end_index, &waveform2_z_offset_end_index};
    int* fiber_length_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_fiber_length_index, &waveform1_fiber_length_index, &waveform2_fiber_length_index};
    int* extraction_start_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_extraction_start_index, &waveform1_extraction_start_index, &waveform2_extraction_start_index};
    int* extraction_end_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_extraction_end_index, &waveform1_extraction_end_index, &waveform2_extraction_end_index};
    int* extracted_elements_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_extracted_elements_index, &waveform1_extracted_elements_index, &waveform2_extracted_elements_index};
    int* interval_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_interval_index, &waveform1_interval_index, &waveform2_interval_index};
    int* offset_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_offset_index, &waveform1_offset_index, &waveform2_offset_index};
    int* slope_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_slope_index, &waveform1_slope_index, &waveform2_slope_index};
    int* extract_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_extract_index, &waveform1_extract_index, &waveform2_extract_index};

    //Hardware interfaces
  protected:
    ScalVal _TriggerHwAutoRearm;
    ScalVal _DataBufferSize;
    ScalVal_RO _ClkFrequency;
    ScalVal_RO _TrigCount;
    Command _WebInit;

    ScalVal _Web0StartAddr;
    ScalVal _Web0EndAddr;
    ScalVal _Web1StartAddr;
    ScalVal _Web1EndAddr;
    ScalVal _Web2StartAddr;
    ScalVal _Web2EndAddr;

    ScalVal* start_addresses[NUMBER_OF_WAVEFORM_RECORDS] = {&_Web0StartAddr, &_Web1StartAddr, &_Web2StartAddr};
    ScalVal* end_addresses[NUMBER_OF_WAVEFORM_RECORDS] = {&_Web0EndAddr, &_Web1EndAddr, &_Web2EndAddr};


  private:
    //epicsInt16* waveformData0; //Not really necessary atm I want to use this when I do data modification things
    std::map<std::string, epicsInt16*> waveform_map; // maps the pv Identifier to the corresponding array
    std::map<std::string, epicsInt16*> complete_waveform_map; // maps the pv Identifier to the corresponding array
    std::map<std::string, epicsFloat64*> complete_z_axis_waveform_map;
    std::map<std::string, epicsInt16*> extracted_waveform_map; // maps the pv Identifier to the corresponding array
    std::map<std::string, epicsFloat64*> extracted_z_axis_waveform_map;
    static WaveformReader* port_driver; // stores the port driver that will be used to execute the iocsh commands
};

/**
 * EpicsThreadCreate only lets us use a void pointer as an argument to the function
 * we would like to pass to it so we define a struct with the necessary information 
 * for streaming data.
 */
struct StreamArgs 
{
  void * pPvt;
  std::string stream_path_to_find;
  std::string pv_identifier;
  int stream;
  
};