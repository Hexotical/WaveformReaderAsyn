//WaveformReader.h

#include <asynPortDriver.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTimer.h>
#include <epicsTypes.h>

//#include <fstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <mutex>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <math.h>
#include <fftw3.h>
#include <fstream>

#include <cpsw_api_user.h>
#include <yaml-cpp/yaml.h>
#include <cpsw_api_builder.h>
#include <yamlLoader.h>
#define STREAM_MAX_SIZE 200UL*1024ULL*1024ULL
#define WAVEFORM_RUN_STRING "RUN"

#define WAVEFORM0_PV_STRING "WAVEFORM:0"
#define WAVEFORM0_INITIALIZE_STRING "INITIALIZE0"
#define WAVEFORM0_END_ADDR_STRING "END_ADDR0"
#define WAVEFORM0_BEGIN_ADDR_STRING "BEGIN_ADDR0"
#define WAVEFORM0_BUFFER_SIZE_STRING "BUFFER_SIZE0"
#define WAVEFORM0_BUFFER_SIZE_INIT_STRING "WAVEFORM_BUFFER_SIZE_INIT0"


#define WAVEFORM1_PV_STRING "WAVEFORM:1"
#define WAVEFORM1_INITIALIZE_STRING "INITIALIZE1"
#define WAVEFORM1_END_ADDR_STRING "END_ADDR1"
#define WAVEFORM1_BEGIN_ADDR_STRING "BEGIN_ADDR1"
#define WAVEFORM1_BUFFER_SIZE_STRING "BUFFER_SIZE1"
#define WAVEFORM1_BUFFER_SIZE_INIT_STRING "WAVEFORM_BUFFER_SIZE_INIT1"


#define WAVEFORM2_PV_STRING "WAVEFORM:2"
#define WAVEFORM2_INITIALIZE_STRING "INITIALIZE2"
#define WAVEFORM2_END_ADDR_STRING "END_ADDR2"
#define WAVEFORM2_BEGIN_ADDR_STRING "BEGIN_ADDR2"
#define WAVEFORM2_BUFFER_SIZE_STRING "BUFFER_SIZE2"
#define WAVEFORM2_BUFFER_SIZE_INIT_STRING "WAVEFORM_BUFFER_SIZE_INIT2"

#define REAL 0
#define IMAG 1
#define NUMBER_OF_WAVEFORM_RECORDS 3

class WaveformReader : public asynPortDriver
{
  public:
    WaveformReader(const char *portName, int bufferSize, int waveformPVs);

    void statusCheck(void);
    void fft(void);
    int findMaxIndex(void);
    void findRange(int& low, int& high, int maxIndex, const int LOWER_LIMIT);
    void findLocalMaxima(void);


    void streamTask(const char *stream, std::string pvID);// takes a path to the stream and then a pv identifier for connection
    void streamInit(std::string pv_identifier, std::string stream_path);
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    //Parameter list indices, should never be written to but need to be read, I really don't want to deal with encapsulation
    //TODO find a way to make these read only without having to write a weird get or set method

    std::vector<std::string> waveform_param_indices; //order of this doesn't really matter
    std::map<std::string, int> pv_param_map; //Identifier of pv to parameter in param list
    std::map<std::string, std::string> streaming_status_map; // map the string identifiers to their streaming status
    std::vector<int> local_maxima_indices; // store indices of local maxima of the waveform data
    //std::map<std::string, std::string> stream_pv_map; //stream path to pv identifier

    //Variables to store indices of records which the asynPortDriver can talk to.
    int waveform_run_index;
    int MAX_BUFFER_SIZE;

    int waveform0_init_index;
    int waveform0_beginAddr_index;
    int waveform0_endAddr_index;
    int waveform0_buffer_size_index;
    
    int waveform1_init_index;
    int waveform1_beginAddr_index;
    int waveform1_endAddr_index;
    int waveform1_buffer_size_index;
    
    int waveform2_init_index;
    int waveform2_beginAddr_index;
    int waveform2_endAddr_index;
    int waveform2_buffer_size_index;
    
    // the indices of the arrays, 0, 1, and 2, refer to WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2, respectively
    int* init_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_init_index, &waveform1_init_index, &waveform2_init_index};
    int* beginAddr_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_beginAddr_index, &waveform1_beginAddr_index, &waveform2_beginAddr_index};
    int* endAddr_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_endAddr_index, &waveform1_endAddr_index, &waveform2_endAddr_index};
    int* buffer_size_indices[NUMBER_OF_WAVEFORM_RECORDS] = {&waveform0_buffer_size_index, &waveform1_buffer_size_index, &waveform2_buffer_size_index};


    //Hardware interfaces
  protected:
    ScalVal _TriggerHwAutoRearm;
    ScalVal _DataBufferSize;
    ScalVal_RO _TrigCount;
    ScalVal _Web0StartAddr;
    ScalVal _Web0EndAddr;
    Command _Web0Init;

    ScalVal _Web1StartAddr;
    ScalVal _Web1EndAddr;
    ScalVal _Web2StartAddr;
    ScalVal _Web2EndAddr;

    ScalVal start_addresses[NUMBER_OF_WAVEFORM_RECORDS] = {_Web0StartAddr, _Web1StartAddr, _Web2StartAddr};
    ScalVal end_addresses[NUMBER_OF_WAVEFORM_RECORDS] = {_Web0EndAddr, _Web1EndAddr, _Web2EndAddr};


  private:
    epicsInt16 *waveformData; //Not really necessary atm I want to use this when I do data modification things


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
