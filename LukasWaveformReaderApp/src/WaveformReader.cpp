//WaveformReader.cpp

#include "WaveformReader.h"

void streamTask(void * driverPointer); 
void streamInit(void *driverPointer); 
void streamInit(int channel);

WaveformReader *bayManager; //Global pointer so iocshell commands can interact with our initialized port driver

/**
 * Initialize an ASYN Port Driver
 *
 * @param portName port for the asyn driver to use
 * @param bufferSize amount of words to read from the buffer
 * @param waveformPVs amount of EPICS waveform records our port driver should be of
 */
WaveformReader::WaveformReader(const char *portName, int bufferSize, int waveformPVs) : asynPortDriver
                                                       (
                                                        portName,
                                                        1,//Max Signals?
                                                        asynDrvUserMask | asynInt32ArrayMask | asynInt16ArrayMask | asynUInt32DigitalMask | asynInt32Mask ,
                                                        asynInt32ArrayMask | asynInt16ArrayMask | asynInt32Mask | asynUInt32DigitalMask,
                                                        ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                                                        1,
                                                        0,
                                                        0
                                                        )
{
  waveformData = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 
 
  //Connecting to the records our port driver will eventually need to interact with
  for(int pvID = 0; pvID < waveformPVs; pvID++)
  {
    //For loop generates the string identifier for each Waveform records and then creates a parameter our asynDriver can interact with for it
    int waveform_param_index;
    std::string pvIdentifier = "WAVEFORM:" + std::to_string(pvID);
    std::cout << pvIdentifier << std::endl;
    createParam(pvIdentifier.c_str(), asynParamInt16Array, &waveform_param_index);
    std::cout << "The identifier is: " << pvIdentifier << " and the waveform_param_index is : " << waveform_param_index << std::endl;
    pv_param_map.insert(std::pair<std::string, int>(pvIdentifier, waveform_param_index));
    waveform_param_indices.push_back(pvIdentifier);
    streaming_status_map[pvIdentifier] = "Not initialized yet";
  }
  //TODO: Do this is a more systematic way, individually connecting isn't really aesthetic 
  createParam(WAVEFORM_RUN_STRING, asynParamUInt32Digital, &waveform_run_index);
  createParam(WAVEFORM0_INITIALIZE_STRING, asynParamUInt32Digital, &waveform_init_index);
  createParam(WAVEFORM0_END_ADDR_STRING, asynParamInt32, &waveform_endAddr_index);
  createParam(WAVEFORM0_BEGIN_ADDR_STRING, asynParamInt32, &waveform_beginAddr_index);
  createParam(WAVEFORM0_BUFFER_SIZE_STRING, asynParamInt32, &waveform_buffer_size_index);
  
  MAX_BUFFER_SIZE = bufferSize; //One of the parameters we pass to our port driver is the bufferSize, which is essentially how many words of information we want at a time

  //We register some useful hardware interfaces our port driver could want to know

  Path p;
  p = cpswGetRoot();
  _TriggerHwAutoRearm = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TriggerHwAutoRearm"));
  _DataBufferSize = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/DataBufferSize"));
  _TrigCount = IScalVal_RO::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TrigCount"));
  //TODO: Do these for the other hardware addresses that are useful
  _Web0StartAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[0]"));
  _Web0EndAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[0]"));
  _Web0Init = ICommand::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/Initialize")); 
}

//TODO Make this actually useful, tell us what streams are currently connected and meant to be streaming, if they're doing that successfully or not
/**
 * Check the health of any stream our port driver is connected to
 */
void WaveformReader::statusCheck(void)
{
  std::cout << "Status of connected waveforms: " << std::endl;
  for(std::string paramIndex:waveform_param_indices)
  {
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "| " << std::setw(15) << paramIndex << " | " << std::setw(50) << streaming_status_map[paramIndex] << " |" << std::endl;
  }
  std::cout << "------------------------------------------------------------------------" << std::endl;
}

void WaveformReader::fft(void)
{
  int maxIndex = findMaxIndex();
  int low, high;
  findRange(low, high, maxIndex);
  // Define the length of the complex arrays
  int n = high - low + 1; // buffer size from waveformConfigure call in st.cmd //200000
  // Input array
  // IS OURS A REAL VALUED SIGNAL
  // Dynamically allocate the array because the size can change
  fftw_complex* x = new fftw_complex[n]; // This is equivalent to: double x[n][2];
  // Output array
  fftw_complex* y = new fftw_complex[n];
  // Samping frequency
  double sampling_frequency = 3.57142857143e8; // NEED VALUE OF SAMPLING FREQUENCY

  //std::cout << "Just trying maxIndex: " << findMaxIndex() << std::endl;
  //std::cout << "Above the first for loop" << std::endl;
  // Fill the first array with data from waveformData
  for (int i = 0; i < n; i++)
  {
      x[i][REAL] = waveformData[i + low];
      x[i][IMAG] = 0;
  }
  //Plant the FFT and execute it
  fftw_plan plan = fftw_plan_dft_1d(n, x, y, FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_execute(plan);
  //Do some cleaning
  fftw_destroy_plan(plan);
  fftw_cleanup();
  // Display the results
  std::cout << "FFT = " << std::endl;
  double frequency;
  std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "|    No.    | FREQUENCY (in Hz) |             FFT RESULT             |     MAGNITUDE     | PHASE (in radians) |" << std::endl;
  std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
  for (int i = 0; i < n; i++)
  {
      frequency = (i * sampling_frequency) / n;
      if (y[i][IMAG] < 0)
      {
          std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " - " << std::setw(16) << std::left << abs(y[i][IMAG]) << "i";
      }
      else
      {
          std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " + " << std::setw(16) << std::left << y[i][IMAG] << "i" ;
      }
      std::cout << "|" << std::setw(19) << std::right << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2));
      std::cout << "|" << std::setw(20) << std::right << atan2(y[i][IMAG], y[i][REAL]) << "|" << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
  }
}

int WaveformReader::findMaxIndex(void)
{
  int maxIndex = waveformData[0];

  for (int i = 0; i < STREAM_MAX_SIZE; i++)
  {
    if (waveformData[i] > maxIndex) {maxIndex = i;}
    //std::cout << "Value at index " << i << " is " << waveformData[i] << std::endl;
  }
  return maxIndex;
}

void WaveformReader::findRange(int& low, int& high, int maxIndex)
{
  const int LOWER_LIMIT = 5; // NEED THIS
  low = maxIndex - 1;
  high = maxIndex + 1;
  while ((waveformData[low - 1] <= waveformData[low] || waveformData[low] > LOWER_LIMIT) && (low > 0))
  {
    low--;
  } 
  while ((waveformData[high + 1] <= waveformData[high] || waveformData[high] > LOWER_LIMIT) && (high < (STREAM_MAX_SIZE - 1)))
  {
    high++;
  } 

}

void WaveformReader::findLocalMaxima(void)
{
  if (waveformData[0] > waveformData[1]) {local_maxima_indices.push_back(0);}

  for(int i = 1; i < (STREAM_MAX_SIZE - 1); i++) 
  { 
         
    if ((waveformData[i - 1] < waveformData[i]) and (waveformData[i] > waveformData[i + 1])) 
    {
      local_maxima_indices.push_back(i);
    } 

  }

  if (waveformData[STREAM_MAX_SIZE - 1] > waveformData[STREAM_MAX_SIZE - 2]) 
  {
    local_maxima_indices.push_back(STREAM_MAX_SIZE - 1);
  }
 
}

/**
 * Launch a thread to stream data to an EPICS records
 *
 * @param pv_identifier string corresponding to the Epics record we wish to have data in
 * @param stream_path string form of the path to the stream we want to connect to
 *
 * Does not return
 */
void WaveformReader::streamInit(std::string pv_identifier, std::string stream_path)
{
  StreamArgs toPass; //Initialize a structure to pass the arguments we need to begin streaming

  toPass.pPvt = this;
  toPass.pv_identifier = pv_identifier;
  toPass.stream_path_to_find = stream_path; 

  //std::cout << "The path we pass is: " << toPass.stream_path_to_find << std::endl;
  //printf("\nChannel number in toPass is %d\n" ,toPass.stream);

  asynStatus status;
  status = (asynStatus)(epicsThreadCreate("WaveformTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::streamTask, &toPass) == NULL);
  if(status == asynError)
  {
    std::cout << "Unable to launch a waveform stream; " << status << std::endl;
  }
  else
  {
    std::cout << "Succesfully launched waveform stream=> " << pv_identifier << " status: " << status << std::endl;
  }
  sleep(3); //Sleep so the launched thread can find the structure before it's overwritten by garbage TODO: Do this in a better way 
}

/**
 * Tell a port driver to use it's streamTask method with the passed args generally called by streamInit
 *
 * @param streamArgs port driver and the required args to begin streaming
 */
void streamTask(void* streamArgs)
{
  StreamArgs *passedArgs = static_cast<StreamArgs*>(streamArgs);

  WaveformReader *pPvt = (WaveformReader *) passedArgs->pPvt;
  pPvt->streamTask(passedArgs->stream_path_to_find.c_str(), passedArgs->pv_identifier);
}

/**
 * Connect to a stream, write the data retrieved from the stream to the specified EPICS record
 *
 * @param streamInit path to the stream to connect and read from
 * @param pvID Identifier of the EPICS record to write data from the stream to
 */
void WaveformReader::streamTask(const char *streamInit = "/Stream0", std::string pvID = "WAVEFORM:0")//, int waveform_param_index = -1)//Stream stm, int param16index, int param32index)
{
        sleep(1);

        //TODO based on streamInit, add key and param to the asynPortDriver

        std::cout << "Passed pvID: " << pvID << std::endl;
        int waveform_param_index = pv_param_map[pvID];
        std::cout << pvID << " corresponding index: " << waveform_param_index << std::endl;

        Path p;
        p = cpswGetRoot();

        std::cout << "Stream Init (path to the stream): " << streamInit << std::endl;
        Stream stm;

        try {
          stm = IStream::create(p->findByName(streamInit));
        }
        catch (CPSWError &e) {
          std::cerr << e.what() << std::endl;
        }

        if(stm)
        {
          std::cout << "Value of stm: " << stm << std::endl;
          printf("Did that thing with a stream?\n");
          streaming_status_map[pvID] = "Successfully initialized"; 
        }
        else {
          printf("No stream access");
          streaming_status_map[pvID] = "Initialization failed";
          return;
        }

        uint8_t *buf = new uint8_t[STREAM_MAX_SIZE];
        size_t nWords16, nBytes;
        int64_t got = 0;
        int64_t lastGot = 0;

        std::cout << "Outside the while loop now " << std::endl;

        while(1)
        {
            //std::cout << "Inside the while loop " << std::endl;
            //std::cout << streamInit << std::endl;
            //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            got = stm->read( buf, MAX_BUFFER_SIZE, CTimeout(-1));
            //std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
            //std::cout << "No. of values read in from the stream (value of got): " << got << std::endl;
            //std::cout << "Value of lastGot: " << lastGot << std::endl;

            if(got > 8)
            {
                //printf("There's a thing in the stream? %ld bytes\n", got);
                lock();
                nBytes = (got - 9); // header = 8 bytes, footer = 1 byte, data = 32bit words.
                nWords16 = nBytes / 2; //Amount of words in our buffer to read
                if (nWords16 > 0)
                {
                  streaming_status_map[pvID] = "Successfully initialized and streaming data";
                }
                else
                {
                  streaming_status_map[pvID] = "Successfully initialized but no data in buffer";
                }
                //std::cout << "No. of words read in from the stream (after removing header and footer) : " << nWords16 << std::endl; 

                doCallbacksInt16Array((epicsInt16*)(buf + 8), nWords16, waveform_param_index, 0);

                for(int i = 0; i < MAX_BUFFER_SIZE; i++)
                {
                  /**
                   * Take data gathered from the stream and move it into an array we can play with
                   * TODO take options so we know how we'd want to modify the array
                   */

                  waveformData[i] = (int16_t)buf[i];
                  //std::cout << "Value of buffer at index " << i << ": " << buf[i] << std::endl;
                }

                //std::cout << "Size of buf: " << (sizeof(buf)/sizeof(buf[0])) << std::endl;
                //std::cout << "Size of waveformData: " << (sizeof(waveformData)/sizeof(waveformData[0])) << std::endl;
                if (lastGot > got)
                {
                  //Clear the the buffer of previously read data
                  memset(buf+got, 0, (lastGot-got)*sizeof(int8_t));
                }
                lastGot = got;
                unlock();
            }
            else
            {
                 asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WaveformReader: Received frame too small\n");
                 streaming_status_map[pvID] = "Successfully initialized but received frame too small";
                 //std::cout << "Inside the else right now" << std::endl;
            }
            //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
            //printf("Getting from the buffer required %llu milliseconds\n", duration);
            
        }
        
    //}
    /**
    catch(IntrError &e)
    {
        delete[] buf;
    }
    **/ 
    return;
}

/**
 * Override of asyn port driver default writeUInt32Digital,lets us set hardware by writing to epics records
 */
asynStatus WaveformReader::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  asynStatus status = setUIntDigitalParam(pasynUser->reason, value, mask);
  callParamCallbacks();
  printf("Driver calls write UInt32\n");
  if(pasynUser->reason == waveform_run_index)
  {
    printf("Value of %d\n", value);
    //Do a thing with the firmware here?????? I mean if I'm writing I just need to set the value don't I
    //This is for turning on and off so this should in theory be the TriggerHardwareAutoRearm
    //Essentially just using my ScalVal interface to tell the address to turn on or off, should be that simple according to Jeremy
    _TriggerHwAutoRearm->setVal((int64_t)value);
  }
  if(pasynUser->reason == waveform_init_index)
  {
    _Web0Init->execute();
  }

  return status;
}


/**
 * Override of asyn port driver default writeInt32,lets us set hardware by writing to epics records
 */
asynStatus WaveformReader::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  printf("Driver calls write Int32\n");
  asynStatus status = setIntegerParam(pasynUser->reason, value);
  callParamCallbacks();
  if(pasynUser->reason == waveform_endAddr_index)
  {
    _Web0EndAddr->setVal((int64_t)value);
    _Web0Init->execute();
  }
  if(pasynUser->reason == waveform_beginAddr_index)
  {
    _Web0StartAddr->setVal((int64_t)value);
  }
  if(pasynUser->reason == waveform_buffer_size_index)
  {
    _DataBufferSize->setVal((int64_t)value);
    MAX_BUFFER_SIZE = 4 * value ;
  }
  return status;
}

//-------------------------------------------------------------------------------------
//IOCSH commands
//-------------------------------------------------------------------------------------

int waveformReaderConfigure(const char* portName, int bufferSize, int waveformPVs)
{
  WaveformReader *channelManager = new WaveformReader(portName, bufferSize, waveformPVs);
  bayManager = channelManager;

  return asynSuccess;
}
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"bufferSize", iocshArgInt};
static const iocshArg initArg2 = {"waveformPVs", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg1};
static const iocshFuncDef initFuncDef = {"waveformReaderConfigure", 3, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
  waveformReaderConfigure(args[0].sval, args[1].ival, args[2].ival);
}
void waveformReaderRegister(void)
{
  iocshRegister(&initFuncDef, initCallFunc);
}

static void waveformStreamInit(std::string pvID, std::string file_path) {
  bayManager->streamInit(pvID, file_path);
  return;
}

static const iocshArg streamArg0 = {"[channel]", iocshArgString};
static const iocshArg streamArg1 = {"[waveform pv ID]", iocshArgString};
static const iocshArg * const streamArgs[] = {&streamArg0, &streamArg1};
static const iocshFuncDef optFuncDef = {"waveformStreamInit", 2, streamArgs};
static void optCallFunc(const iocshArgBuf *args)
{
  waveformStreamInit(args[1].sval, args[0].sval);
}
void waveformStreamRegister(void)
{
  iocshRegister(&optFuncDef, optCallFunc);
}

static void waveformStatus(void)
{
  bayManager->statusCheck();
  return;
}
static const iocshFuncDef statusFuncDef = {"waveformStatus", 0};
static void statusCallFunc(const iocshArgBuf *args)
{
  waveformStatus();
}

void waveformStatusRegister(void)
{
  iocshRegister(&statusFuncDef, statusCallFunc);
}

static void printHelp() 
{
  std::cout << "Supported waveform commands:" << std::endl
    << "waveformStreamInit" << std::endl
    << "Usage: waveformStreamInit [channel] [waveform pv ID]" << std::endl
    << "[channel] is the path to the stream that cpsw can find, eg '/Stream0' " << std::endl
    << "[waveform pv ID] refers to the asyn string identifier for a given record, generally located in OUT or INP field" <<std::endl
    << std::endl
    << "waveformStatus" << std::endl
    << "Usage: waveformStatus" << std::endl
    << "Health check of initialized streams." << std::endl;
}

static const iocshFuncDef helpFuncDef = {"printHelp", 0};
static void helpCallFunc(const iocshArgBuf *args)
{
  printHelp();
}

void printHelpRegister(void)
{
  iocshRegister(&helpFuncDef, helpCallFunc);
}

static void fourierTransform(void)
{
  bayManager->fft();
  return;
}
static const iocshFuncDef fftFuncDef = {"fourierTransform", 0};
static void fftCallFunc(const iocshArgBuf *args)
{
  fourierTransform();
}

void fourierTransformRegister(void)
{
  iocshRegister(&fftFuncDef, fftCallFunc);
}


extern "C" {
  epicsExportRegistrar(printHelpRegister);
  epicsExportRegistrar(waveformStatusRegister);
  epicsExportRegistrar(waveformReaderRegister);
  epicsExportRegistrar(waveformStreamRegister);
  epicsExportRegistrar(fourierTransformRegister);
}

