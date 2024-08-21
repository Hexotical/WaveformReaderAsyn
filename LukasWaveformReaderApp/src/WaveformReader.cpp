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
WaveformReader::WaveformReader(const char *portName, int bayNumber, int bufferSize, int waveformPVs) : asynPortDriver
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

  //We register some useful hardware interfaces our port driver could want to know
  Path p;
  p = cpswGetRoot();
  
  _TriggerHwAutoRearm = IScalVal::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/TriggerHwAutoRearm").c_str()));
  _DataBufferSize = IScalVal::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/DataBufferSize").c_str()));
  _TrigCount = IScalVal_RO::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/TrigCount").c_str()));
  _WebInit = ICommand::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/Initialize").c_str()));
 
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
    // connect to the PVs that represent parameters of each waveform record using corresponding arrays
    createParam(("INITIALIZE" + std::to_string(pvID)).c_str(), asynParamUInt32Digital, init_indices[pvID]);
    createParam(("END_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, endAddr_indices[pvID]);
    createParam(("BEGIN_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, beginAddr_indices[pvID]);
    createParam(("BUFFER_SIZE" + std::to_string(pvID)).c_str(), asynParamInt32, buffer_size_indices[pvID]);
  
    (*(start_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/StartAddr[" + std::to_string(pvID) + "]").c_str()));
    (*(end_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/EndAddr[" + std::to_string(pvID) + "]").c_str()));

    //retrieve hardware addresses and store them into corresponding records
    uint32_t u32_begin, u32_end;
    (*(start_addresses[pvID]))->getVal(&u32_begin, 1);
    std::cout << "u32_begin is " << u32_begin << std::endl;
    setIntegerParam(*(beginAddr_indices[pvID]), u32_begin);
    (*(end_addresses[pvID]))->getVal(&u32_end, 1);
    std::cout << "u32_end is " << u32_end << std::endl;
    setIntegerParam(*(endAddr_indices[pvID]), u32_end);
    callParamCallbacks();
    //int value;
    //getIntegerParam(*(beginAddr_indices[pvID]), &value);
    //std::cout << "THe value is: " << value << std::endl;

  }
  //TODO: Do this is a more systematic way, individually connecting isn't really aesthetic 
  createParam(WAVEFORM_RUN_STRING, asynParamUInt32Digital, &waveform_run_index);
  createParam(NO_OF_WORDS_STRING, asynParamInt32, &number_of_words_index);
  MAX_BUFFER_SIZE = bufferSize; //One of the parameters we pass to our port driver is the bufferSize, which is essentially how many words of information we want at a time

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

/**
 * Computes and displays the results of a fast fourier transform on a section of the waveform data 
 * that contains the peak value
 * Uses the fftw library (https://www.fftw.org/)
 */
void WaveformReader::fft(void)
{
  int maxIndex = findMaxIndex();
  int low, high;
  const int LOWER_LIMIT = 5;
  findRange(low, high, maxIndex, LOWER_LIMIT);
  // Define the length of the complex arrays
  int n = high - low + 1;
  // Dynamically allocate the array because the size can change
  // Input array
  fftw_complex* x = new fftw_complex[n]; // This is equivalent to: double x[n][2];
  // Output array
  fftw_complex* y = new fftw_complex[n];
  // Samping frequency
  double sampling_frequency = 3.57142857143e8; 

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
  int choice;
  std::string extra;
  std::cout << "Would you like to: \n1) Display the results of the fast fourier transform\n2) Output the results to a csv and display a graph of the same" << std::endl;
  while (true)
  {
    std::cout << "Enter choice (1 or 2): ";
    std::cin >> choice;

    if (std::cin.fail())
    {
      std::cout << "Invalid choice, try again." << std::endl;
      std::cin.clear();
      std::cin.ignore();
      std::getline(std::cin, extra); 
    }

    else if (choice == 1)
    {
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
      break;
    }

    else if (choice == 2)
    {
      double frequency;
      for (int i = 0; i < n; i++)
      {
        frequency = (i * sampling_frequency) / n;
        std::cout << frequency << "," << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2)) << "\n";
      }
      std::cout << "\nRun the plot.py script to generate the graph.\n" << std::endl;
      break;
    }
    else 
    {
      std::cout << "Invalid choice, try again." << std::endl;
      std::cin.clear();
      std::cin.ignore();
    }  
  }
}

/**
 * Finds the index of the peak or global maximum value in waveformData
 * 
 * @return the index of the peak value
 */
int WaveformReader::findMaxIndex(void)
{
  int maxIndex = waveformData[0];

  for (int i = 0; i < STREAM_MAX_SIZE; i++)
  {
    if (waveformData[i] > maxIndex) {maxIndex = i;}
  }
  return maxIndex;
}

/**
 * Determines the relevant window from the entire waveform data on which data analysis is to be perfomed
 * 
 * @param low the starting point of the window
 * @param high the ending point of the window
 * @param maxIndex the index of the peak value of the waveform data
 * @param LOWER_LIMIT the smallest value of a local maxima that the window will include
 */
void WaveformReader::findRange(int& low, int& high, int maxIndex, const int LOWER_LIMIT)
{
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

/**
 * Finds the indices of all the local maxima of the waveform data and stores them in local_maxima_indices
 */
void WaveformReader::findLocalMaxima(void)
{
  if (waveformData[0] > waveformData[1]) {local_maxima_indices.push_back(0);}

  for(int i = 1; i < (STREAM_MAX_SIZE - 1); i++) 
  { 
         
    if ((waveformData[i - 1] < waveformData[i]) && (waveformData[i] > waveformData[i + 1])) 
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
  if((pasynUser->reason == waveform0_init_index) || (pasynUser->reason == waveform1_init_index) || (pasynUser->reason == waveform2_init_index))
  {
    _WebInit->execute();
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

  // execute instructions at hardware addresses based on which parameter of which waveform record
  // pasynUser matches with
  for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
  {
    if(pasynUser->reason == (*endAddr_indices[i]))
    {
      (*(end_addresses[i]))->setVal((int64_t)value);
      _WebInit->execute();
      std::cout << "endAddr call" << std::endl;
    }

    if(pasynUser->reason == (*beginAddr_indices[i]))
    {
      (*(start_addresses[i]))->setVal((int64_t)value);
      std::cout << "startAddr call" << std::endl;
    }

    if(pasynUser->reason == (*buffer_size_indices[i]))
    {
      _DataBufferSize->setVal((int64_t)value);
      MAX_BUFFER_SIZE = 4 * value ;
      std::cout << "bufferSizecall" << std::endl;
    }
  }

  // if the NO:OF:WORDS record updates
  if(pasynUser->reason == number_of_words_index)
  {
    int number_of_words;
    getIntegerParam(number_of_words_index, &number_of_words);

    int beginAddress, endAddress;

    for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
    {
      // get the beginning address of the stream
      getIntegerParam(*(beginAddr_indices[i]), &beginAddress);
        
      endAddress = beginAddress + (2 * number_of_words);
      // this function calls writeInt32 and thus it writes to the corresponding hardware address
      setIntegerParam(*(endAddr_indices[i]), endAddress);

      callParamCallbacks();

      // set the value of the hardware
      //(*(end_addresses[i]))->setVal(endAddress);
      
    }
    // initialize once for the entire bay
    //_WebInit->execute();
  }

  return status;
}

//-------------------------------------------------------------------------------------
//IOCSH commands
//-------------------------------------------------------------------------------------

int waveformReaderConfigure(const char* portName, int bayNumber, int bufferSize, int waveformPVs)
{
  WaveformReader *channelManager = new WaveformReader(portName, bayNumber, bufferSize, waveformPVs);
  bayManager = channelManager;

  return asynSuccess;
}
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"bayNumber", iocshArgInt};
static const iocshArg initArg2 = {"bufferSize", iocshArgInt};
static const iocshArg initArg3 = {"waveformPVs", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2, &initArg3};
static const iocshFuncDef initFuncDef = {"waveformReaderConfigure", 4, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
  waveformReaderConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
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














// //WaveformReader.cpp

// #include "WaveformReader.h"

// void streamTask(void * driverPointer); 
// void streamInit(void *driverPointer); 
// void streamInit(int channel);

// WaveformReader *bayManager; //Global pointer so iocshell commands can interact with our initialized port driver

// /**
//  * Initialize an ASYN Port Driver
//  *
//  * @param portName port for the asyn driver to use
//  * @param bayNumber bay from which the asyn port driver streams data
//  * @param bufferSize amount of words to read from the buffer
//  * @param waveformPVs amount of EPICS waveform records our port driver should be of
//  */
// WaveformReader::WaveformReader(const char *portName, int bufferSize, int waveformPVs) : asynPortDriver
//                                                        (
//                                                         portName,
//                                                         1,//Max Signals?
//                                                         asynDrvUserMask | asynInt32ArrayMask | asynInt16ArrayMask | asynUInt32DigitalMask | asynInt32Mask | asynFloat64Mask,
//                                                         asynInt32ArrayMask | asynInt16ArrayMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,
//                                                         ASYN_MULTIDEVICE | ASYN_CANBLOCK,
//                                                         1,
//                                                         0,
//                                                         0
//                                                         )
// {
//   waveformData = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 

  

//   //Path p;
//   //p = cpswGetRoot();
//   // _TriggerHwAutoRearm = IScalVal::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/TriggerHwAutoRearm").c_str()));
//   // _DataBufferSize = IScalVal::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/DataBufferSize").c_str()));
//   // _TrigCount = IScalVal_RO::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(bayNumber) + "]/TrigCount").c_str()));
//   // _WebInit = ICommand::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/Initialize").c_str()));
 

  
 

//   //Connecting to the records our port driver will eventually need to interact with
//   for(int pvID = 0; pvID < waveformPVs; pvID++)
//   {
//     //For loop generates the string identifier for each Waveform records and then creates a parameter our asynDriver can interact with for it
//     int waveform_param_index;
//     std::string pvIdentifier = "WAVEFORM:" + std::to_string(pvID);
//     std::cout << pvIdentifier << std::endl;
//     createParam(pvIdentifier.c_str(), asynParamInt16Array, &waveform_param_index);
//     std::cout << "The identifier is: " << pvIdentifier << " and the waveform_param_index is : " << waveform_param_index << std::endl;
//     pv_param_map.insert(std::pair<std::string, int>(pvIdentifier, waveform_param_index));
//     waveform_param_indices.push_back(pvIdentifier);
//     streaming_status_map[pvIdentifier] = "Not initialized yet";
//     // connect to the PVs that represent parameters of each waveform record using corresponding arrays
//     createParam(("INITIALIZE" + std::to_string(pvID)).c_str(), asynParamUInt32Digital, init_indices[pvID]);
//     createParam(("END_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, endAddr_indices[pvID]);
//     createParam(("BEGIN_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, beginAddr_indices[pvID]);
//     createParam(("BUFFER_SIZE" + std::to_string(pvID)).c_str(), asynParamInt32, buffer_size_indices[pvID]);
    
//     //createParam(("START_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, start_loc_indices[pvID]);
//     //createParam(("END_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, end_loc_indices[pvID]);
//     //createParam(("BEAM_LOSS_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, beam_loss_loc_indices[pvID]);

//     // (*(start_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/StartAddr[" + std::to_string(pvID) + "]").c_str()));
//     // (*(end_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/EndAddr[" + std::to_string(pvID) + "]").c_str()));

//     //(*(start_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[" + std::to_string(pvID) + "]").c_str()));
//     //(*(end_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[" + std::to_string(pvID) + "]").c_str()));

//     // retrieve hardware addresses and store them into corresponding records
//     //uint32_t u32;
//     //(*(start_addresses[pvID]))->getVal(&u32, 1);
//     //setIntegerParam(*(beginAddr_indices[pvID]), u32);

//   }
//   //TODO: Do this is a more systematic way, individually connecting isn't really aesthetic 
//   createParam(WAVEFORM_RUN_STRING, asynParamUInt32Digital, &waveform_run_index);

//   createParam(NO_OF_WORDS_STRING, asynParamInt32, &number_of_words_index);

//   MAX_BUFFER_SIZE = bufferSize; //One of the parameters we pass to our port driver is the bufferSize, which is essentially how many words of information we want at a time

//   //We register some useful hardware interfaces our port driver could want to know

//   Path p;
//   p = cpswGetRoot();
//   _TriggerHwAutoRearm = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TriggerHwAutoRearm"));
//   _DataBufferSize = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/DataBufferSize"));
//   _TrigCount = IScalVal_RO::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TrigCount"));
//   _WebInit = ICommand::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/Initialize"));
//   //TODO: Do these for the other hardware addresses that are useful
//   _Web0StartAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[0]"));
//   _Web0EndAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[0]"));
//   _Web1StartAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[1]"));
//   _Web1EndAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[1]"));
//   _Web2StartAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[2]"));
//   _Web2EndAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[2]"));

//   //setIntegerParam(waveform0_beginAddr_index, 6);
//   //setIntegerParam(waveform0_endAddr_index, 6);
//   // CHECK THIS: MAYBE THE ISSUE IS NOT CONVERTING THE FIRST PARAMETER TO A C_STR
//   //createParam("FFT_MAGNITUDES0", asynParamFloat64Array, &waveform0_fft_magnitudes_index);

//   // createParam(WAVEFORM0_STARTING_LOCATION_STRING, asynParamInt32, &waveform0_start_loc_index);
//   // createParam(WAVEFORM0_ENDING_LOCATION_STRING, asynParamInt32, &waveform0_end_loc_index);
//   // createParam(WAVEFORM1_STARTING_LOCATION_STRING, asynParamInt32, &waveform1_start_loc_index);
//   // createParam(WAVEFORM1_ENDING_LOCATION_STRING, asynParamInt32, &waveform1_end_loc_index);
//   // createParam(WAVEFORM2_STARTING_LOCATION_STRING, asynParamInt32, &waveform2_start_loc_index);
//   // createParam(WAVEFORM2_ENDING_LOCATION_STRING, asynParamInt32, &waveform2_end_loc_index);

// }

// //TODO Make this actually useful, tell us what streams are currently connected and meant to be streaming, if they're doing that successfully or not
// /**
//  * Check the health of any stream our port driver is connected to
//  */
// void WaveformReader::statusCheck(void)
// {
//   std::cout << "Status of connected waveforms: " << std::endl;
//   for(std::string paramIndex:waveform_param_indices)
//   {
//     std::cout << "------------------------------------------------------------------------" << std::endl;
//     std::cout << "| " << std::setw(15) << paramIndex << " | " << std::setw(50) << streaming_status_map[paramIndex] << " |" << std::endl;
//   }
//   std::cout << "------------------------------------------------------------------------" << std::endl;
// }

// /**
//  * Computes and displays the results of a fast fourier transform on a section of the waveform data 
//  * that contains the peak value
//  * Uses the fftw library (https://www.fftw.org/)
//  */
// void WaveformReader::fft(void)
// {
//   int maxIndex = findMaxIndex();
//   int low, high;
//   const int LOWER_LIMIT = 5;
//   findRange(low, high, maxIndex, LOWER_LIMIT);
//   // Define the length of the complex arrays
//   int n = high - low + 1;
//   std::cout << "n is: " << n << std::endl;
//   // Dynamically allocate the array because the size can change
//   // Input array
//   //DEALLOCATE THIS MEMORY
//   fftw_complex* x = new fftw_complex[n]; // This is equivalent to: double x[n][2];
//   // Output array
//   fftw_complex* y = new fftw_complex[n];
//   // Samping frequency
//   double sampling_frequency = 3.57142857143e8; 

//   // Fill the first array with data from waveformData
//   for (int i = 0; i < n; i++)
//   {
//       x[i][REAL] = waveformData[i + low];
//       x[i][IMAG] = 0;
//   }
//   //Plant the FFT and execute it
//   fftw_plan plan = fftw_plan_dft_1d(n, x, y, FFTW_FORWARD, FFTW_ESTIMATE);
//   fftw_execute(plan);
//   //Do some cleaning
//   fftw_destroy_plan(plan);
//   fftw_cleanup();

//   // Display the results
//   int choice;
//   std::string extra;
//   //double* magnitude_array = new double[5];
//   //for (int i = 0; i < 5; i++)
//   //{
//   //  magnitude_array[i] = 0.0;
//   //}
//   std::cout << "Would you like to: \n1) Display the results of the fast fourier transform\n2) Output the results to a csv and display a graph of the same" << std::endl;
//   while (true)
//   {
//     std::cout << "Enter choice (1 or 2): ";
//     std::cin >> choice;


//     if (std::cin.fail())
//     {
//       std::cout << "Invalid choice, try again." << std::endl;
//       std::cin.clear();
//       std::cin.ignore();
//       std::getline(std::cin, extra); 
//     }

//     else if (choice == 1)
//     {
//       std::cout << "FFT = " << std::endl;
//       double frequency;
//       std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
//       std::cout << "|    No.    | FREQUENCY (in Hz) |             FFT RESULT             |     MAGNITUDE     | PHASE (in radians) |" << std::endl;
//       std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
//       for (int i = 0; i < n; i++)
//       {
//           frequency = (i * sampling_frequency) / n;
//           //magnitude_array[i] = frequency;
//           //std::cout << "This is the frequency: " << magnitude_array[i] << std::endl;
//           if (y[i][IMAG] < 0)
//           {
//             std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " - " << std::setw(16) << std::left << abs(y[i][IMAG]) << "i";
//           }
//           else
//           {
//             std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " + " << std::setw(16) << std::left << y[i][IMAG] << "i" ;
//           }
//           std::cout << "|" << std::setw(19) << std::right << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2));
//           std::cout << "|" << std::setw(20) << std::right << atan2(y[i][IMAG], y[i][REAL]) << "|" << std::endl;
//           std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
//       }
//       break;
//     }

//     else if (choice == 2)
//     {
//       double frequency;
//       for (int i = 0; i < n; i++)
//       {
//         frequency = (i * sampling_frequency) / n;
//         std::cout << frequency << "," << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2)) << "\n";
//       }
//       std::cout << "\nRun the plot.py script to generate the graph.\n" << std::endl;
//       break;
//     }
//     else 
//     {
//       std::cout << "Invalid choice, try again." << std::endl;
//       std::cin.clear();
//       std::cin.ignore();
//     }  
//   }
//   std::cout << "Am I here now" << std::endl;
//   //doCallbacksFloat64Array(magnitude_array, 5, waveform0_fft_magnitudes_index, 0);
//   std::cout << "THIS PLACE" << std::endl;
//   callParamCallbacks();
// }

// /**
//  * Finds the index of the peak or global maximum value in waveformData
//  * 
//  * @return the index of the peak value
//  */
// int WaveformReader::findMaxIndex(void)
// {
//   int maxIndex = 0;

//   for (int i = 1; i < MAX_BUFFER_SIZE; i++)
//   {
//     if (waveformData[i] > waveformData[maxIndex]) 
//     {
//       maxIndex = i;
//     }
//   }
//   return maxIndex;
// }

// /**
//  * Determines the relevant window from the entire waveform data on which data analysis is to be perfomed
//  * 
//  * @param low the starting point of the window
//  * @param high the ending point of the window
//  * @param maxIndex the index of the peak value of the waveform data
//  * @param LOWER_LIMIT the smallest value of a local maxima that the window will include
//  */
// void WaveformReader::findRange(int& low, int& high, int maxIndex, const int LOWER_LIMIT)
// {
//   low = maxIndex - 1;
//   high = maxIndex + 1;
//   while ((waveformData[low - 1] <= waveformData[low] || waveformData[low] > LOWER_LIMIT) && (low > 0))
//   {
//     low--;
//   } 
//   while ((waveformData[high + 1] <= waveformData[high] || waveformData[high] > LOWER_LIMIT) && (high < (MAX_BUFFER_SIZE- 1)))
//   {
//     high++;
//   } 

// }

// /**
//  * Finds the indices of all the local maxima of the waveform data and stores them in local_maxima_indices
//  */
// void WaveformReader::findLocalMaxima(void)
// {
//   if (waveformData[0] > waveformData[1]) {local_maxima_indices.push_back(0);}

//   for(int i = 1; i < (MAX_BUFFER_SIZE - 1); i++) 
//   { 
         
//     if ((waveformData[i - 1] < waveformData[i]) && (waveformData[i] > waveformData[i + 1])) 
//     {
//       local_maxima_indices.push_back(i);
//     } 

//   }

//   if (waveformData[MAX_BUFFER_SIZE - 1] > waveformData[MAX_BUFFER_SIZE - 2]) 
//   {
//     local_maxima_indices.push_back(MAX_BUFFER_SIZE - 1);
//   }
 
// }

// /**
//  * Computes and displays the location of the maximum beam loss detected by the monitor
//  * 
//  * @param startingPosition the starting position of the beam loss monitor (BLM)
//  * @param endingPosition the ending position of the beam loss monitor (BLM)
//  * @param bufferSize size of the buffer read by the sensors in the BLM
//  */
// // ADD INPUT VALIDATION
// void WaveformReader::maxBeamLoss(int waveformIndex)
// {
//   double startingPosition, endingPosition;
//   getDoubleParam(*(start_loc_indices[waveformIndex]), &startingPosition);
//   getDoubleParam(*(end_loc_indices[waveformIndex]), &endingPosition);
//   std::cout << "Starting position and ending position: " << startingPosition << " " << endingPosition << std::endl;
//   double lengthOfMonitor = endingPosition - startingPosition;
//   std::cout << "Length is: " << lengthOfMonitor << std::endl;

//   // UPDATE FUNCTION TO USE WAVEFORM_INDEX FOR WAVEFORM SPECIFIC COMPUTATION
//   int maxIndex = findMaxIndex();
//   std::cout << "Max index is: " << maxIndex << std::endl;
//   // CHANGE 1 MIL TO BUFFER SIZE OF CORRESPONDING WAVEFORM
//   int bufferSize;
//   getIntegerParam(number_of_words_index, &bufferSize);
//   std::cout << "The buffer size is: " << bufferSize << std::endl;
//   double locationOfMaxIndex = (maxIndex * (lengthOfMonitor / bufferSize)) + startingPosition;
//   std::cout << "The location of maximum beam loss is " << locationOfMaxIndex << std::endl;
//   setDoubleParam(*(beam_loss_loc_indices[waveformIndex]), locationOfMaxIndex);
// }


// /**
//  * Launch a thread to stream data to an EPICS records
//  *
//  * @param pv_identifier string corresponding to the Epics record we wish to have data in
//  * @param stream_path string form of the path to the stream we want to connect to
//  *
//  * Does not return
//  */
// void WaveformReader::streamInit(std::string pv_identifier, std::string stream_path)
// {
//   StreamArgs toPass; //Initialize a structure to pass the arguments we need to begin streaming

//   toPass.pPvt = this;
//   toPass.pv_identifier = pv_identifier;
//   toPass.stream_path_to_find = stream_path; 

//   //std::cout << "The path we pass is: " << toPass.stream_path_to_find << std::endl;
//   //printf("\nChannel number in toPass is %d\n" ,toPass.stream);

//   asynStatus status;
//   status = (asynStatus)(epicsThreadCreate("WaveformTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::streamTask, &toPass) == NULL);
//   if(status == asynError)
//   {
//     std::cout << "Unable to launch a waveform stream; " << status << std::endl;
//   }
//   else
//   {
//     std::cout << "Succesfully launched waveform stream=> " << pv_identifier << " status: " << status << std::endl;
//   }
//   sleep(3); //Sleep so the launched thread can find the structure before it's overwritten by garbage TODO: Do this in a better way 
// }

// /**
//  * Tell a port driver to use it's streamTask method with the passed args generally called by streamInit
//  *
//  * @param streamArgs port driver and the required args to begin streaming
//  */
// void streamTask(void* streamArgs)
// {
//   StreamArgs *passedArgs = static_cast<StreamArgs*>(streamArgs);

//   WaveformReader *pPvt = (WaveformReader *) passedArgs->pPvt;
//   pPvt->streamTask(passedArgs->stream_path_to_find.c_str(), passedArgs->pv_identifier);
// }

// /**
//  * Connect to a stream, write the data retrieved from the stream to the specified EPICS record
//  *
//  * @param streamInit path to the stream to connect and read from
//  * @param pvID Identifier of the EPICS record to write data from the stream to
//  */
// void WaveformReader::streamTask(const char *streamInit = "/Stream0", std::string pvID = "WAVEFORM:0")//, int waveform_param_index = -1)//Stream stm, int param16index, int param32index)
// {
//         sleep(1);

//         //TODO based on streamInit, add key and param to the asynPortDriver

//         std::cout << "Passed pvID: " << pvID << std::endl;
//         int waveform_param_index = pv_param_map[pvID];
//         std::cout << pvID << " corresponding index: " << waveform_param_index << std::endl;

//         Path p;
//         p = cpswGetRoot();

//         std::cout << "Stream Init (path to the stream): " << streamInit << std::endl;
//         Stream stm;

//         try {
//           stm = IStream::create(p->findByName(streamInit));
//         }
//         catch (CPSWError &e) {
//           std::cerr << e.what() << std::endl;
//         }

//         if(stm)
//         {
//           std::cout << "Value of stm: " << stm << std::endl;
//           printf("Did that thing with a stream?\n");
//           streaming_status_map[pvID] = "Successfully initialized"; 
//         }
//         else {
//           printf("No stream access");
//           streaming_status_map[pvID] = "Initialization failed";
//           return;
//         }

//         uint8_t *buf = new uint8_t[STREAM_MAX_SIZE];
//         size_t nWords16, nBytes;
//         int64_t got = 0;
//         int64_t lastGot = 0;

//         std::cout << "Outside the while loop now " << std::endl;

//         while(1)
//         {
//             std::cout << "Inside the while loop " << std::endl;
//             //std::cout << streamInit << std::endl;
//             //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//             got = stm->read( buf, MAX_BUFFER_SIZE, CTimeout(-1));
//             //std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
//             std::cout << "No. of values read in from the stream (value of got): " << got << std::endl;
//             //std::cout << "Value of lastGot: " << lastGot << std::endl;

//             if(got > 8)
//             {
//                 //printf("There's a thing in the stream? %ld bytes\n", got);
//                 lock();
//                 nBytes = (got - 9); // header = 8 bytes, footer = 1 byte, data = 32bit words.
//                 nWords16 = nBytes / 2; //Amount of words in our buffer to read
//                 if (nWords16 > 0)
//                 {
//                   streaming_status_map[pvID] = "Successfully initialized and streaming data";
//                 }
//                 else
//                 {
//                   streaming_status_map[pvID] = "Successfully initialized but no data in buffer";
//                 }
//                 //std::cout << "No. of words read in from the stream (after removing header and footer) : " << nWords16 << std::endl; 

//                 doCallbacksInt16Array((epicsInt16*)(buf + 8), nWords16, waveform_param_index, 0);

//                 for(int i = 0; i < MAX_BUFFER_SIZE; i++)
//                 {
//                   /**
//                    * Take data gathered from the stream and move it into an array we can play with
//                    * TODO take options so we know how we'd want to modify the array
//                    */

//                   waveformData[i] = (int16_t)buf[i];
//                   //std::cout << "Value of buffer at index " << i << ": " << buf[i] << std::endl;
//                 }

//                 //std::cout << "Size of buf: " << (sizeof(buf)/sizeof(buf[0])) << std::endl;
//                 //std::cout << "Size of waveformData: " << (sizeof(waveformData)/sizeof(waveformData[0])) << std::endl;
//                 if (lastGot > got)
//                 {
//                   //Clear the the buffer of previously read data
//                   memset(buf+got, 0, (lastGot-got)*sizeof(int8_t));
//                 }
//                 lastGot = got;
//                 unlock();
//             }
//             else
//             {
//                  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WaveformReader: Received frame too small\n");
//                  streaming_status_map[pvID] = "Successfully initialized but received frame too small";
//                  //std::cout << "Inside the else right now" << std::endl;
//             }
//             //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//             //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
//             //printf("Getting from the buffer required %llu milliseconds\n", duration);
            
//         }
        
//     //}
//     /**
//     catch(IntrError &e)
//     {
//         delete[] buf;
//     }
//     **/ 
//     return;
// }

// /**
//  * Override of asyn port driver default writeUInt32Digital,lets us set hardware by writing to epics records
//  */
// asynStatus WaveformReader::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
// {
//   asynStatus status = setUIntDigitalParam(pasynUser->reason, value, mask);
//   callParamCallbacks();
//   printf("Driver calls write UInt32\n");
//   if(pasynUser->reason == waveform_run_index)
//   {
//     printf("Value of %d\n", value);
//     //Do a thing with the firmware here?????? I mean if I'm writing I just need to set the value don't I
//     //This is for turning on and off so this should in theory be the TriggerHardwareAutoRearm
//     //Essentially just using my ScalVal interface to tell the address to turn on or off, should be that simple according to Jeremy
//     _TriggerHwAutoRearm->setVal((int64_t)value);
//   }
//   if((pasynUser->reason == waveform0_init_index) || (pasynUser->reason == waveform1_init_index) || (pasynUser->reason == waveform2_init_index))
//   {
//     _WebInit->execute();
//   }

//   return status;
// }


// /**
//  * Override of asyn port driver default writeInt32,lets us set hardware by writing to epics records
//  */
// asynStatus WaveformReader::writeInt32(asynUser *pasynUser, epicsInt32 value)
// {
//   printf("Driver calls write Int32\n");
//   asynStatus status = setIntegerParam(pasynUser->reason, value);
//   callParamCallbacks();

//   // execute instructions at hardware addresses based on which parameter of which waveform record
//   // pasynUser matches with
//   for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
//   {
//     if(pasynUser->reason == (*(endAddr_indices[i])))
//     {
//       (*(end_addresses[i]))->setVal((int64_t)value);
//       _WebInit->execute();
//     }

//     if(pasynUser->reason == (*(beginAddr_indices[i])))
//     {
//       (*(start_addresses[i]))->setVal((int64_t)value);
//     }

//     if(pasynUser->reason == (*(buffer_size_indices[i])))
//     {
//       _DataBufferSize->setVal((int64_t)value);
//       MAX_BUFFER_SIZE = 4 * value ;
//     }
//   }

//   // if the NO:OF:WORDS record updates
//   /*if(pasynUser->reason == number_of_words_index)
//   {
//     int number_of_words;
//     getIntegerParam(number_of_words_index, &number_of_words);

//     int beginAddress, endAddress;

//     for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
//     {
//       // get the beginning address of the stream
//       getIntegerParam(*(beginAddr_indices[i]), &beginAddress);
        
//       endAddress = beginAddress + (2 * number_of_words);
//       // this function calls writeInt32 and thus it writes to the corresponding hardware address
//       setIntegerParam(*(endAddr_indices[i]), endAddress);

//       callParamCallbacks();

//       // set the value of the hardware
//       //(*(end_addresses[i]))->setVal(endAddress);
      
//     }
//     // initialize once for the entire bay
//     //_WebInit->execute();
//   }  */
//   return status;
// }

// //-------------------------------------------------------------------------------------
// //IOCSH commands
// //-------------------------------------------------------------------------------------

// // int waveformReaderConfigure(const char* portName, int bayNumber, int bufferSize, int waveformPVs)
// // {
// //   WaveformReader *channelManager = new WaveformReader(portName, bayNumber, bufferSize, waveformPVs);
// //   bayManager = channelManager;

// //   return asynSuccess;
// // }
// // static const iocshArg initArg0 = {"portName", iocshArgString};
// // static const iocshArg initArg1 = {"bayNumber", iocshArgInt};
// // static const iocshArg initArg2 = {"bufferSize", iocshArgInt};
// // static const iocshArg initArg3 = {"waveformPVs", iocshArgInt};
// // static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2, &initArg3};
// // static const iocshFuncDef initFuncDef = {"waveformReaderConfigure", 4, initArgs};
// // static void initCallFunc(const iocshArgBuf *args)
// // {
// //   waveformReaderConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
// // }

// int waveformReaderConfigure(const char* portName, int bufferSize, int waveformPVs)
// {
//   WaveformReader *channelManager = new WaveformReader(portName, bufferSize, waveformPVs);
//   bayManager = channelManager;

//   return asynSuccess;
// }
// static const iocshArg initArg0 = {"portName", iocshArgString};
// static const iocshArg initArg1 = {"bufferSize", iocshArgInt};
// static const iocshArg initArg2 = {"waveformPVs", iocshArgInt};
// static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2};
// static const iocshFuncDef initFuncDef = {"waveformReaderConfigure", 3, initArgs};
// static void initCallFunc(const iocshArgBuf *args)
// {
//   waveformReaderConfigure(args[0].sval, args[1].ival, args[2].ival);
// }

// void waveformReaderRegister(void)
// {
//   iocshRegister(&initFuncDef, initCallFunc);
// }

// static void waveformStreamInit(std::string pvID, std::string file_path) {
//   bayManager->streamInit(pvID, file_path);
//   return;
// }

// static const iocshArg streamArg0 = {"[channel]", iocshArgString};
// static const iocshArg streamArg1 = {"[waveform pv ID]", iocshArgString};
// static const iocshArg * const streamArgs[] = {&streamArg0, &streamArg1};
// static const iocshFuncDef optFuncDef = {"waveformStreamInit", 2, streamArgs};
// static void optCallFunc(const iocshArgBuf *args)
// {
//   waveformStreamInit(args[1].sval, args[0].sval);
// }
// void waveformStreamRegister(void)
// {
//   iocshRegister(&optFuncDef, optCallFunc);
// }

// static void waveformStatus(void)
// {
//   bayManager->statusCheck();
//   return;
// }
// static const iocshFuncDef statusFuncDef = {"waveformStatus", 0};
// static void statusCallFunc(const iocshArgBuf *args)
// {
//   waveformStatus();
// }

// void waveformStatusRegister(void)
// {
//   iocshRegister(&statusFuncDef, statusCallFunc);
// }

// static void printHelp() 
// {
//   std::cout << "Supported waveform commands:" << std::endl
//     << "waveformStreamInit" << std::endl
//     << "Usage: waveformStreamInit [channel] [waveform pv ID]" << std::endl
//     << "[channel] is the path to the stream that cpsw can find, eg '/Stream0' " << std::endl
//     << "[waveform pv ID] refers to the asyn string identifier for a given record, generally located in OUT or INP field" <<std::endl
//     << std::endl
//     << "waveformStatus" << std::endl
//     << "Usage: waveformStatus" << std::endl
//     << "Health check of initialized streams." << std::endl;
// }

// static const iocshFuncDef helpFuncDef = {"printHelp", 0};
// static void helpCallFunc(const iocshArgBuf *args)
// {
//   printHelp();
// }

// void printHelpRegister(void)
// {
//   iocshRegister(&helpFuncDef, helpCallFunc);
// }

// static void fourierTransform(void)
// {
//   bayManager->fft();
//   return;
// }
// static const iocshFuncDef fftFuncDef = {"fourierTransform", 0};
// static void fftCallFunc(const iocshArgBuf *args)
// {
//   fourierTransform();
// }

// void fourierTransformRegister(void)
// {
//   iocshRegister(&fftFuncDef, fftCallFunc);
// }

// static void maxBeamLossLocation(int waveformIndex) {
//   bayManager->maxBeamLoss(waveformIndex);
//   return;
// }

// static const iocshArg lossArg0 = {"waveformIndex", iocshArgInt};
// static const iocshArg * const lossArgs[] = {&lossArg0};
// static const iocshFuncDef lossFuncDef = {"maxBeamLossLocation", 1, lossArgs};
// static void lossCallFunc(const iocshArgBuf *args)
// {
//   maxBeamLossLocation(args[0].ival);
// }
// void maxBeamLossLocationRegister(void)
// {
//   iocshRegister(&lossFuncDef, lossCallFunc);
// }


// extern "C" {
//   epicsExportRegistrar(printHelpRegister);
//   epicsExportRegistrar(waveformStatusRegister);
//   epicsExportRegistrar(waveformReaderRegister);
//   epicsExportRegistrar(waveformStreamRegister);
//   epicsExportRegistrar(fourierTransformRegister);
//   epicsExportRegistrar(maxBeamLossLocationRegister);
// }

