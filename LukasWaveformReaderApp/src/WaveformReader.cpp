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
    pv_param_map.insert(std::pair<std::string, int>(pvIdentifier, waveform_param_index));
    waveform_param_indices.push_back(pvIdentifier);
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
  for(std::string paramIndex:waveform_param_indices)
  {
    std::cout << paramIndex << ' ';
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
        std::cout << pvID << std::endl;

        Path p;
        p = cpswGetRoot();

        std::cout << "Stream Init: " << streamInit;
        Stream stm;

        try {
          stm = IStream::create(p->findByName(streamInit));
        }
        catch (CPSWError &e) {
          std::cerr << e.what() << std::endl;
        }

        if(stm)
        {
          printf("Did that thing with a stream?\n");
        }
        else {
          printf("No stream access");
          return;
        }

        uint8_t *buf = new uint8_t[STREAM_MAX_SIZE];
        size_t nWords16, nBytes;
        int64_t got = 0;
        int64_t lastGot = 0;

        while(1)
        {
            //std::cout << streamInit << std::endl;
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            got = stm->read( buf, MAX_BUFFER_SIZE, CTimeout(-1));
            //std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();

            if(got > 8)
            {
                //printf("Theres a thing in the stream? %ld bytes\n", got);
                lock();
                nBytes = (got - 9); // header = 8 bytes, footer = 1 byte, data = 32bit words.
                nWords16 = nBytes / 2; //Amount of words in our buffer to read

                doCallbacksInt16Array((epicsInt16*)(buf + 8), nWords16, waveform_param_index, 0);

                for(int i = 0; i < MAX_BUFFER_SIZE; i++)
                {
                  /**
                   * Take data gathered from the stream and move it into an array we can play with
                   * TODO take options so we know how we'd want to modify the array
                   */

                  waveformData[i] = (int16_t)buf[i];
                  //std::cout << buf[i];
                }
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
            }
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
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

int WaveformReaderConfigure(const char* portName, int bufferSize, int waveformPVs)
{
  WaveformReader *channelManager = new WaveformReader(portName, bufferSize, waveformPVs);
  bayManager = channelManager;

  return asynSuccess;
}
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"bufferSize", iocshArgInt};
static const iocshArg initArg2 = {"waveformPVs", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg1};
static const iocshFuncDef initFuncDef = {"WaveformReaderConfigure", 3, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
  WaveformReaderConfigure(args[0].sval, args[1].ival, args[2].ival);
}
void WaveformReaderRegister(void)
{
  iocshRegister(&initFuncDef, initCallFunc);
}

static void WaveformStreamInit(std::string pvID, std::string file_path) {
  bayManager->streamInit(pvID, file_path);
  return;
}

static const iocshArg streamArg0 = {"[channel]", iocshArgString};
static const iocshArg streamArg1 = {"[waveform pv ID]", iocshArgString};
static const iocshArg * const streamArgs[] = {&streamArg0, &streamArg1};
static const iocshFuncDef optFuncDef = {"WaveformStreamInit", 2, streamArgs};
static void optCallFunc(const iocshArgBuf *args)
{
  WaveformStreamInit(args[1].sval, args[0].sval);
}
void WaveformStreamRegister(void)
{
  iocshRegister(&optFuncDef, optCallFunc);
}

static void WaveformStatus(void)
{
  bayManager->statusCheck();
  return;
}
static const iocshFuncDef statusFuncDef = {"WaveformStatus", 0};
static void statusCallFunc(const iocshArgBuf *args)
{
  WaveformStatus();
}

void WaveformStatusRegister(void)
{
  iocshRegister(&statusFuncDef, statusCallFunc);
}

static void PrintHelp() 
{
  std::cout << "Supported waveform commands:" << std::endl
    << "WaveformStreamInit" << std::endl
    << "Usage: WaveformStreamInit [channel] [waveform pv ID]" << std::endl
    << "[channel] is the path to the stream that cpsw can find, eg '/Stream0' " << std::endl
    << "[waveform pv ID] refers to the asyn string identifier for a given record, generally located in OUT or INP field" <<std::endl
    << std::endl
    << "WaveformStatus" << std::endl
    << "Usage: WaveformStatus" << std::endl
    << "Health check of initialized streams." << std::endl;
}

static const iocshFuncDef helpFuncDef = {"PrintHelp", 0};
static void helpCallFunc(const iocshArgBuf *args)
{
  PrintHelp();
}

void PrintHelpRegister(void)
{
  iocshRegister(&helpFuncDef, helpCallFunc);
}

extern "C" {
  epicsExportRegistrar(PrintHelpRegister);
  epicsExportRegistrar(WaveformStatusRegister);
  epicsExportRegistrar(WaveformReaderRegister);
  epicsExportRegistrar(WaveformStreamRegister);
}

