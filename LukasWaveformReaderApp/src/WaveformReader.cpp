//WaveformReader.cpp

#include "WaveformReader.h"
#include <unistd.h>
#include <chrono>

void streamTask(void * driverPointer); //added to fix compiler errors
void streamInit(void *driverPointer); //TODO Add a stream argument so it can check on a specific stream
void streamInit(int channel);                                

WaveformReader *bayManager;

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
//Craete params here at some point
  waveformData = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16));

  for(int pvID = 0; pvID < waveformPVs; pvID++)
  {
    int waveform_param_index;
    std::string pvIdentifier = "WAVEFORM:" + std::to_string(pvID);
    std::cout << pvIdentifier << std::endl;
    createParam(pvIdentifier.c_str(), asynParamInt16Array, &waveform_param_index);
    pv_param_map.insert(std::pair<std::string, int>(pvIdentifier, waveform_param_index));
    waveform_param_indices.push_back(pvIdentifier);
  }
  
  //createParam(WAVEFORM0_PV_STRING, asynParamInt16Array, &waveform_param_index_0);
  //createParam(WAVEFORM1_PV_STRING, asynParamInt16Array, &waveform_param_index_1);
  createParam(WAVEFORM_RUN_STRING, asynParamUInt32Digital, &waveform_run_index);
  createParam(WAVEFORM0_INITIALIZE_STRING, asynParamUInt32Digital, &waveform_init_index);
  createParam(WAVEFORM0_END_ADDR_STRING, asynParamInt32, &waveform_endAddr_index);
  createParam(WAVEFORM0_BEGIN_ADDR_STRING, asynParamInt32, &waveform_beginAddr_index);
  createParam(WAVEFORM0_BUFFER_SIZE_STRING, asynParamInt32, &waveform_buffer_size_index);
  
  MAX_BUFFER_SIZE = bufferSize;

  Path p;
  p = cpswGetRoot();
  std::cout << "Path " << p;
  _TriggerHwAutoRearm = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TriggerHwAutoRearm"));
  _DataBufferSize = IScalVal::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/DataBufferSize"));
  _TrigCount = IScalVal_RO::create(p->findByName("/mmio/AppTop/DaqMuxV2[0]/TrigCount"));
  _Web0StartAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/StartAddr[0]"));
  _Web0EndAddr = IScalVal::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/EndAddr[0]"));
  _Web0Init = ICommand::create(p->findByName("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/Initialize")); 
  asynStatus status;
  //status = (asynStatus)(epicsThreadCreate("WaveformTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::streamTask, this) == NULL);
}

void streamInit(void* driverPointer)
{
  WaveformReader *pPvt = (WaveformReader *) driverPointer;
  pPvt->streamInit();
  //TODO add some registers to do a healthcheck 

}

void WaveformReader::statusCheck(void)
{
  for(std::string paramIndex:waveform_param_indices)
  {
    std::cout << paramIndex << ' ';
  }
}
void WaveformReader::streamInit(std::string pv_identifier, std::string file_path)
{
  StreamArgs toPass;
  toPass.pPvt = this;
  //toPass.stream = channel;
  toPass.pv_identifier = pv_identifier;
  toPass.stream_path_to_find = file_path; // file path is a little bit of a misnomer, path to the stream to use 
  std::cout << "The path we pass is: " << toPass.stream_path_to_find << std::endl;
  printf("\nChannel number in toPass is %d\n" ,toPass.stream);
  asynStatus status;
  status = (asynStatus)(epicsThreadCreate("WaveformTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::streamTask, &toPass) == NULL);
  sleep(3); //Sleep so the launched thread can find the structure before it's overwritten by garbage
}

void WaveformReader::streamInit(void)
{
  asynStatus status;
  //Need to pass an argument to stream task that tells it the appropriate register access
  
  StreamArgs toPass;
  toPass.pPvt = this;
  
  status = (asynStatus)(epicsThreadCreate("WaveformTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::streamTask, &toPass) == NULL);
  printf("Successful status check\n");
  //TODO Check on things we care about in the stream, if it's errored or not etc
}
 

void streamTask(void* streamArgs)
{
  StreamArgs *passedArgs = static_cast<StreamArgs*>(streamArgs);

  WaveformReader *pPvt = (WaveformReader *) passedArgs->pPvt;
  //printf("\nSuccessfulyl converted streamArgs given trying to launch streamTask.\n");
  //printf("Stored channel is %d\n", passedArgs->stream);
  pPvt->streamTask(passedArgs->stream_path_to_find.c_str(), passedArgs->pv_identifier);
  //std::cout << passedArgs->streamPath << std::endl;
  //pPvt->streamTask(passedArgs->streamPath); //TODO add args
}
 
 /**
//This function is called when creating a thread do read for some reason.
void streamTask(void* waveformPointer)
{
  WaveformReader *pPvt = (WaveformReader *) waveformPointer;
  const char* toPass = "/Stream1";
  pPvt->streamTask(toPass);
}
**/ 

void WaveformReader::streamTask(const char *streamInit = "/Stream0", std::string pvID = "WAVEFORM:0")//, int waveform_param_index = -1)//Stream stm, int param16index, int param32index)
{
        sleep(1);

        //TODO based on streamInit, add key and param to the asynPortDriver
        
        createParamMutex.lock();

        std::cout << "Passed pvID: " << pvID << std::endl;
        int waveform_param_index = pv_param_map[pvID];
        std::cout << pvID << std::endl;

        createParamMutex.unlock();


        int retrievedVal;
        printf("Max size passed = %d\n", MAX_BUFFER_SIZE);

        //_TriggerHwAutoRearm->setVal((uint64_t)1);
        Path p;
        p = cpswGetRoot();

        std::cout << "Path " << p;
        std::cout << "Stream Init: " << streamInit;
        //while(1);
        Stream stm;
        try {
          //const char *stream = "/Stream0";
          //stm = IStream::create(p->findByName("/Stream1")); //Baseline working finds a stream
          stm = IStream::create(p->findByName(streamInit));
        }
        catch (CPSWError &e) {
        }
        if(stm)
        {
          printf("Did that thing with a stream?\n");
        }
        else {
          printf("No stream access ");
        }
        //while(1);
        uint8_t *buf = new uint8_t[STREAM_MAX_SIZE];
        printf("STREAM MAX SIZE IS %d\n", STREAM_MAX_SIZE);
        size_t nWords16, nWords32, nBytes;
        int nFrame;
        int64_t got = 0;
        int64_t lastGot = 0;

        int iter = 0;
        
        while(1)
        {
            //std::cout << streamInit << std::endl;
            //getIntegerParam(MAX_STREAM_SIZE, &waveform_buffer_size_init_index);
            //printf("Max stream size %d\n", MAX_STREAM_SIZE);
            epicsInt32 run;
            iter += 1;
            /**
            MAX_BUFFER_SIZE = MAX_BUFFER_SIZE - 10000;
            if(MAX_BUFFER_SIZE <= 500000)
            {
              MAX_BUFFER_SIZE = 500000;
            }
            **/ 
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            got = stm->read( buf, MAX_BUFFER_SIZE, CTimeout(-1));
            std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
            
            //printf("We got %d bytes, iter %d, max buffer size: %d.\n", got, iter, MAX_BUFFER_SIZE);

            if(got > 8)
            {
                //std::cout << 
                //printf("Theres a thing in the stream? %ld bytes\n", got);
                lock();
                nBytes = (got - 9); // header = 8 bytes, footer = 1 byte, data = 32bit words.
                nWords16 = nBytes / 2;
                nWords32 = nWords16 / 2;

                nFrame = (buf[1]<<4) | (buf[0] >> 4);

                //Need to grab the word count from the thing, check the asyn params
                
                doCallbacksInt16Array((epicsInt16*)(buf + 8), nWords16, waveform_param_index, 0);

                for(int i = 0; i < MAX_BUFFER_SIZE; i++)
                {
                  //printf("Index %d, Value %d\n", i, buf[i]);
                  waveformData[i] = (int16_t)buf[i];
                  //std::cout << buf[i];
                }
                //doCallbacksInt16Array(waveformData, 100000, waveform_param_index, 0);

                //doCallbacksInt16Array((epicsInt16*)(buf+8), nWord16, param16index, DEV_STM);
                if (lastGot > got)
                    memset(buf+got, 0, (lastGot-got)*sizeof(int8_t));

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

extern "C" {
  epicsExportRegistrar(WaveformStatusRegister);
  epicsExportRegistrar(WaveformReaderRegister);
  epicsExportRegistrar(WaveformStreamRegister);
}

