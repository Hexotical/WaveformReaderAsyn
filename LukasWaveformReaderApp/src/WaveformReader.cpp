//WaveformReader.cpp

#include "WaveformReader.h"

void streamTask(void * driverPointer); 
void streamInit(void *driverPointer); 
void streamInit(int channel);


// initialize static variable
WaveformReader* WaveformReader::port_driver = nullptr;


/**
 * Initialize an ASYN Port Driver
 *
 * @param portName port for the asyn driver to use
 * @param bayNumber bay for the asyn driver to use (0 or 1)
 * @param bufferSize amount of words to read from the buffer
 * @param waveformPVs amount of EPICS waveform records our port driver should be of
 */
WaveformReader::WaveformReader(const char *portName, int bayNumber, int bufferSize, int waveformPVs) : asynPortDriver
                                                       (
                                                        portName,
                                                        1,//Max Signals?
                                                        asynDrvUserMask | asynInt32ArrayMask | asynInt16ArrayMask | asynUInt32DigitalMask | asynInt32Mask | asynFloat64Mask,
                                                        asynInt32ArrayMask | asynInt16ArrayMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,
                                                        ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                                                        1,
                                                        0,
                                                        0
                                                        )
{

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
    // initialize values 
    index_map[pvIdentifier] = pvID;
    streaming_status[pvID] = "Not initialized yet";
    initialization_status[pvID] = false;
    duration_data[pvID] = std::chrono::milliseconds(0);

    waveform_map[pvIdentifier] = (epicsInt16 *)calloc(STREAM_MAX_SIZE, sizeof(epicsInt16)); 

    // connect to the PVs that represent parameters of each waveform record using corresponding arrays
    createParam(("END_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, endAddr_indices[pvID]);
    createParam(("BEGIN_ADDR" + std::to_string(pvID)).c_str(), asynParamInt32, beginAddr_indices[pvID]);
    createParam(("START_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, start_loc_indices[pvID]);
    createParam(("END_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, end_loc_indices[pvID]);
    createParam(("BEAM_LOSS_LOC" + std::to_string(pvID)).c_str(), asynParamFloat64, beam_loss_loc_indices[pvID]);


    (*(start_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/StartAddr[" + std::to_string(pvID) + "]").c_str()));
    (*(end_addresses[pvID])) = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[" + std::to_string(bayNumber) + "]/WaveformEngineBuffers/EndAddr[" + std::to_string(pvID) + "]").c_str()));

    //retrieve hardware addresses and store them into corresponding records
    uint32_t u32_begin, u32_end;
    (*(start_addresses[pvID]))->getVal(&u32_begin, 1);
    setIntegerParam(*(beginAddr_indices[pvID]), u32_begin);
    (*(end_addresses[pvID]))->getVal(&u32_end, 1);
    setIntegerParam(*(endAddr_indices[pvID]), u32_end);
    callParamCallbacks();

  }

  //TODO: Do this is a more systematic way, individually connecting isn't really aesthetic 
  createParam(WAVEFORM_RUN_STRING, asynParamUInt32Digital, &waveform_run_index);
  createParam(NO_OF_WORDS_STRING, asynParamInt32, &number_of_words_index);
  createParam(WAVEFORM_BUFFER_SIZE_STRING, asynParamInt32, &waveform_buffer_size_index);
  createParam(WAVEFORM_INITIALIZE_STRING, asynParamUInt32Digital, &waveform_init_index);
  //MAX_BUFFER_SIZE = bufferSize; //One of the parameters we pass to our port driver is the bufferSize, which is essentially how many words of information we want at a time

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

        int index = index_map[pvID];

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
          streaming_status[index] = "Successfully initialized"; 
          initialization_status[index] = true;
          std::chrono::system_clock::time_point real_time = std::chrono::system_clock::now();
          initialization_times[index] = real_time;

        }
        else {
          printf("No stream access");
          streaming_status[index] = "Initialization failed";
          return;
        }

        uint8_t *buf = new uint8_t[STREAM_MAX_SIZE];
        size_t nWords16, nBytes;
        int64_t got = 0;
        int64_t lastGot = 0;

        std::cout << "Outside the while loop now " << std::endl;
        std::cout << "MAX_BUFFER_SIZE is: " << MAX_BUFFER_SIZE << std::endl;
        while(1)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            got = stm->read( buf, MAX_BUFFER_SIZE, CTimeout(-1));
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
            //printf("Getting from the buffer required %llu milliseconds\n", duration);
            duration_data[index] = duration;

            std::chrono::system_clock::time_point real_time = std::chrono::system_clock::now();
            retrieval_times[index] = real_time;
            

            if(got > 8)
            {
                //printf("There's a thing in the stream? %ld bytes\n", got);
                lock();
                nBytes = (got - 9); // header = 8 bytes, footer = 1 byte, data = 32bit words.
                nWords16 = nBytes / 2; //Amount of words in our buffer to read
                if (nWords16 > 0)
                {
                  streaming_status[index] = "Successfully initialized and streaming data";
                }
                else
                {
                  streaming_status[index] = "Successfully initialized but no data in buffer";
                } 

                doCallbacksInt16Array((epicsInt16*)(buf + 8), nWords16, waveform_param_index, 0);

                for(int i = 0; i < MAX_BUFFER_SIZE; i++)
                {
                  /**
                   * Take data gathered from the stream and move it into an array we can play with
                   * TODO take options so we know how we'd want to modify the array
                   */
                  waveform_map[pvID][i] = (int16_t)buf[i];
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
                 streaming_status[index] = "Successfully initialized but received frame too small";
            }
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


//-------------------------------------------------------------------------------------
//IOCSH commands
//-------------------------------------------------------------------------------------

int waveformReaderConfigure(const char* portName, int bayNumber, int bufferSize, int waveformPVs)
{
  WaveformReader* temp = new WaveformReader(portName, bayNumber, bufferSize, waveformPVs);
  WaveformReader::setPortDriver(temp);
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
  WaveformReader* bayManager = WaveformReader::getPortDriver();
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


extern "C" {
  epicsExportRegistrar(waveformReaderRegister);
  epicsExportRegistrar(waveformStreamRegister);
}