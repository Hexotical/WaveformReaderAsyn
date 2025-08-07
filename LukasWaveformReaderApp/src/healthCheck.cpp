#include "WaveformReader.h"

void healthTask(void*);

/**
 * Initiates a health-checking thread that continuously checks the status of all threads streaming
 * data and relaunches dead threads
 */
void WaveformReader::healthCheck(void)
{
    std::cout << "Initiating health check" << std::endl;

    asynStatus status;
    status = (asynStatus)(epicsThreadCreate("WaveformHealthTask", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)::healthTask, this) == NULL);
    if(status == asynError)
    {
        std::cout << "Unable to launch health check: " << status << std::endl;
    }
    else
    {
        std::cout << "Succesfully launched health check, status: " << status << std::endl;
    }
}

void healthTask(void* waveformPointer)
{
    WaveformReader *reader = static_cast<WaveformReader *>(waveformPointer);
    reader->healthTask();
}

void WaveformReader::healthTask(void)
{
    while(1)
    {
        sleep(HEALTH_CHECK_SLEEP_TIME);
        for (int i = 0; i < NUMBER_OF_WAVEFORM_RECORDS; i++)
        {
            std::string paramIndex = waveform_param_indices[i];

            int time_interval;
            getIntegerParam(*(interval_indices[i]), &time_interval);

            if (initialization_status[i]) 
            {   
                if (time_interval <= (std::chrono::milliseconds(100000)).count()) {thread_status[i] = "EXECUTING";}
                else 
                {
                    thread_status[i] = "DEAD, relaunching";
                    std::cout << "Relaunching dead thread: " << paramIndex << std::endl;
                    streamInit(paramIndex, stream_path_map[paramIndex]);
                }

                time_interval += 5000;
                setIntegerParam(*(interval_indices[i]), time_interval);
                callParamCallbacks();
            }
        } 
    }
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void waveformHealth(void)
{
  WaveformReader* bayManager = WaveformReader::getPortDriver();
  bayManager->healthCheck();
  return;
}
static const iocshFuncDef healthFuncDef = {"waveformHealth", 0};
static void healthCallFunc(const iocshArgBuf *args)
{
  waveformHealth();
}

void waveformHealthRegister(void)
{
  iocshRegister(&healthFuncDef, healthCallFunc);
}

extern "C" {
  epicsExportRegistrar(waveformHealthRegister);
}