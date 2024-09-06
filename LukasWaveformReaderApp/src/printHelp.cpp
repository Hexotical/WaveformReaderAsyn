//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

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

extern "C" {
  epicsExportRegistrar(printHelpRegister);
}