#include "WaveformReader.h"

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void resetRegisters()
{
	Path p;
	p = cpswGetRoot();

	ScalVal _ChannelReg0DestSel, _ChannelReg0Enable, _ChannelReg1DestSel, _ChannelReg1Enable, 
			_ChannelReg1RateSel, _ChannelReg6DestSel, _ChannelReg6Enable, _ChannelReg7Enable, 
			_TriggerReg6Enable, _TriggerReg6Source, _TriggerReg7Enable, _TriggerReg7Source,
			_OutputConfig1, _OutputConfig3;


	_ChannelReg0DestSel = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[0]/DestSel")));
	_ChannelReg0DestSel->setVal(0x00007);

	_ChannelReg0Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[0]/Enable")));
	_ChannelReg0Enable->setVal(0x1);

	_ChannelReg1DestSel = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[1]/DestSel")));
	_ChannelReg1DestSel->setVal(0x00005);

	_ChannelReg1Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[1]/Enable")));
	_ChannelReg1Enable->setVal(0x1);

	_ChannelReg1RateSel = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[1]/RateSel")));
	_ChannelReg1RateSel->setVal(0x0001);

	_ChannelReg6DestSel = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[6]/DestSel")));
	_ChannelReg6DestSel->setVal(0x20000);
	
	_ChannelReg6Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[6]/Enable")));
	_ChannelReg6Enable->setVal(0x1);

	_ChannelReg7Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2ChannelReg[7]/Enable")));
	_ChannelReg7Enable->setVal(0x1);

	_TriggerReg6Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2TriggerReg[6]/Enable")));
	_TriggerReg6Enable->setVal(0x1);

	_TriggerReg6Source = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2TriggerReg[6]/Source")));
	_TriggerReg6Source->setVal(0x6);

	_TriggerReg7Enable = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2TriggerReg[7]/Enable")));
	_TriggerReg7Enable->setVal(0x1);

	_TriggerReg7Source = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AmcCarrierTiming/EvrV2CoreTriggers/EvrV2TriggerReg[7]/Source")));
	_TriggerReg7Source->setVal(0x6);

	_OutputConfig1 = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AxiSy56040/OutputConfig[1]")));
	_OutputConfig1->setVal("BP_TIMING_IN");

	_OutputConfig3 = IScalVal::create(p->findByName(("/mmio/AmcCarrierCore/AxiSy56040/OutputConfig[3]")));
	_OutputConfig3->setVal("FPGA_TIMING_IN");
}

static const iocshFuncDef resetFuncDef = {"resetRegisters", 0};
static void resetCallFunc(const iocshArgBuf *args)
{
  resetRegisters();
}

void resetRegister(void)
{
  iocshRegister(&resetFuncDef, resetCallFunc);
}

extern "C" {
  epicsExportRegistrar(resetRegister);
}

