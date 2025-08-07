We removed the following hardware connections since they aren't required at the moment:

ScalVal_RO _TrigCount_0;
ScalVal_RO _TrigCount_1;
_TrigCount_0 = IScalVal_RO::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(0) + "]/TrigCount").c_str()));
_TrigCount_1 = IScalVal_RO::create(p->findByName(("/mmio/AppTop/DaqMuxV2[" + std::to_string(1) + "]/TrigCount").c_str()));