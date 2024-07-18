TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application
# =====================================================
# Path to "NON EPICS" External PACKAGES: USER INCLUDES
# =====================================================
USR_CXXFLAGS += -DHAVE_YAML -std=c++0x

USR_INCLUDES += -I$(BOOST_INCLUDE)
USR_INCLUDES += -I$(CPSW_FRAMEWORK_INCLUDE)
USR_INCLUDES += -I$(YAML_INCLUDE)
USR_INCLUDES += -I$(YAMLREADER_INCLUDE)
USR_INCLUDES += -I$(DEVICELIB_INCLUDE)

# =====================================================

# ======================================================
#PATH TO "NON EPICS" EXTERNAL PACKAGES: USER LIBRARIES
# ======================================================
cpsw_DIR         = $(CPSW_FRAMEWORK_LIB)
yaml-cpp_DIR     = $(YAML_LIB)
deviceLib_DIR    = $(DEVICELIB_LIB)
yamlReader_DIR   = $(YAMLREADER_LIB)
boost_system_DIR = $(BOOST_LIB)
# ======================================================

# ======================================================
# LINK "NON EPICS" EXTERNAL PACKAGE LIBRARIES STATICALLY
# ======================================================

USR_LIBS_Linux += cpsw
USR_LIBS_Linux += boost_system
USR_LIBS_Linux += yaml-cpp
USR_LIBS_Linux += yamlReader 
USR_LIBS_Linux += deviceLib


# ======================================================
PROD_IOC = LukasWaveformReader
# LukasWaveformReader.dbd will be created and installed
DBD += LukasWaveformReader.dbd

# LukasWaveformReader.dbd will be made up from these files:
LukasWaveformReader_DBD += base.dbd
LukasWaveformReader_DBD += asyn.dbd
LukasWaveformReader_DBD += yamlLoader.dbd
LukasWaveformReader_DBD += yamlDownloader.dbd 
LukasWaveformReader_DBD += waveform.dbd
# Include dbd files from all support applications:
#LukasWaveformReader_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#LukasWaveformReader_LIBS += xxx

# LukasWaveformReader_registerRecordDeviceDriver.cpp derives from LukasWaveformReader.dbd
LukasWaveformReader_SRCS += LukasWaveformReader_registerRecordDeviceDriver.cpp
LukasWaveformReader_SRCS += WaveformReader.cpp

# Build the main IOC entry point on workstation OSs.
LukasWaveformReader_SRCS_DEFAULT += LukasWaveformReaderMain.cpp
LukasWaveformReader_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#LukasWaveformReader_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
LukasWaveformReader_LIBS += $(EPICS_BASE_IOC_LIBS)
LukasWaveformReader_LIBS += cpsw
LukasWaveformReader_LIBS += asyn
LukasWaveformReader_LIBS += yamlLoader
LukasWaveformReader_LIBS += yamlDownloader

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

