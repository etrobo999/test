mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
	Starter.o \
	ScenarioList.o \
	DoubleLoopScenario.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += \
	$(mkfile_path)scenario_handler \
	$(mkfile_path)unit 

INCLUDES += \
	-I$(mkfile_path)scenario_handler \
	-I$(mkfile_path)unit 