mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))
INCLUDE_DIRS = -I/usr/local/include/opencv4

# C言語のオブジェクトファイル
APPL_COBJS +=

# C++言語のオブジェクトファイル
APPL_CXXOBJS += \
	ScenarioList.o \
	Drive.o \
	DoubleLoopScenario.o \
	Starter.o

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

# 使用するライブラリ
APPL_LIBS += -lm
APPL_LIBS += `pkg-config --libs opencv4`

# ソースディレクトリ
APPL_DIRS += \
	$(mkfile_path)scenario_handler \
	$(mkfile_path)unit \
	$(mkfile_path)MotionControl

INCLUDES += \
	-I$(mkfile_path)scenario_handler \
	-I$(mkfile_path)unit \
	-I$(mkfile_path)MotionControl

# プログラム言語設定
SRCLANG := c++

# EV3RTアプリケーションの設定がある場合
ifdef CONFIG_EV3RT_APPLICATION

# ライブラリのインクルード
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

# コンパイルオプション
# COPTS += -fno-use-cxa-atexit

# C++11フラグの追加
CXXFLAGS += -std=c++11