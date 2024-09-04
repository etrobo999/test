mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))
INCLUDE_DIRS = -I/usr/local/include/opencv4

# C言語のオブジェクトファイル
APPL_COBJS +=

# C++言語のオブジェクトファイル
APPL_CXXOBJS += \
	LineTracer.o

# 使用するライブラリ
APPL_LIBS += -lm
APPL_LIBS += `pkg-config --libs opencv4`
APPL_LIBS += -lraspicam -lraspicam_cv

# ソースディレクトリ
APPL_DIRS += \
	$(mkfile_path)LineTracer

# インクルードパス
INCLUDES += \
	-I$(mkfile_path)LineTracer \
	-I/usr/local/include/opencv4\
	-I/usr/local/include/raspicam_cv
# プログラム言語設定
SRCLANG := c++

# EV3RTアプリケーションの設定がある場合
ifdef CONFIG_EV3RT_APPLICATION

# ライブラリのインクルード
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

# コンパイルオプション
# COPTS += -fno-use-cxa-atexit