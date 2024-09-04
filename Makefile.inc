mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

# C言語のオブジェクトファイル
APPL_COBJS +=

# C++言語のオブジェクトファイル
APPL_CXXOBJS += \
	LineTracer.o

# 使用するライブラリ
APPL_LIBS += -lm
APPL_LIBS += `pkg-config --libs opencv4`  # OpenCVのライブラリをpkg-configで自動設定
APPL_LIBS += -lraspicam -lraspicam_cv

# ソースディレクトリ
APPL_DIRS += \
	$(mkfile_path)LineTracer

# インクルードパス
INCLUDES += \
	-I$(mkfile_path)LineTracer \
	-I/usr/include/opencv4 \  # OpenCV4のインクルードパスを指定
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