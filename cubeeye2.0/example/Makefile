#!/bin/sh

#
# Makefile
# 
#  Created on: 2018. 10. 15.
#      Author: erato
#

CC				?= gcc
CXX				?= g++

LOCAL_PATH		:= $(shell pwd)
LOCAL_MODULE 	:= example
LOCAL_CFLAGS	:= -std=c++17 -Wall -O2 -DNDEBUG
LOCAL_CFLAGS	+= -Wno-strict-aliasing -Wno-unused-result -Wno-unused-variable

LOCAL_CINCLUDES := -I$(LOCAL_PATH)/../include/CubeEye

LOCAL_LDLIBS	:= -lpthread \
				   -L$(LOCAL_PATH)/lib -lCubeEye
				
vpath %.cpp	$(LOCAL_SRC_PATH)

LOCAL_OBJECT_PATH	:= $(LOCAL_PATH)/obj
LOCAL_BINARY_PATH	:= $(LOCAL_PATH)/bin
LOCAL_LD_LIB_PATH 	:=../lib

LOCAL_SRC_PATH 	:= $(LOCAL_PATH)/.
LOCAL_SRC_FILES	:= $(foreach dir, $(LOCAL_SRC_PATH), $(wildcard $(dir)/*.cpp))

LOCAL_THIRDPARTY_PATH := $(LOCAL_PATH)/../thirdparty
ifneq ("$(wildcard $(LOCAL_THIRDPARTY_PATH)/liblive555/lib/Release)", "")
LOCAL_LDLIBS	+= -L$(LOCAL_THIRDPARTY_PATH)/liblive555/lib/Release \
				   -lLive555ClientSource -lUsageEnvironment -lBasicUsageEnvironment -lliveMedia -lgroupsock
LOCAL_LD_LIB_PATH := $(LOCAL_LD_LIB_PATH):$(LOCAL_THIRDPARTY_PATH)/liblive555/lib/Release
endif
ifneq ("$(wildcard $(LOCAL_THIRDPARTY_PATH)/libffmpeg/lib)", "")
LOCAL_LDLIBS	+= -L$(LOCAL_THIRDPARTY_PATH)/libffmpeg/lib \
				   -lavutil -lavformat -lavcodec -lswscale -lswresample
LOCAL_LD_LIB_PATH := $(LOCAL_LD_LIB_PATH):$(LOCAL_THIRDPARTY_PATH)/libffmpeg/lib
endif
ifneq ("$(wildcard $(LOCAL_THIRDPARTY_PATH)/libopencv/lib)", "")
LOCAL_LDLIBS	+= -L$(LOCAL_THIRDPARTY_PATH)/libopencv/lib \
				   -lopencv_core -lopencv_imgproc
LOCAL_LD_LIB_PATH := $(LOCAL_LD_LIB_PATH):$(LOCAL_THIRDPARTY_PATH)/libopencv/lib
endif

$(LOCAL_OBJECT_PATH)/%.o: %.cpp
	@echo "[build : $<]"
	$(CXX) -c -o $@ $(LOCAL_CDEFINES) $(LOCAL_CINCLUDES) $(LOCAL_CFLAGS) $<

LOCAL_OBJECT_FILES 	:= \
		$(patsubst %.cpp, $(LOCAL_OBJECT_PATH)/%.o, $(notdir $(LOCAL_SRC_FILES)))

define run_script_default
#!/bin/bash\
\n\n\
LD_LIBRARY_PATH=$(strip $(LOCAL_LD_LIB_PATH)) ./$(LOCAL_MODULE) "\$$\@"
endef

define run_sudo_user_script_default
#!/bin/bash\
\n\n\
sudo LD_LIBRARY_PATH=$(strip $(LOCAL_LD_LIB_PATH)) ./$(LOCAL_MODULE) "\$$\@"
endef

run_script := $(run_script_default)
run_sudo_user_script := $(run_sudo_user_script_default)

.PHONY: all clean
all: $(eval SHELL:=/bin/bash) mkdir $(LOCAL_OBJECT_FILES)
	$(CXX) -o $(LOCAL_BINARY_PATH)/$(LOCAL_MODULE) $(wildcard $(LOCAL_OBJECT_PATH)/*.o) $(LOCAL_LDLIBS)
	@echo -e "$(run_script)" > $(LOCAL_PATH)/bin/run
	@echo -e "$(run_sudo_user_script)" > $(LOCAL_PATH)/bin/run_sudo_user
	@chmod a+x $(LOCAL_PATH)/bin/run
	@chmod a+x $(LOCAL_PATH)/bin/run_sudo_user

clean:
	@if [ -d $(LOCAL_BINARY_PATH) ]; then rm -rf $(LOCAL_BINARY_PATH); fi
	@if [ -d $(LOCAL_OBJECT_PATH) ]; then rm -rf $(LOCAL_OBJECT_PATH); fi

mkdir:
	@if [ ! -d $(LOCAL_BINARY_PATH) ]; then mkdir $(LOCAL_BINARY_PATH); fi
	@if [ ! -d $(LOCAL_OBJECT_PATH) ]; then mkdir $(LOCAL_OBJECT_PATH); fi

