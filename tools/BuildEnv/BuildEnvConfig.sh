#! /bin/sh
#set -x
#set -e
#配置安装的环境变量
cat << End-of-msg
--------------------------
Welcome to apollo project
I wish you a happy study
--------------------------
End-of-msg
#path setting......
PROJECT_ROOT_PATH=$PWD

#工程根目录路径
export PROJECT_ROOT_PATH=$(realpath "$PROJECT_ROOT_PATH/")
echo PROJECT_ROOT_PATH=${PROJECT_ROOT_PATH}

#realpath 获取文件的绝对路径，最终获得tools文件路径
export TOOL_BASE_PATH=$(realpath "$PROJECT_ROOT_PATH/tools")
echo TOOL_BASE_PATH =  $TOOL_BASE_PATH

#环境build目录
export BUILD_ENV_DIR=$(realpath "$TOOL_BASE_PATH/ApolloBuildEnv")
echo BUILD_ENV_DIR = $BUILD_ENV_DIR
#cmake目录
export CMAKE_BIN_PATH="$TOOL_BASE_PATH/Cmake-3.20.5/bin/cmake"
echo CMAKE_BIN_PATH =  $CMAKE_BIN_PATH
#python目录
export PYTHON_BIN_PATH="$TOOL_BASE_PATH/Python-3.9.5/bin/"
echo PYTHON_BIN_PATH = $PYTHON_BIN_PATH
#第三方库下载目录
export THIRD_PARTY_PATH="${TOOL_BASE_PATH}/thirdparty"
echo THIRD_PARTY_PATH = $THIRD_PARTY_PATH

if [ ! -d "$TOOL_BASE_PATH/../bin" ];then
  mkdir -p $TOOL_BASE_PATH/../bin/library
fi

#第三方库安装路径
export THIRD_LIBRARY_INSTALL_PATH=$PROJECT_ROOT_PATH/bin/library
echo THIRD_LIBRARY_INSTALL_PATH = $PROJECT_ROOT_PATH/bin/library

# cd $BUILD_ENV_DIR

#apollo key world setting......
export DOWNLOAD_LOG="$BUILD_ENV_DIR/build.log"
echo DOWNLOAD_LOG = $DOWNLOAD_LOG

export WORKHORSE="gpu"
echo "WORKHORSE is: " ${WORKHORSE} 

export INSTALL_MODE="download" #build download
echo "INSTALL_MODE is: " ${INSTALL_MODE}

export nproc="32"
echo "make -j is: 32"


export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

# for DEBUG log
export GLOG_v=4
echo "GLOG level is debug"
