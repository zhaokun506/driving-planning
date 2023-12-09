#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh


VERSION="0.8.0"
PKG_NAME="osqp-eigen-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://github.com/robotology/osqp-eigen/archive/v${VERSION}.tar.gz"
#文件不存在则下载，下载不成功请手动下载，放在thirdparty目录
# if [ ! -e "${PKG_NAME}" ]; then  
#     download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}" "${THIRD_PARTY_PATH}"
# fi

tar xzf ${THIRD_PARTY_PATH}/${PKG_NAME} -C ${THIRD_PARTY_PATH}

pushd ../thirdparty/osqp-eigen-${VERSION}
# mkdir build

cd build

cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=${THIRD_LIBRARY_INSTALL_PATH}/osqp-eigen-${VERSION} \
    -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install

popd

# rm -rf ${THIRD_PARTY_PATH}/${PKG_NAME_QSQP}

# ldconfig

ok "Successfully installed osqp-eigen-${VERSION}"