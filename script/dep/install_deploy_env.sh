#! /bin/bash

set -e
# 在 nx 上安装环境基础环境，需要安装 ros 环境之后

install_deb() {
  local param="$1"  # 包名
  wget http://192.168.10.101:8081/repository/jszr_deb/${param} && sudo dpkg -i ${param} && rm ${param}
}

JSZR_CURRENT_PREFIX=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
#bash ${JSZR_CURRENT_PREFIX}/ros2_dep.sh
install_deb "lcm_1.5.0-1_arm64.deb"
install_deb "unitree_sdk2_2.0.0_aarch64_Linux.deb"
install_deb "jszr_robots_dog_msg_0.0.1_aarch64_humble_Linux.deb"
install_deb "livox_sdk2_aarch64_Linux.deb"
install_deb "livox_driver_1.0.0_aarch64_humble_Linux.deb"
