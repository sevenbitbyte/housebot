#!/bin/bash

listOfPublicRepos="https://github.com/stonier/ecl_core.git
https://github.com/stonier/ecl_lite.git
https://github.com/stonier/ecl_navigation.git
https://github.com/stonier/ecl_tools.git
https://github.com/arebgun/dynamixel_motor.git
https://github.com/yujinrobot/kobuki.git
https://github.com/yujinrobot/kobuki_core.git
https://github.com/yujinrobot/kobuki_core.git
https://github.com/yujinrobot/kobuki_msgs.git
https://github.com/yujinrobot/yocs_msgs.git
https://github.com/yujinrobot/yujin_ocs.git"

for repoUri in $listOfPublicRepos
do
  git clone "$repoUri"
done
