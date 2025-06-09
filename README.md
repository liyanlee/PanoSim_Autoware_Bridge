# PanoSim_Autoware_Bridge
# 概述
PanoSim和Autoware联合仿真教程。基于PanoSim V32.9和Autoware.universe版本，包含通讯中间件，地图场景，安装配置。
# 目录结构
1. PanoSim：包含联合仿真中PanoSim端的相关文件
2. Autoware：包含联合仿真中Autoware端的相关文件
# 环境说明
PanoSim和Autoware电脑需要在一个网段，并且互相可以ping通，本例的地址如下
PanoSim：192.168.10.2
Autoware: 192.168.10.12
# 安装说明
1. 完成PanoSim V32.9 和 Autoware.universe 版本的安装。
2. 将目录PanoSim\PanoSimDatabase 下文件拷贝到PanoSim V32.9的Database下的相同目录
3. 将目录Autoware\install 和 Autoware\PanoSimBridge 目录拷贝到 Autoware的安装目录，一般为/home/${user_name}/autoware/
4. 用PanoExp打开实验PanoTown01并修改LidarSender, GnssSender,CameraSender的地址为autoware电脑的地址，端口若无特殊需求则不用修改
   ![image](images/LidarSenderConfig.png)
   ![image](images/CameraSenderConfig.png)
   ![image](images/GnssSenderConfig.png)
   
# 使用说明
