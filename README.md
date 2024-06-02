# EPUCK Range_and_Bearing board for webots
# 文件描述
epuck_controller.py:智能体的控制器

supervisor_controller.py:中央控制器

config.py:参数设置文件

# 安装
1.安装webots（2023b版本）：
- [Linux](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

2.建立虚拟环境：
```
conda create -n webots python=3.8
```

3.安装deepbots:
```
pip install deepbots
```

4.配置文件：

第一种方法（推荐）：

在epuck_controller、target_controller文件夹下，找到对应文件夹名字的.py文件，并在第一行指定环境：
```
#!<自己安装路径>/miniconda3/envs/webots/bin/python
```

第二种方法：

在epuck_controller和supervisor_controller文件夹下，找到runtime.ini文件：
```
[python]
COMMAND = <自己安装路径>/miniconda3/envs/webots/bin/python
```

# 执行
1.打开webots载入worlds.wbt

2.执行simulation

3.运行supervisor_controller.py文件
```
python supervisor_controller.py
```

