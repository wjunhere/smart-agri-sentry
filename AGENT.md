# AGENT

Project agent notes and instructions.
- 可以通过ssh连接到RDKX5板端 ，板端环境是Ubuntu22.04，ssh sunrise@10.101.47.106 ，密码是sunrise，项目放在了~\dev_ws,项目从本项目克隆而来，只是文件夹名重命名为dev_ws， 通过git进行版本管理
- 上位机RDK代码优先先在本地进行开发，本地没有ros2运行的环境，先提交到远程仓库，再在RDK板端拉取仓库，进行测试