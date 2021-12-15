
Analyze todo list:

- [x] save data when run `save_data` node
- [ ] show car path for view node
- [ ] place new model for view
- [ ] calculate parameters for 'height / cylinder axis / base distance'

- [ ] Rviz & Qt control



流程：
1. 启动程序，开始slam和reconstruct节点
2. 保存原始数据到指定文件
3. 对原始数据进行完整的处理，得到关键参数，保存到中间过程文件
4. 从中间过程文件进行读取参数，并进行显示。


中间过程文件包括：
1. 车辆轨迹（cleaned）
2. 每个support的6dof位姿
