# arm_ws说明文档
## bringup功能包
- 用来编写bringup launch启动脚本，实现三种电机的节点启动
## serial_test功能包
- 用来编写串口测试脚本，其中：
    serial_test.py实现了串口数据的yaml文件格式保存。serial_test_2_port_read/write.py实现了串口数据的收发。
## DM_4310功能包
- 用来编写DM_4310电机的控制节点脚本。scripts/can_port_active.py脚本实现了在Jetson Xavier NX上注册CAN通讯口。src/DM4310_control.cpp中，首先将电机封装为一个类，然后再在控制类中实例化调用。设立子线程，独立CAN接收的程序，对电机状态进行实时更新。利用rosparam实现跨进程的电机位置同步。🐍
## Unitree_Go1功能包
- 用来编写Go1_8010电机的控制节点脚本。Go1_main.cpp中，控制类中实现电机控制与反馈。和前面一样，设立子线程，独立串口接收程序，对电机状态进行实时更新。利用rosparam实现跨进程的电机位置同步。
## Unitree_A1功能包
- 主要思路和Go1的思路相同，只是因为rosserial只能支持到4000000波特率，所以在python中实现串口的读写，然后利用sub/pub，实现跨进程的数据交换。