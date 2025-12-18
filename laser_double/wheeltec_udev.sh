#CH9102，同时系统没有安装对应驱动 串口号0002 设置别名为wheeltec_laser
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="laser1"' >/etc/udev/rules.d/wheeltec_controller2.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="laser2"' >/etc/udev/rules.d/wheeltec_controller3.rules
service udev reload
sleep 2
service udev restart
