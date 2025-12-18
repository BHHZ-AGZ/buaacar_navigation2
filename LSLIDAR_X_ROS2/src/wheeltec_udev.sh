#CH9102，同时系统没有安装对应驱动 串口号0002 设置别名为wheeltec_laser
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser"' >/etc/udev/rules.d/wheeltec_controller3.rules

service udev reload
sleep 2
service udev restart


