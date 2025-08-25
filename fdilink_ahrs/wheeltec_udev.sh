#CH9102，同时系统没有安装对应驱动 串口号0003 设置别名为fdilink_ahrs
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/wheeltec_fdi_imu_gnss3.rules

service udev reload
sleep 2
service udev restart


