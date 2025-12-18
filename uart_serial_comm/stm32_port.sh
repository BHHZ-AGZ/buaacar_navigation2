#与下位机通讯串口 设置别名为  stm32_port
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", GROUP:="dialout", SYMLINK+="stm32_port"' >/etc/udev/rules.d/serial.rules

service udev reload
sleep 2
service udev restart


