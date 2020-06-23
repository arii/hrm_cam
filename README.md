   
   sudo modprobe v4l2loopback devices=2
   sudo modprobe v4l2loopback devices=1 video_nr=2 card_label="v4l2loopback" exclusive_caps=1


plug in bluetooth device -- do lsusb to get the idVendor:idProduct eg

    Bus 003 Device 007: ID 0a5c:21e8 Broadcom Corp. BCM20702A0 Bluetooth 4.0
    id vendor: 0a5c
    product id: 21e8

 create an /etc/udev/rules.d/ant2.rules with the following:
SUBSYSTEM=="usb", ATTRS{idVendor}=="0a5c", ATTRS{idProduct}=="21e8", RUN+="/sbin/modprobe usbserial vendor=0x0a5c product=0x21e8", MODE="0666", OWNER="ari", GROUP="root"

replug in the usb.  you should have dev ttyUSB0 node
>ls /dev/ttyUSB0
/dev/ttyUSB0

python passthru.py
ffplay /dev/video1


dmesg error:
dmesg | grep -i bluetooth | grep -i firmware
[ 4229.574617] bluetooth hci0: Direct firmware load for brcm/BCM20702A1-0a5c-21e8.hcd failed with error -2
 instlaling updated drivers here: https://github.com/winterheart/broadcom-bt-firmware
 

# remove hcitool gatttool permissions:
sudo setcap cap_net_raw+ep /usr/bin/hcitool 
sudo setcap cap_net_raw+ep /usr/bin/gatttool


https://johannesbader.ch/blog/track-your-heartrate-on-raspberry-pi-with-ant/
