sudo modprobe v4l2loopback devices=1
sudo modprobe v4l2loopback devices=1 video_nr=3 card_label="v4l2loopback" exclusive_caps=1

# report current video devices
v4l2-ctl --list-devices

