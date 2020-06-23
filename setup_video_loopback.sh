sudo modprobe v4l2loopback devices=2
sudo modprobe v4l2loopback devices=1 video_nr=2 card_label="v4l2loopback" exclusive_caps=1

