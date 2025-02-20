sudo apt install v4l-utils
v4l2-ctl --list-devices 用于列出所有视频设备
v4l2-ctl --device=/dev/videoX --list-formats-ext 查看某个设备支持的格式和分辨率
v4l2-ctl --device=/dev/videoX --all 获取设备的详细信息

export GST_DEBUG=*:5

python3 -m http.server 9999