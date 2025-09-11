sudo apt install swig libgtk2.0-dev pkg-config

gst-launch-1.0 webrtcsink stun-server=NULL name=ws signalling-server-host=localhost meta="meta,device=usb-vid1-video-index0" v4l2src device=/dev/video4 ! video/x-raw,width=640,height=480 ! videoconvert ! ws.
gst-launch-1.0 webrtcsink stun-server=NULL name=ws signalling-server-host=localhost meta="meta,device=usb-vid2-video-index0" v4l2src device=/dev/video5 ! video/x-raw,width=640,height=480 ! videoconvert ! ws.
gst-launch-1.0 webrtcsink stun-server=NULL name=ws signalling-server-host=localhost meta="meta,device=usb-vid3-video-index0" v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480 ! videoconvert ! ws.
