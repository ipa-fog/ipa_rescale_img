# The ipa_rescale_img Package
Helper Pkg to reduce Bandwidth (e.g. WIFI) for Visualization

Scales down and reduce FPS from camera images.


# Parameter Setting:

- Param:  rescale_factor  is used to resize input image in Percentages
- Param:  reduce_fps  is used to skip  x times before processing next frame


## Example Bandwidth of color images:

Uncompressed Color Image (sensor_msgs::Image, 30 FPS, 640x480)    Bandwidth = 27500 KB/s

Compressed Color Image (sensor_msgs::CompressedImage 30 FPS, 640x480)    Bandwidth = 750 KB/s


## Example how to save WIFI Bandwidth by reducing resolution:

1. Compressed Color Image & this Pkg (30 FPS, 320x240)    Bandwidth = 235 KB/s

1. Compressed Color Image & this Pkg (30 FPS, 128x96)    Bandwidth = 75 KB/s


## Example how to save WIFI Bandwidth by reducing FPS:

1. Compressed Color Image & this Pkg (15 FPS, 640x480)    Bandwidth = 200 KB/s


## Example how to save WIFI Bandwidth by reducing FPS and resolution:

1. Compressed Color Image & this Pkg (15 FPS, 128x96)    Bandwidth = 37 KB/s
