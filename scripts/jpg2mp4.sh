ffmpeg -framerate 2 -i $1_%0d.jpg -c:v libx264 -profile:v high -crf 23 -pix_fmt yuv420p $1.mp4
