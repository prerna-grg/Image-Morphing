# Image-Morphing


Prerna Garg
IIT ROPAR
BTECH CSE

To compile the code use:
g++ .cpp -o m `pkg-config --cflags --libs opencv`

If you want to create morph for affine transformation simply enter the image name and 3:3 transformation matrix

If you want to create morph for two images using tie points, then enter the file names for tie points and image names

All the names must be entered as and when asked by console, initially just run ./m

The code will output all the intermediate images 
now to create gif run the following command:
convert -delay 15 -loop 0 *.jpg linux.gif

To change the transition speed in gif you may change 15 to any desired value

NOTE: There should not be any residual .jpg files else they will also get embedded in the gif
