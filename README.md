# Freenect-sample
alter openCV+openCL cartoon sample to use kinect depth sensor to improve outlines

This is a combination of 3 libraries openCV openCL freenect.

Started with the APPSDK sample for CartoonFilter that does two things. It finds the edges in an image and softens the colors using meanShiftFiltering to look cartoonish. The edge detection looks at color changes so many edges are not spacial edges.

To corect the edges we use freenect to access depth information. Doing edge detection on the depth information we can find where the actual object edges are. There is some image resizing/shifting to overlap the two cameras and then comparison is done between the two images to see where the edges overlap (give or take some number of pixels.)

TODO:
  lots of code cleanup (learning can be messy)
  optimize openCL kernel
  better overlaping of the two cameras
  make event driven instead of bussy loop
