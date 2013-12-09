Image Coordinate Frames
=======================

In Ubitrack, we always assume the following image coordinate frame:

   The origin of the image is in the bottom left corner, the x values increase 
   from left to right, and y values increase from the bottom to the top.
   
OpenCV image coordinates already have this coordinate frame if the origin flag
of the IplImage structure is set to 1. In case this flag is 0, the y 
coordinates must be flipped before they are sent to other components.

Furthermore, all components that receive images must be able to deal with both
types of images.

We also assume that the z axis points into the camera, i.e. all world objects
seen by the camera have negative z coordinates. Together with the above, this 
implies that intrinsic camera matrices have the form:

   +fx  sk -cx
     0 +fy -cy
     0   0  -1

Integer subpixel coordinates are in the center of a pixel where no 
interpolation is necessary.