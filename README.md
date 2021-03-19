COMP 5823 Animation and Simulation cw2

Thomas Moreno Cooper sc17tm@leeds.ac.uk
University of Leeds 
nov 2020

Libraries used:
glm
https://glm.g-truc.net/0.9.2/api/a00001.html
Eigen
http://eigen.tuxfamily.org/index.php?title=Main_Page
glu
https://www.opengl.org/resources/libraries/
Qt

! uses c++ 11 

With the libraries and Qt version 5.9.5 on Linux x86
- run qmake (version 3.1)
- make
- execute

Make sure the folder called bvh is in the same directory. It must contain at least the "arms.bvh" file. This will be updated later along with extra control.

Application can read BVH files containing animations or poses. Saving a modified pose will overwrite the current frame. A desirable features is control points to constrain the movement of the pose when modifying it, may be added in the future when I have spare time.

![Dungeon2](https://media.giphy.com/media/sH9Cq1mNDGGmPt0iUl/giphy.gif)
