

In this lesson, you will see how to detect and localize ArUco markers in the camera frame or world frame.

###
* location_aruco_marker.py

Detect and localize an ArUco marker in the camera reference frame.

###

* location_aruco_world.py

Detect and localize an ArUco in the world frame. That is done in two different ways: directly using FrameTransformation or getting the matrices that relates the ArUco with the camera, and the camera with the world frame. For the last method, we multiply the matrices and compare the result with the matrix obtained from the direct method.

###
