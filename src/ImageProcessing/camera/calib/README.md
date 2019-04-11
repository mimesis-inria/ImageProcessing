#GENERATE A CALIBRATION FILE, READABLE IN SOFACV:

- use OpenCV's calibrate.py script for instance, present in the source code (opencv/samples/python/calibrate.py) to generate the camera matrix, distortion coefficients etc.
You can also implement your own program to perform this, but the calibrate.py script does this well enough IMHO. [More documentation on OpenCV's website](https://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html)

- Open one of the yml file present in SofaCV/resources/calib/ and copy-paste the calibration parameters respecting the yaml format.

- Use the CalibLoader component to set the calibration settings in the CameraSettings component, as detailed in the CalibLoader.scn example scene (ImageProcessing/examples/camera/calib)
