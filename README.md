# The Building an Estimator Project Readme #

This is the readme for the building an estimator C++ project.

Implemented:
    1.Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data:
      Plug in the standard deviation of the the GPS X signal and the IMU Accelerometer X signal into the top of config/6_Sensornoise.txt, set the values for MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY to be the values I have calculated 
    2.Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function:
      Implemented at QuadEstimatorEKF.cpp:111
    3.Implement all of the elements of the prediction step for the estimator:
      Implemented at QuadEstimatorEKF.cpp:179,213,264
    4.Implement the magnetometer update:
      Implemented at QuadEstimatorEKF.cpp:335
    5.Implement the GPS update:
      Implemented at QuadEstimatorEKF.cpp:303
