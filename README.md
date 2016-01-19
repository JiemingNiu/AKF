Integrating high-rate GPS and strong motion time series using an adaptive Kalman filter

++++++++++++++++++++
++++++   AKF  ++++++
++++++++++++++++++++

Jieming Niu, 03/2014

School of Geodesy and Geomatics
Wuhan University

++++++++++++++++++++

You can do what you will with this code please acknowledge,

Jieming Niu, Caijun Xu*, (2014), Real-time assessment of the broadband coseismic deformation of the 2011 Tohoku-Oki earthquake using an adaptive Kalman filter.

In the paper you can find more specific information about the algorithm.

FLOW:

0. Read this readme;

1. Prepare the data. I have included two simple data files for testing program from GPS 0041 and accelerometer FKS011 during the 2011 Tohoku-oki earthquake. The first column of the acceleration file acc.dat is seconds of day, and the second column is acceleration in SI, which is same to the GPS displacement file gps.dat, whose third column is positioning uncertainty.

2. Run AKF. If you would like to output the estimate of process noise variance, please uncomment the Line 59 and 91 of akf.m. The file output.dat is the running result of sample data, where the second column is displacements, and the third is velocity in SI.

Good Luck! 
