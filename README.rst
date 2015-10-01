Nanoshield_IMU
==============

This is an Arduino library to control the Nanoshield_IMU. This nanoshield is
composed of LSM303D and L3GD20H CIs, from ST Microeletronics.

* Source code: https://github.com/circuitar/Nanoshield_IMU
* Documentation: http://nanoshield-imu.readthedocs.org/
* Reference board: `IMU Nanoshield`_ from Circuitar_

Library features include:

* Acceleration measure (accelerometer)
* Angular speed measure (gyroscope)
* Magnetic field measure (magnetometer)
* Read calibrated magnetometer
* North heading angle
* Set interruption from IMU
* Accelerometer buffered reading
* Interruption to custom specific movements

To install, just click **Download ZIP** and install it using **Sketch > Include Library... > Add .ZIP Library** in the Arduino IDE.

The following examples_ are provided:

* DataReadyInterruption_ Data ready interruption from accelerometer and magnetometer.
* FifoMode_ Producer/Consumer example using LSM303D internal buffer and buffer interruptions.
* ImuTest_ Simple IMU test reading all available measures.
* InterruptGenerator_ Specify and bound custom movement interruptions.
* mcalib_ Use with mcalib chrome app to find magnetometer calibration factors.

.. _`IMU Nanoshield`: https://www.circuitar.com/nanoshields/modules/imu/
.. _Circuitar: https://www.circuitar.com
.. _examples: https://github.com/circuitar/Nanoshield_IMU/tree/master/examples
.. _DataReadyInterruption: https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/DataReadyInterruption/DataReadyInterruption.ino
.. _FifoMode: https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/FifoMode/FifoMode.ino
.. _ImuTest: https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/ImuTest/ImuTest.ino
.. _InterruptGenerator: https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/InterruptGenerator/InterruptGenerator.ino
.. _mcalib: https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/mcalib/mcalib.ino

----

Copyright (c) 2015 Circuitar
All rights reserved.

This software is released under an MIT license. See the attached LICENSE file for details.