# State Estimation and Localization for Self-Driving Cars

State estimation is a vital part of any self-driving car software stack, which relies on the sensor measurements consisting of an IMU, a GNSS receiver, and a LiDAR to provide an accurate state estimation. However, since all of the sensors provide measurements of varying reliability and at different rates and reference frames, spatial and temporal multisensor calibration is required.

## Design
This project applied the Error-State Extended Kalman Filter (ES-EKF) on the state estimation pipeline to enhance the accuracy and reliability of the state estimation.

![flow chart](flow_chart.png)
###### Credit: University of Toronto Institute for Aerospace Studies

Where the motion model includes the specific force and rotational rates from our IMU and vehicle state consisting of position, velocity, and parametrization of
orientation using a unit quaternion. \
More details on implementation are provided in implementation_detial.pdf

## Tests of reliability and accuracy

### Test 1: normal case
First of all, testing the performance of the estimation with normal sensor measurements. The model is able to provide accurate estimate within error range.\
<img src="part_1A.png" alt="drawing" width="400"/><img src="part_1B.png" alt="drawing" width="400"/>
### Test 2: wrong extrinsic calibration
In orfer to test out the reliability of the state estimation, an intentional error is applied on the rotational matrix when transforming the reference frame of the LIDAR, resulting unreliable LIDAR measurements. By increasing the variance of the LIDAR estimated error, the model adapts and still performs well.\
<img src="part_2A.png" alt="drawing" width="400"/><img src="part_2B.png" alt="drawing" width="400"/>
### Test 3: sensor measurement dropout
A portion of the GNSS and LIDAR data is intentionally erased to test the performance of the estimation when only IMU is available. (like vehicle entering a tunnel)\
Although there exists a small shift in height estimation due to the wrong estimation of pitch angle, the model quickly adjusted state estimation of pitch angle and can perform as expected within a reasonable time and error range.\
<img src="part_3A.png" alt="drawing" width="400"/><img src="part_3B.png" alt="drawing" width="400"/>

## Installation

Install the official release of NumPy and matplotlib through pip:

```bash
python -m pip install -U pip
python -m pip install -U numpy
python -m pip install -U matplotlib
```

## Compilation & Run


```python
python es_ekf.py
```

## Special Thanks
Special thanks to [State Estimation and Localization for Self-Driving Cars by the University of Toronto](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars/home/info) for providing this excellent online course, and my mentor Dr. Tao from Texas A&M University for advising and sharing this information.

## Reference
[Vinohith](https://github.com/Vinohith/Self_Driving_Car_specialization)\
[daniel](https://github.com/daniel-s-ingram/self_driving_cars_specialization)\
[deepanshut041](https://github.com/deepanshut041/self-driving-car-specialization)\
[qiaoxu123](https://github.com/qiaoxu123/Self-Driving-Cars)
