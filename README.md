# State Estimation and Localization for Self-Driving Cars

State estimation is a vital part of any self-driving car software stack, which relies on the sensor measurements consists of an IMU, a GNSS receiver, and a LiDAR to provide an accurate state estimation. However, since all of sensors provide measurements of varying reliability and at different rates and reference frame, spacial and temporal multisensor calibration is required.

## Design
This project applied the Error-State Extended Kalman Filter (ES-EKF) on state estimation pipeline to enhence the accuracy and reliability of the state estimation.

![flow chart](flow_chart.png)
###### Credit: University of Toronto Institute for Aerospace Studies

Where the motion model includes the specific force and rotational rates from our IMU and vehicle state consists of position, velocity and parametrization of
orientation using a unit quaternion. \
More details on implementation is provided in implementation_detial.pdf

## Tests of reliability and accuracy

### Test one: normal case
![part 1A](part_1A.png)
![part 1B](part 1B.png)
![part 2A](part 2A.png)
![part 2B](part 2B.png)
![part 3A](part 3A.png)
![part 3B](part 3B.png)

## Installation

Install the offcial release of numpy and matplotlib through pip:

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
Special thanks to [State Estimation and Localization for Self-Driving Cars by University of Toronto](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars/home/info) for providing this excellent online course, and my mentor Dr. Tao from Texas A&M University for advising and sharing this information.

## Reference
[Vinohith](https://github.com/Vinohith/Self_Driving_Car_specialization)\
[daniel](https://github.com/daniel-s-ingram/self_driving_cars_specialization)\
[deepanshut041](https://github.com/deepanshut041/self-driving-car-specialization)\
[qiaoxu123](https://github.com/qiaoxu123/Self-Driving-Cars)
