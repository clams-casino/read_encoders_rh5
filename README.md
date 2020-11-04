ROS node that read the duckiebot wheel encoders ticks and wheel commands and publishes and estimate of the distance travelled by each wheel using the encoder ticks

# Usage 

Assuming in the root of the repo run:

To build
```
dts devel build -f -H <duckiebot name>.local
```

To run
```
dts devel run -H <duckiebot name>.local
```

# Wheel calibration
The wheel calibration is set up as a ROS service `read_encoders_node/calibrate_wheels`, which can be called on the node after performing the calibration procedure to get the an estimate of the radius of both wheels. The calibration proceedure involves driving the duckiebot forward for a short period of time. At the end measuring how much the duckiebot travelled forwards (X), laterally (Y), and turned (Theta). The measured change in position is used to obtain a least squares estimate of the radii of the wheels (detailed explanation found in the code).

## Calibration service usage

Call the service with
```
rosservice call /<duckiebot name>/read_encoders_node/calibrate_wheels <x in meters> <y in meters> <theta in degrees>
```
The service will responsed with the wheel radii in meters.

Note that a custom service message was defined in this package. So this service will have to be called from a container running an image where this service is built.