# Project - 04: PID Control

---
## Introduction

This project required us to design a PID (proportional/integral/differential) controller for the vehicle to run in the unity based simulator. There were primarily two steps involved. One, to come up with the initial P-I-D values so that the car can move on the track reasonably. Two, the P-I-D values be tuned by applying the general processing flow as described in the earlier lessons or otherwise. The sensors from the simulated vehicle provides cross-track error (CTE), speed, and steering angle data via the local websocket which the PID controller should use to generate the appropriate actuating command for steering and throttle (optional).


## Reflection on the Implementation w.r.t. the Rubric

- *Describe the effect each of the P, I, D components had in your implementation.*

In order to investigate th eeffect of various hyperparameters on the CTE, the CTE data was written to a file and later plotted for visualization and comparison.

The P, or the "Proportional" component provides the "Stiffness" to the dynamical system. It acts like a spring in a spring-mass-damper system. Like a spring pulls (or, pushes) with more force when the mass moves farther away from the neutral position, the P-parameter provides sharper steering when the car is far away from the central line of the road. As the car comes closer and closer to the center, the steering becomes less and less severe. It behaves exactly like the F = -kx term in the spring system. 

![P_controller](./output_images/P_controller.png)

From observing the variation of CTE for the P-controller from the above, the D, or the "Differential" component then was added to the controller in order to control the blowing up of the CTE. The variation with respect to this parameter can be seen in the figure below.

![PD_controller](./output_images/PD_controller.png)

It can be clearly seen that the effect of the D-parameter is to damp the system and to restrict the overshoot by a great amount. However, it can be observed that the steady state error does not come down. That means the vehicle stays away from the center line. Then the necesity for the I-component of the controller was felt.

The I, or the "Integral" component tries to remove this steady state error by accumulating this error and acting so that this error will be reduced thereby bringing the vehicle towards the center. The effect of this parameter on the vehicle dynamics can be seen from the figure below.

![PID_controller](./output_images/PID_controller.png)

With the above manual tuning the vehicle was able to move around the track quite comfotablyy. However, the auto tuning method could be applid to improve the efficiency of the controller. 

- *Describe how the final hyperparameters were chosen.*

As described in the previous section, the hyperparameters were tuned manually to start with so that the vehicle can traverse the complete track without going out of the road boundaries. With these parameters as the starting point the autotuner algorithm was started which was implemented using the Twiddle Algorithm explained by Sebastian. The auto tuner initially started getting tuned with smaller portions of the track thereby changing the parameters. Then gradually the size of the segments for parameter tuning was increased so that the parameters can be tuned for th ewhole of the track. This step was followed many times to fine tune the arameters  and their their final values were set at (P: 0.134611, I: 0.0107811, D: 9.07742). The results for these hyper parameters are shown in the following graph.

![PID_controller_Twiddle](./output_images/PID_controller_Twiddle.png)

---


