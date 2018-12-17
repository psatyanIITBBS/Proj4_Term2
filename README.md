# Project - 04: PID Control

---
## Introduction

This project required us to design a PID (proportional/integral/differential) controller for the vehicle to run in the unity based simulator. There were primarily two steps involved. One, to come up with the initial P-I-D values so that the car can move on the track reasonably. Two, the P-I-D values be tuned by applying the general processing flow as described in the earlier lessons or otherwise. The sensors from the simulated vehicle provides cross-track error (CTE), speed, and steering angle data via the local websocket which the PID controller should use to generate the appropriate actuating command for steering and throttle (optional).

[//]: # (Image References)

[image1]: ./output_images/P_Controller.png  "P" 
        
      
[image2]: ./output_images/PD_Controller.png
[image3]: ./output_images/PID_Controller.png
[image4]: ./output_images/PID_Controller_Twiddle.png

## Reflection on the Implementation w.r.t. the Rubric

- *Describe the effect each of the P, I, D components had in your implementation.*

The P, or the "Proportional" component provides the "Stiffness" to the dynamical system. It acts like a spring in a spring-mass-damper system. Like a spring pulls (or, pushes) with more force when the mass moves farther away from the neutral position, the P-parameter provides sharper steering when the car is far away from the central line of the road. As the car comes closer and closer to the center, the steering becomes less and less severe. It behaves exactly like the F = -kx term in the spring system. 

![P](//Proj4_Term2/output_images/P_controller.png)

The D, or "Differential", component counteracts the P component's tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

![alt text][image2]

The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift (as in the Control unit lessons), but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

The final PID controller implementation performed much like in the following video (although, the controller performance suffered due to the screen recording consuming computation resources away from the websocket).

[Final Parameters](https://github.com/jeremy-shannon/CarND-PID-Control-Project/blob/master/demo_videos/PID09%20-%20final%20settings.m4v)

The following video demonstrates the subtle difference in performance when the I component is removed from the controller. Notice that the center line is not followed as closely around curves.

[I Parameter Removed](https://github.com/jeremy-shannon/CarND-PID-Control-Project/blob/master/demo_videos/PID10%20-%20zero%20i.m4v)

This final video demonstrates the disastrous effects of removing the D component from the controller. It begins to ring back and forth across the center line until finally leaving the track.

[D Parameter Removed](https://github.com/jeremy-shannon/CarND-PID-Control-Project/blob/master/demo_videos/PID11%20-%20zero%20d.m4v)


- *Describe how the final hyperparameters were chosen.*

Hyperparameters were tuned manually at first. This was necessary because the narrow track left little room for error, and when attempting to automate parameter optimization (such as Twiddle) it was very common for the car to leave the track, thus invalidating the optimization. Once I found parameters that were able to get the car around the track reliably, I then implemented Twiddle. I felt it necessary to complete a full lap with each change in parameter because it was the only way to get a decent "score" (total error) for the parameter set. For this reason my parameter changes are allowed to "settle in" for 100 steps and are then evaluated for the next 2000 steps. In all, I allowed Twiddle to continue for over 1 million steps (or roughly 500 trips around the track) to fine tune the parameters to their final values (P: 0.134611, I: 0.000270736, D: 3.05349).

I also implemented a PID controller for the throttle, to maximize the car's speed around the track. The throttle PID controller is fed the magnitude of the CTE because it doesn't make sense to throttle up for right-side CTE and down for left-side CTE, for example. For this reason the throttle controller doesn't include an I component, which would only grow indefinitely. The throttle controller was also fine-tuned using the same Twiddle loop, simultaneously with the steering controller. Though this is not an ideal setup (tuning parameters for two different controllers simultaneously), it still mostly converged to a good (if I do say so myself) solution.

---


