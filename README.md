# PID Controller Implementation
Self-Driving Car Engineer Nanodegree Program

---

This projects implements a PID controller with an online twiddle algorithm 
discussed in the lectures.  

The twiddle algorithm tries to optimize 3 coefficients P,I,D adjusting the vehicle's steering angle 
in order to minimize cross-track error CTE (car's distance from the road's center).  P coefficient (Kp) 
simply causes an adjustment **P**roportional to CTE.  It results in the car oscillating around 
the set point (middle of the road) due to the overshoot: CTE is smaller near the set point causing 
less steering correction (Kp\*CTE) than CTE at the apex of the resulting sinusoid.

D coefficient (Kd) minimizes overshoot by dampening the **D**ifference between the current and 
previous CTE.  Kd\*(CTE2-CTE1) has the most impact on the steering angle near the set point, because 
that's where CTE changes most rapidly (in the sinusoid caused by Kp).

I coefficient (Ki) takes care of a situation when a steady error is introduced to driving (e.g. wheel
misalignment).  The steering angle is adjusted via **I**negration of a number of CTE measurements as 
follows Ki\*Sum(CTE 1..n).  If P & D coefficients result in the car stably driving off the set point,
Ki*Sum(CTE 1..n) adjusts the steering angle till the car drives near the point. 

In class, the offline twiddle algorithm was discussed.  I found it unsatisfying to apply this algorithm
to a real world scenario, so I decided to adjust the algo to work online.  First, I manually found more 
or less satisfactory Kp coefficient (0.2), which caused the car to oscillate around the middle of the road.  
This, by itself, was almost enough to drive the track.   Hoping to make the drive smoother, I introduced 
the twiddle when the average error over a number of CTE measurements (STEPS_PER_CYCLE) exceeded a max allowed 
error (MAX_ALLOWED_CTE).  See [PID.cpp](./src/PID.cpp) *twiddleIfNecessary* function.  I discovered that 
twiddle algo could not produce good results if the coefficient adjustment values (*dp*) were too large.  I manually
set *dp* values to 0.001.  This corroborates the fact that when we drive, we usually make very small adjustments. 
Ki and Kd coefficients are adjusted only at the sharpest turns of the track.  Those coefficients had less impact 
on the driving than Kp.

In the end, the car is able to drive the simulated track, albeit with some wobbling around the set point after
the sharp turns.  I was unable to find a way to make driving completely smooth.   One idea would be to implement 
a PID controller for the cars speed, as it is another contributing factor to overshoot.


To run the project and for more details refer to the original Udacity
[README](./README_orig.md).