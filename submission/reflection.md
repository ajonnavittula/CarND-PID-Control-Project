# PID Control Project Rubric Discussion

## Compilation

The code compiles with cmake and make. There were no modifications to the CMakeLists.txt.

## Implementation

The base PID algorithm follows what was taught in the lectures. For hyperparameter tuning, manual tuning was used to find a workable jumpoff point. The twiddle algorithm was used to fine tune/optimize the control gains.

## Reflection

### Effects of PID components

The PID controller uses proportional, integral and derivative components to calculate the control gain for a given process error. The process error in this case is the Cross Track Error (CTE) defined as the difference between the car's position from the center of the lane. Each of the individual controller gain components will be discussed below.

#### Proportional term

The proportional term creates a linear relationship between the CTE and the set point (controller output). Use of proportional gain in this particular case creates a steering angle that is in the opposite direction of the incoming cte (due to the negative sign). The proportional gain is a response to the current cte and does not include past cte or predictions of future cte's. The proportional gain on its own leads to oscillations, which depending on the gain value may amplify or decrease in magnitude in consecutive time steps. The short video below shows the simulator running a simple P controller and it can be observed that the oscillations increase in magnitude as the car keeps moving.

[![Proportional Only Controller](http://img.youtube.com/vi/PatkDqzkSfM/0.jpg)](http://www.youtube.com/watch?v=PatkDqzkSfM)

#### Derivative term

The derivative term creates a linear relationship between the rate of change in CTE and the set point (controller output). Use of the derivative gain in this particular case anticipates the future state of the car and dampens/softens the oscillations created by the proportional gain. However, the derivative gain is usually sensitive to noise and may react to it strongly if there is a lot of noise in the sensors being used. The short video below shows the simulator running a simple D controller. Due to the rate of change in error being small in consecutive steps, we can see that the car reacts with small steering angles and has a hard time with sharp turns.

[![Derivative Only Controller](http://img.youtube.com/vi/rGF4cr0w8ag/0.jpg)](http://www.youtube.com/watch?v=rGF4cr0w8ag)


#### Integral term

The integal term creates a linear relationship between the cumulative cte over time and current set point (controller output). Usually the effects of the integal terms are not observed until sufficient time has passed. Use of the integral term helps remove a bias in the system created due to process constraints. In the case of a self driving car, it can be noticed that the turns are smoother with the integral gain in use. The short video below shows the simulator running a simple I controller. As expected the controller does not react instantaneously and requires that the cumulative CTE be sufficiently high before changing the steer angle.

[![Integral Only Controller](http://img.youtube.com/vi/o8WuwsmXCzI/0.jpg)](http://www.youtube.com/watch?v=o8WuwsmXCzI)

#### Hyperparameter Tuning

To tune the hyperparameters, a combination of manual tuning and twiddle algorithm were used. The trial and error method was used to obtain a baseline for the controller. With the controller gains obtained by manual tuning, the car was able to make one full lap around the track without hitting the curb. However, there were significant oscillations in the system and it would not have been considered safe for a human to be sitting in the car. The twiddle algorithm was used to find a local minima that would dampen the oscillations. The short video below shows the controller running with the twiddle algorithm enabled. It can be seen that the car makes most of the turns smoothly. However, there are some oscillations lingering and the car has a hard with the last curve. Using an average window filter or a low pass filter may help eliminate the osicllations.

[![PID Controller](http://img.youtube.com/vi/QS_azTC0frc/0.jpg)](http://www.youtube.com/watch?v=QS_azTC0frc)
