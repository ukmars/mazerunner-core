# Systick

The systick function is the beating heart of the robot control software. During operation many tasks must be performed at regular, fixed intervals. These tasks are time-sensitive and they must run frequently for good control of the robot.

While some tasks could be performed at a lower rate and with reduced priority compared to others, the Arduino nano used in UKMARSBOT is not running a real-time operating system so it is most convenient to run all the periodic tasks at the same rate.

There is a balance to be struck between wanting to run the tasks as frequently as possible and the risk of using up all the system resources just running regular systick updates.

In UKMARSBOT, the systick task runs at 500Hz - every 2 milliseconds.

## Tasks

Systick looks after the following:

 * collect encoder counts
 * monitor battery voltage
 * update motion profiles
 * correct steering errors
 * update the motor controllers
 * provide drive signals to the motors
 * update the sensors

 ### Encoder counts

 The motors each have an encoder attached that generates pulses at what can be quite high frequencies when running fast. These pulses generate interrupts that are very simple and execute quickly. All those interrupts do is increment or decrement counters. There is not time for anything more sophisticated. In systick, those counters - one for each wheel - are checked and the values used to work out how far the each wheel has moved since the last systick event. The counts for each wheel are converted into forward motion, in mm, and rotary motion, in degrees. These converted encoder values are used as the feedback inputs to the motor controllers.

 ### Battery voltage

 There are two primary reasons for checking the battery voltage. First, it is important that the battery not be discharged too much. Not only might this damage the battery but the robot control may become unreliable and unpredictable. The other reason is to make it possible to ensure that the motor drive is able to take into account changes in battery voltage. Freshly charged batteries will have a higher voltage that soon drops to a more steady value. Heavy demand on the batteries, such as when accelerating the robot, can also reduce the voltage temporarily. By monitoring the available voltage every systick cycle, the motor drive can be adjusted to compensate for supply changes.

 ### Motion profiles

 Robot movement is governed by generating velocity profiles for forward and rotational motion. These profiles keep track of the robot's commanded speed and position. The profiler software takes into account the acceleration and speed limits and ensures that the robot will reach a set point at exactly the right speed. The current output of the profilers is used as the set point input for the motor controllers.

 ### Steering correction

 Whenever the robot is traveling in a straight line, it is possible to use the values read from the sensors to calculate any errors caused by the robot being offset to one side or caused by the robot having a heading error. Both kinds of error will cause the robot to crash and both look very similar in terms of the values obtained form the sensors. The calculated steering error is called the Cross Track Error (CTE). It is always treated as if it is a heading error. To give a suitable correction to the robot, a separate steering controller uses the CTE to calculate a correction value that is added to the measured encoder angle. From the motor controller's point of view, this steering correction looks like the robot is turning unexpectedly.

 ### Motor controllers

 Once the robot set points, from the profilers, and the actual positions, from the encoders and steering, are known, they can be used as inputs to the motor controllers. There is one controller each for forward and rotational motion. Both these types of motion are profiled and controlled independently so that any combination of the two can be used to generate complex motion. When the controllers have made their calculations, the resulting correction outputs are combined into drive components for each of the wheels and sent to the motors as left and right drive voltages. The battery voltage compensation adjusts the actual PWM duty cycles so that the motors see the commanded voltage even if the battery voltage changes.

 ### Sensor control

 It may seem odd to be testing the sensors at the end of the systick cycle rather than the beginning. The reason is that the ADC conversion times on the ATmega328 chip are particularly slow and if systick had to wait around for all eight channels to convert, twice, is would waste a lot of processor time. Instead, the sensors are sampled using a separate sequence of interrupts. The last thing that happens in systick is that the first ADC conversion is triggered. Each conversion generates an interrupt which lets the code collect the relevant value and start another conversion. In this way, processing time is only used in collecting results, not waiting for conversions to finish. By the time the next systick cycle occurs, all the sensor results have been collected and are ready to use. At most, they are likely to be 1-2ms out of date. For the performance levels of the system, this delay is of no real consequence.
 No code must follow the sensor cycle start in systick or it will be interrupted by the sensor conversion interrupts.
