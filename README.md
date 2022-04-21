# DC Motor Control
DC motors convert electrical energy into mechanical energy, compared with AC motors, DC motors have many advantages good speed performance, high starting torque and overload, etc.,</br>
A well known application on DC motors is DC Motor Control. When I say control, I mean, precicely, speed, position and both speed & position control.</br>
For that I will be using the STMH723ZG board as an MCU to control the motor, an H-Bridge to command the motor. The DC-Motor it self is equiped with an encoder.</br>
From the encoders I can read the current speed and position of the motor.</br>
Speed is calculated using the Input Capture Timer's feature as the angle between two pulses (Constant) / Time between two consecutive pulses.
The position is presented as a number of pulses covered in consequence the distance covered can be determined knowing the angle between two pulses (Constant and given) and the diameter of the wheel or gear (or whatever) put on the motor shaft. </br>
For more understanding here is an encoder schematic :
><img src="https://www.orientalmotor.com/images/servo-motors/servo-motor-incremental-encoder.jpg" width="300" height="200"><br/>

For this application, I used the PID controller both for speed and position control.</br>
Here is the block diagram of the application implemented on the STM board:
~~~
                               _ _ _ _ _ _ _ _ _ _ _ _ _ _ _                 _ _ _ _ _ _ _ _ _ _ 
  Ref_Speed(k)         Err    |                             |   Voltage     |                   | output
  --------------(+ _ )------->|  PID Block (Speed Control)  |-------------->|  DC Motor Plant   |------------------------>
                   ^          |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _|               |_ _ _ _ _ _ _ _ _ _|          |
                   |                                                                                       |
                   |                                                                                       |
                   |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ __ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  _|

                                                    ~Speed Control~
~~~

~~~
                               _ _ _ _ _ _ _ _ _ _ _ _ _ _ _                 _ _ _ _ _ _ _ _ _ _ 
  Ref_Position         Err    |                             |   Voltage     |                   | output
  --------------(+ _ )------->|  PID Block (Pos Control)    |-------------->|  DC Motor Plant   |------------------------>
                   ^          |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _|               |_ _ _ _ _ _ _ _ _ _|          |
                   |                                                                                       |
                   |                                                                                       |
                   |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ __ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  _|

                                                    ~Position Control~
~~~



