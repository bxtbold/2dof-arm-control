# 2dof-arm-control
The goal of the project was control the arm and draw or write a given shape or character using a pen at the end-effector.
Inverse kinematics, PWM control, and one cosine methods are used in the task.

Setup:
- Arduino Mega
- Servo Motor MG996R x 2
- Arduino Cable
- Jump Cables
- 3D printed arm
- Pen

The first test (and i know this is not perfect as expected):


https://user-images.githubusercontent.com/55473193/143670874-74005fee-3906-432c-b7be-4ddbd06ec7ed.mp4


Conclusion:
First of all, I did not properly secured the pen at the end effector that resulted in a not straight or smooth lines.
Also, it was not possible to control at high precision because the board and the motors could not computationally handle small steps, which is necessary for high precision. Lastly, the arm beam is deflected when assembled. 

Tip:
Test on the plain surface;)
