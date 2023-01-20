# Simulation of a heater with PID control
This example uses a simple model of a heater heating an object. 
A PID controller is used to regulate the object's teperature by manipulating the heating duty of the heating element.  
The temperature of the object is determined by the amount of energy from the heater and convective heat transfer with the ambient environment. 

## System model
The temperature of the object is determined with a simple energy balance:
```math
\begin{gather}
\Delta Q_{conv} = (T_{ambient} - T) * UA_{convection} \\
\frac{\Delta T}{\Delta t} = \frac{Q_{heater} + Q_{convection})}{m * Cp} \\ \\
Cp - \text{Heat capacity of object} \\
T - \text{Temperature of object} \\
T_{ambient} - \text{Ambient temperature} \\
UA_{convection} - \text{Product of convective heat transfer coefficient and object's surfae area} \\
Q_{conv} - \text{Convective heat transfer between object and environment} \\
Q_{heater} - \text{Heat output of heater} \\
\end{gather}
```
This model is very simplified and neglects many details such as shape of the object, heat conduction through the object, variation of heat capcity with temperature and so forth.  It does however give a realistic response for testing PID settings.

## Temperature noise
In the application includes an option to add random noise to the reported temperature of the object, to simulate a noisy measurement and its effect on controller robustness.
