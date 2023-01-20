# Simulation of a heater with PID control
This example uses a simple model of a heater heating an object. 
A PID controller is used to regulate the object's teperature by manipulating the heating duty of the heating element.  
The temperature of the object is determined by the amount of energy from the heater and convective heat transfer with the ambient environment. 

## System model
The temperature of the object is determined with a simple energy balance:
```math
\begin{gather}
\Delta Q_conv = (T_ambient - T) * UA_convection \label{eq:1}
\frac{\Delta T}{\Delta t} = \frac{Q_heater + Q_convection)}{m * Cp}
Cp - \text{Heat capacity of object}
T - \text{Temperature of object}
T_ambient - \text{Ambient temperature}
UA_convection - \text{Product of convetive heat transfercoefficient and object's surfae area}
Q_conv - \text{Convective heat transfer between object and environment}
Q_heater - \text{Heat output of heater}
\end{gather}
```

