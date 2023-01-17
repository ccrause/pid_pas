# pid_pas
PID controller model written in Pascal.  

## Derivation of discrete equation
The conventional parallel form of a proportional - integral - derivate controller is:
```math
OP(t) = Kp.\epsilon (t) + Ki.\int_0^t \epsilon (t) \partial t + Kd.\frac{\partial \epsilon (t)}{\partial t}
```
Where $OP(t)$ is the controller output at time $t$, $\epsilon$ is the error between the controller setpoint ( $SP$ ) and the process value ( $PV$ ): $\epsilon = SP - PV$.
