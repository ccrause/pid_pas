# pid_pas
PID controller model written in Pascal.  

## Derivation of discrete form (1)
### Parallel form of PID
The conventional parallel form of a proportional - integral - derivate controller is:
```math
OP(t) = Kp.\epsilon (t) + Ki.\int_0^t \epsilon (t) \partial t + Kd.\frac{\partial \epsilon (t)}{\partial t}
```
Where $OP(t)$ is the controller output at time $t$, $\epsilon$ is the error between the controller setpoint $SP$ and the process value $PV$: $\epsilon = SP - PV$.  
This can be solved using a simple discrete formulation (to reduce computational power):

```math
OP(k) = Kp.\epsilon (k) + \underbrace { Ki. \sum_{j=0}^{k} \epsilon (j) \Delta t}_{\text{I(k)}} + \underbrace { Kd. \left( \frac{\epsilon (k) - \epsilon (k-1)}{\Delta t} \right)}_{D(k)}
```
The integral term $I(k)$ is further limited in magnitude to prevent unbounded growth of this term:
```math
\big| I(k) \big| < I_{max}
```
where
```math
I_{max} = \frac{OP_{max}}{Ki}
```
The derivative term $D(k)$ is sensitive to measurement noise of the $PV$.  To limit the impact of noise on the derivate term a low pass filter is used:
```math
D_{filtered}(k) = \alpha D(k-1) + (1-\alpha)D(k)
```
The filtered derivate term $D_{filtered}(k)$ is then used when calculating the output of the PID controller.
  
### Series (or standard) form of PID
The series (or standard) form of a proportional - integral - derivate controller is:
```math
OP(t) = Kg \left( \epsilon (t) + \frac{1}{Ti}\int_0^t \epsilon (t) \partial t + Td.\frac{\partial \epsilon (t)}{\partial t}\right)
```
This is mathematically identical to the first equation, with the gain calculated as $Kg = Kp$, the integral reset time calculated as $Ti = \frac{Kp}{Ki}$ and the integration time is calculated as $Td = Kp.Kd$.  This form is often used in industry, hence the option is available to specify the PID settings in this form.  Internally the parameters are converted to the equivalent parallel form and solved via equations above.
