unit pidunit;

{$mode ObjFPC}{$H+}

interface

type
  { TPIDcontrollerBase }

  TPIDcontrollerBase = class
  private
    // PID parameters
    fOPmax, fOPmin: double;  // defaults to 0 - 1, or a fractional output
    fSP: double;

    fDerivativeFilterWeight, oldderiv: double;
    function getDerivativeFilterValue: double;
    procedure setDerivativeFilterValue(AValue: double);
  public
    // Just set integral error = 0
    procedure initialize; virtual; abstract;
    procedure setParametersParallel(const Kp, Ki, Kd: double); virtual; abstract;
    procedure setParametersSeries(const Kp, Ti, Td: double); virtual; abstract;
    procedure newSetpoint(const SP: double);
    // PV = process value, deltaT is time step from previous iteration
    function calcOP(const PV, deltaT: double): double; virtual; abstract;
    // Change OP limits, by default OP range is 0 - 1
    procedure setOPlimits(const OPmin, OPmax: double);
    // Reset error history to 0, call e.g. when restarting controller
    procedure reset; virtual; abstract;
    property DerivativeFilterValue: double read getDerivativeFilterValue write setDerivativeFilterValue;
  end;

  { TPIDcontroller1 }

  TPIDcontroller1 = class(TPIDcontrollerBase)
  private
    // PID parameters
    fKp, fKi, fKd: double;
    fIntegralError: double;
    fIntegralErrorClamp: double;
    fOldError: double;
    // Storage for discrete calculations
    fOP, fPVold: double;
  public
    // Just set integral error = 0
    procedure initialize; override;
    procedure setParametersParallel(const Kp, Ki, Kd: double); override;
    procedure setParametersSeries(const Kp, Ti, Td: double); override;

    //procedure setParameters(const proportional, integral, derivative: double); override;
    //procedure setParametersStandard(const Kp, Ti, Td: double);
    // PV = process value, deltaT is time step from previous iteration
    function calcOP(const PV, deltaT: double): double; override;
    // Reset error history to 0, call e.g. when restarting controller
    procedure reset; override;
  end;

  { TPIDcontroller2 }
  // Basic discrete calculation of output (with process value used for derivative term):
  //   OP[k] = Kp*err[k] + Ki*sum(err[j]*deltaT) + Kd*(PV[k] - PV[k-1])/deltaT
  // Calculate difference between previous and current output
  //    OP[k] - OP[k-1] = Kp*(err[k] - err[k-1]) +
  //                      Ki*err[k]*deltaT +
  //                      Kd*(PV[k] - 2*PV[k-1] + PV[k-2]) / deltaT
  TPIDcontroller2 = class(TPIDcontrollerBase)
  private
    // PID parameters
    fKp, fKi, fKd: double;
    fIntegralError: double;
    // History
    fPV: array[0..2] of double;
    fError: array[0..2] of double;
    fOP: array[0..1] of double;
    fDerivative: array[0..1] of double;
  public
    // Just set integral error = 0
    procedure initialize; override;
    procedure setParametersParallel(const Kp, Ki, Kd: double); override;
    procedure setParametersSeries(const Kp, Ti, Td: double); override;
    // PV = process value, deltaT is time step from previous iteration
    function calcOP(const PV, deltaT: double): double; override;
    // Reset error history to 0, call e.g. when restarting controller
    procedure reset; override;
  end;

implementation

{ TPIDcontroller2 }

procedure TPIDcontroller2.initialize;
begin
  fKp := 1;
  fKi := 0.5;
  fKd := 0;
  fOPmax := 1;
  fOPmin := 0;
  // Discrete storage
  fOP[0] := 0;
  fOP[1] := 0;
  fError[0] := 0;
  fError[1] := 0;
  fError[2] := 0;
  fDerivative[0] := 0;
  fDerivative[1] := 0;
end;

procedure TPIDcontroller2.setParametersParallel(const Kp, Ki, Kd: double);
begin
  fKp := Kp;
  fKi := Ki;
  fKd := Kd;
end;

procedure TPIDcontroller2.setParametersSeries(const Kp, Ti, Td: double);
var
  Ki, Kd: double;
begin
  Kd := Kp*Td;
  // Clamp Ti to prevent division by zero during conversion to Ki
  if abs(Ti) < 1e-4 then
    Ki := 0
  else
    Ki := Kp/Ti;
  setParametersParallel(Kp, Ki, Kd);
end;

function TPIDcontroller2.calcOP(const PV, deltaT: double): double;
begin
  fError[2] := fError[1];
  fError[1] := fError[0];
  if abs(fSP) < 1e-3 then
    fError[0] := (fSP - PV)
  else
    fError[0] := 2*(fSP - PV)/(abs(fSP) + abs(PV));

  fIntegralError := deltaT*(fError[0] + fError[1])/2;

  fPV[2] := fPV[1];
  fPV[1] := fPV[0];
  fPV[0] := PV;
  fDerivative[1] := fDerivative[0];
  fDerivative[0] := -(fPV[0] - 2*fPV[1] + fPV[2])/deltaT;
  fDerivative[0] := (1-fDerivativeFilterWeight)*fDerivative[0] +
                    fDerivativeFilterWeight*fDerivative[1];

  Result := fOP[0] + fKp*(fError[0] - fError[1])
            + fKi*fIntegralError
            + fKd*fDerivative[0];

  //Result := fOP[0] +
  //        (fKp + fKi*deltaT/2 + fKd/deltaT) * fError[0] +
  //        (-fKp + fKi*deltaT/2 - 2*fKd/deltaT) * fError[1] +
  //        fKd/deltaT*fError[2];

  // Apply output clamping if set
  if not(fOPmin = fOPmax) then
  begin
    if Result > fOPmax then
      Result := fOPmax
    else if Result < fOPmin then
      Result := fOPmin;

  if Result > 2*fOPMax then
    Result := 2*fOPMax
  else if Result < fOPMin then
    Result := fOPMin;
  end;
  fOP[1] := fOP[0];
  fOP[0] := Result;
end;

procedure TPIDcontroller2.reset;
begin
  // Discrete storage
  fOP[0] := 0;
  fOP[1] := 0;
  fError[0] := 0;
  fError[1] := 0;
  fError[2] := 0;
end;

//function TPIDcontrollerStandard.calcOP(const PV, deltaT: double): double;
//var
//  err: double;
//begin
//  //fOP, fE1, fE2: double;
//  if abs(fSP) < 1e-3 then
//    err := (fSP - PV)
//  else
//    err := 0.5*(fSP - PV)/(abs(fSP) + abs(PV));
//
//  Result := fOP + fKp(1 + deltaT/(2*fTi) + fTd/deltaT) * err +
//            (-fKp + fKi*deltaT/2 - 2*fKd/deltaT) * fE1 +
//            fKd/deltaT*fE2;
//
//  fOP := Result;
//
//  // Apply output clamping if set
//  if not(fOPmin = fOPmax) then
//  begin
//    if Result > fOPmax then
//      Result := fOPmax
//    else if Result < fOPmin then
//      Result := fOPmin;
//
//  if fOP > 2*fOPMax then
//    fOP := 2*fOPMax
//  else if fOP < fOPMin then
//    fOP := fOPMin;
//  end;
//
//  fE2 := fE1;
//  fE1 := err;
//  //fOP := Result;
//end;

{ TPIDcontrollerBase }

function TPIDcontrollerBase.getDerivativeFilterValue: double;
begin
  Result := fDerivativeFilterWeight;
end;

procedure TPIDcontrollerBase.setDerivativeFilterValue(AValue: double);
begin
  if AValue >= 0 then
    fDerivativeFilterWeight := AValue
  else
    fDerivativeFilterWeight := 0;
end;

procedure TPIDcontrollerBase.newSetpoint(const SP: double);
begin
  fSP := SP;
end;

procedure TPIDcontrollerBase.setOPlimits(const OPmin, OPmax: double);
begin
  fOPmin := OPmin;
  fOPmax := OPmax;
end;

{ TPIDcontroller1 }

procedure TPIDcontroller1.initialize;
begin
  fKp := 1;
  fKi := 0.5;
  fKd := 0;
  fIntegralError := 0;
  fIntegralErrorClamp := 0;
  fOldError := 0;
  fOPmax := 1;
  fOPmin := 0;
  // Discrete storage
  fOP := 0;
end;

procedure TPIDcontroller1.setParametersParallel(const Kp, Ki, Kd: double);
begin
  fKp := Kp;
  fKi := Ki;
  fKd := Kd;
  fIntegralErrorClamp := fOPmax / fKi;
end;

procedure TPIDcontroller1.setParametersSeries(const Kp, Ti,
  Td: double);
var
  Ki, Kd: double;
begin
  Kd := Kp*Td;
  // Clamp Ti to prevent division by zero during conversion to Ki
  if abs(Ti) < 1e-4 then
    Ki := 0
  else
    Ki := Kp/Ti;
  setParametersParallel(Kp, Ki, Kd);
end;

function TPIDcontroller1.calcOP(const PV, deltaT: double): double;
var
  err, deriv: double;
begin
  // Scale error and derivative
  // Prevent problems when scale is close to 0
  if abs(fSP) < 1e-3 then
    err := 0
  else
    err := 2*(fSP - PV) / (abs(fSP) + abs(PV));

  fIntegralError := fIntegralError + err*deltaT;
  if fIntegralErrorClamp > 0 then
  begin
    if fIntegralError > fIntegralErrorClamp then
      fIntegralError := fIntegralErrorClamp
    else if fIntegralError < -fIntegralErrorClamp then
      fIntegralError := -fIntegralErrorClamp;
  end;

  deriv := (fPVold - PV) / deltaT;
  fPVold := PV;
  // Filter derivative, helps when PV is noisy
  deriv := oldderiv*(fDerivativeFilterWeight) + (1-fDerivativeFilterWeight)*deriv;

  Result := fKp * err +             // Proportional term
            fKi * fIntegralError +  // Integral term
            fKd * deriv;            // Derivative term

  oldderiv := deriv;
  // Apply output clamping if set
  if not(fOPmin = fOPmax) then
  begin
    if Result > fOPmax then
      Result := fOPmax
    else if Result < fOPmin then
      Result := fOPmin;
  end;
  fOldError := err;
end;

procedure TPIDcontroller1.reset;
begin
  fIntegralError := 0;
  fOldError := 0;
  fOP := 0;
  fPVold := 0;
end;

end.

