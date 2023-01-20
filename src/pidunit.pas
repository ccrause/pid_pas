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
    procedure setParametersStandard(const Kp, Ti, Td: double); virtual; abstract;
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
    procedure setParametersStandard(const Kp, Ti, Td: double); override;
    // PV = process value, deltaT is time step from previous iteration
    function calcOP(const PV, deltaT: double): double; override;
    // Reset error history to 0, call e.g. when restarting controller
    procedure reset; override;
  end;

  { TPIDcontroller2 }

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
    procedure setParametersStandard(const Kp, Ti, Td: double); override;
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
  fPV[0] := 0;
  fPV[1] := 0;
  fPV[2] := 0;
end;

procedure TPIDcontroller2.setParametersParallel(const Kp, Ki, Kd: double);
begin
  fKp := Kp;
  fKi := Ki;
  fKd := Kd;
end;

procedure TPIDcontroller2.setParametersStandard(const Kp, Ti, Td: double);
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
  if abs(fSP) > 1 then
    fError[0] := 1 - PV/fSP
  else
    fError[0] := (fSP - PV);

  fIntegralError := deltaT*(fError[0] + fError[1])/2;

  fPV[2] := fPV[1];
  fPV[1] := fPV[0];
  fPV[0] := PV;
  fDerivative[1] := fDerivative[0];
  fDerivative[0] := -(fPV[0] - 2*fPV[1] + fPV[2])/deltaT;
  if abs(fSP) > 1 then
    fDerivative[0] := fDerivative[0] / fSP;
  fDerivative[0] := (1-fDerivativeFilterWeight)*fDerivative[0] +
                    fDerivativeFilterWeight*fDerivative[1];

  Result := fOP[0] + fKp*(fError[0] - fError[1])
            + fKi*fIntegralError
            + fKd*fDerivative[0];

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
  fPVold := 0;
end;

procedure TPIDcontroller1.setParametersParallel(const Kp, Ki, Kd: double);
begin
  fKp := Kp;
  fKi := Ki;
  fKd := Kd;
  fIntegralErrorClamp := fOPmax / fKi;
end;

procedure TPIDcontroller1.setParametersStandard(const Kp, Ti,
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
  if abs(fSP) > 1 then
    err := 1 - PV/fSP
  else
    err := (fSP - PV);

  fIntegralError := fIntegralError + err*deltaT;
  if fIntegralErrorClamp > 0 then
  begin
    if fIntegralError > fIntegralErrorClamp then
      fIntegralError := fIntegralErrorClamp
    else if fIntegralError < -fIntegralErrorClamp then
      fIntegralError := -fIntegralErrorClamp;
  end;

  deriv := (fPVold - PV) / deltaT;
  if abs(fSP) > 1 then
    deriv := deriv / fSP;

  fPVold := PV;
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

