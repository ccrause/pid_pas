unit heatermodel;

{$mode ObjFPC}{$H+}

interface

type

  { THeaterModel }

  THeaterModel = class
  private
    // Heat capacity
    fCp: double;                       // kJ/kg/K
    fMass: double;                     // kg
    fConvectionRate: double;           // kW/K
    fTemperature, fTAmbient: double;   // K
  public
    procedure initialize(const initialT, Cp, mass, convectionRate,
      TAmbient: double);
    // Duty in kW, deltaTime is seconds
    function calcTemperature(const duty, deltaTime: double): double;
    property Temperature: double read fTemperature;
  end;

implementation

{ THeaterModel }

procedure THeaterModel.initialize(const initialT, Cp, mass, convectionRate,
  TAmbient: double);
begin
  fTemperature := initialT;
  fCp := Cp;
  fMass := mass;
  fConvectionRate := convectionRate;
  fTAmbient := TAmbient;
end;

function THeaterModel.calcTemperature(const duty, deltaTime: double): double;
var
  dQ_conv: double;
begin
  // Heat loss through convection to ambient
  // dQ(convection) = (Tamb - T) x k x A       [-K- x kJ/s/-m2-/-K- x -m2-] = kJ/s
  dQ_conv := (fTAmbient - fTemperature) * fConvectionRate;
  // dT(duty) = (Q + dQ) x dt / m / Cp                [-kJ-/-s- x -s- / -kg-] / [-kJ- / -kg- / K] = K
  fTemperature := fTemperature + (duty + dQ_conv) * deltaTime / fMass / fCp;
  Result := fTemperature;
end;

end.

