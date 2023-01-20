unit gui;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls,
  ExtCtrls, ComCtrls, TAGraph, TASeries, TACustomSeries, TATransformations,
  pidunit, heatermodel;

type

  { TForm1 }

  TForm1 = class(TForm)
    DerivativeLabel1: TLabel;
    DerivativeFilterWeightEdit: TEdit;
    PIDTypeRadioGroup: TRadioGroup;
    PIDCalcRadioGroup: TRadioGroup;
    SimulateButton: TButton;
    Chart1: TChart;
    Chart1LineSeries1: TLineSeries;
    Chart1LineSeries2: TLineSeries;
    Chart1LineSeries3: TLineSeries;
    Chart1LineSeries4: TLineSeries;
    Chart1LineSeries5: TLineSeries;
    Chart1LineSeries6: TLineSeries;
    convectionEdit: TEdit;
    CpEdit: TEdit;
    GroupBox1: TGroupBox;
    GroupBox2: TGroupBox;
    KdEdit: TEdit;
    KiEdit: TEdit;
    KpEdit: TEdit;
    ProportionalLabel: TLabel;
    Label10: TLabel;
    IntegalLabel: TLabel;
    DerivativeLabel: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    LeftAxisTransformations: TChartAxisTransformations;
    LeftAxisTransformationsAutoScaleAxisTransform1: TAutoScaleAxisTransform;
    LeftAxisTransformationsAutoScaleAxisTransform2: TAutoScaleAxisTransform;
    massEdit: TEdit;
    Panel4: TPanel;
    qMaxEdit: TEdit;
    RightAxisTransformations: TChartAxisTransformations;
    RightAxisTransformationsAutoScaleAxisTransform1: TAutoScaleAxisTransform;
    RightAxisTransformationsAutoScaleAxisTransform2: TAutoScaleAxisTransform;
    SPEdit: TEdit;
    TambEdit: TEdit;
    TnoiseEdit: TEdit;
    procedure PIDTypeRadioGroupSelectionChanged(Sender: TObject);
    procedure SimulateButtonClick(Sender: TObject);
  private
    Tnoise: double;
    procedure initializeModels(var heater: THeaterModel; var PID: TPIDcontrollerBase);
  public

  end;

var
  Form1: TForm1;

implementation

{$R *.lfm}

uses
  math;

{ TForm1 }

procedure TForm1.SimulateButtonClick(Sender: TObject);
var
  PID: TPIDcontrollerBase;
  heater: THeaterModel;
  SP, OP, qMax: double;
  q, t, time: double;
  i: integer;
begin
  SP := StrToFloat(SPEdit.Text);
  qMax := StrToFloat(qMaxEdit.Text);

  Chart1.AxisList[0].Range.Max := SP*1.2;
  Chart1.AxisList[0].Range.UseMax := true;
  Chart1.AxisList[2].Range.Max := 100;
  Chart1.AxisList[2].Range.UseMax := true;

  heater := THeaterModel.Create;
  if PIDCalcRadioGroup.ItemIndex = 0 then
    PID := TPIDcontroller1.Create
  else
    PID := TPIDcontroller2.Create;
  initializeModels(heater, PID);
  PID.newSetpoint(SP);

  q := 0;
  i := 0;
  time := 0;
  Chart1LineSeries1.Clear;
  Chart1LineSeries2.Clear;
  Chart1LineSeries3.Clear;

  PID.reset;

  repeat
    time := i / 60;
    t := heater.calcTemperature(q, 1);
    t := t + Tnoise*(1 - 2*Random);
    OP := PID.calcOP(t, 1);
    q := qMax * OP;
    inc(i);
    Chart1LineSeries1.AddXY(time, t);
    Chart1LineSeries2.AddXY(time, OP*100);
    Chart1LineSeries3.AddXY(time, SP);
  until i > 3600;

  PID.Free;
  heater.Free;
end;

procedure TForm1.PIDTypeRadioGroupSelectionChanged(Sender: TObject);
const
  rounding = 100000;
var
  p, i, d: double;
begin
  case PIDTypeRadioGroup.ItemIndex of
    0:
    begin
      ProportionalLabel.Caption := 'Kp, 1/°C';
      IntegalLabel.Caption := 'Ki, 1/s/°C';
      DerivativeLabel.Caption := 'Kd, s/°C';
      // Convert from series form
      p := StrToFloat(KpEdit.Text);
      i := StrToFloat(KiEdit.Text);
      d := StrToFloat(KdEdit.Text);
      // Clamp Ki to prevent division by zero
      if abs(i) < 1e-4 then
        i := sign(i)*1e-4;
      // Ki = Kp/Ti
      KiEdit.Text := FloatToStr(round(rounding*p/i)/rounding);
      // Kd = Kp*Td
      KdEdit.Text := FloatToStr(round(rounding*p*d)/rounding);
    end;
    1:
    begin
      ProportionalLabel.Caption := 'Kp, 1/°C';
      IntegalLabel.Caption := 'Ti, s';
      DerivativeLabel.Caption := 'Td, s';
      // Convert from series form
      p := StrToFloat(KpEdit.Text);
      i := StrToFloat(KiEdit.Text);
      d := StrToFloat(KdEdit.Text);
      // Clamp Kp, Ki to prevent division by zero
      if abs(p) < 1/rounding then
        i := sign(p)/rounding;
      if abs(i) < 1e-4 then
        i := sign(i)/rounding;
      // Ti = Kp/Ki
      KiEdit.Text := FloatToStr(round(rounding*p/i)/rounding);
      // Td = Kd/Kp
      KdEdit.Text := FloatToStr(round(rounding*d/p)/rounding);
    end;
  end;
end;

procedure TForm1.initializeModels(var heater: THeaterModel;
  var PID: TPIDcontrollerBase);
var
  Tamb, convection, mass, Cp: double;
  Kp, integral, derivative: double;
begin
  Tamb := StrToFloat(TambEdit.Text);
  convection:= StrToFloat(convectionEdit.Text);
  mass := StrToFloat(massEdit.Text);
  Cp := StrToFloat(CpEdit.Text);
  heater.initialize(Tamb, Cp, mass, convection, Tamb);

  Kp := StrToFloat(KpEdit.Text);
  integral := StrToFloat(KiEdit.Text);
  derivative := StrToFloat(KdEdit.Text);
  PID.initialize;
  if PIDTypeRadioGroup.ItemIndex = 0 then
    PID.setParametersParallel(Kp, integral, derivative)
  else
    PID.setParametersStandard(Kp, integral, derivative);
  PID.DerivativeFilterValue := StrToFloat(DerivativeFilterWeightEdit.Text);
  Tnoise := StrToFloat(TnoiseEdit.Text);
end;

end.

