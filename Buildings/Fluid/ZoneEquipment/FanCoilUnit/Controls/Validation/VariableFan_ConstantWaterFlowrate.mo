within Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls.Validation;
model VariableFan_ConstantWaterFlowrate
  "Validation model for controller with constant water flow rates and variable speed fan"
  extends Modelica.Icons.Example;

  Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls.VariableFan_ConstantWaterFlowrate
    conVarFanConWat(
    final nRowOccSch=5,
    final tableOcc=[0,0; 15,1; 30,0; 45,1; 60,1],
    final timeScaleOcc=1,
    final tFanEnaDel=2,
    final tFanEna=5)
    "Instance of controller with variable fan speed and constant water flowrate"
    annotation (Placement(transformation(extent={{10,-10},{30,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con(
    final k=273.15+25)
    "Zone cooling setpoint signal"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con1(
    final k=273.15+23)
    "Zone heating setpoint signal"
    annotation (Placement(transformation(extent={{-40,-50},{-20,-30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin(
    final amplitude=2,
    final freqHz=1/60,
    final offset=273.15+24)
    "Zone temperature signal"
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));

equation
  connect(con.y, conVarFanConWat.TCooSet)
    annotation (Line(points={{-18,0},{-6,0},{-6,-2},{8,-2}},
                                             color={0,0,127}));
  connect(con1.y, conVarFanConWat.THeaSet) annotation (Line(points={{-18,-40},{0,
          -40},{0,-6},{8,-6}},   color={0,0,127}));
  connect(sin.y, conVarFanConWat.TZon)
    annotation (Line(points={{-18,40},{0,40},{0,2},{8,2}}, color={0,0,127}));
  connect(conVarFanConWat.yFan, conVarFanConWat.uFan) annotation (Line(points={{
          32,-6},{40,-6},{40,30},{4,30},{4,6},{8,6}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(file= "modelica://Buildings/Resources/Scripts/Dymola/Fluid/ZoneEquipment/FanCoilUnit/Controls/Validation/VariableFan_ConstantWaterFlowrate.mos"
      "Simulate and plot"));
end VariableFan_ConstantWaterFlowrate;