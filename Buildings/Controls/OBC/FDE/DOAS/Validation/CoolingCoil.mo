within Buildings.Controls.OBC.FDE.DOAS.Validation;
model CoolingCoil "This model simulates CoolingCoil"

   parameter Real erwDPadj(
  final unit = "K",
  final quantity = "TemperatureDifference") = 5
  "Value subtracted from ERW supply air dewpoint.";

  parameter CDL.Types.SimpleController controllerTypeDeh=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for cooling air in dehumidification mode";

  parameter Real kDeh(
  final unit="1") = 1
    "Gain of conPIDDeh controller";

  parameter Real TiDeh(
  final unit="s") = 60
    "Time constant of integrator block for conPIDDeh controller";

  parameter Real TdDeh(
  final unit="s") = 0.1
    "Time constant of derivative block for conPIDDeh controller";

  parameter CDL.Types.SimpleController controllerTypeRegOpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for regular cooling coil operation mode";

  parameter Real kRegOpe(
  final unit="1") = 1
    "Gain of conPIDRegOpe controller";

  parameter Real TiRegOpe(
  final unit="s")=60
    "Time constant of integrator block for conPIDRegOpe controller";

  parameter Real TdRegOpe(
  final unit="s")=0.1
    "Time constant of derivative block for conPIDRegOpe controller";

  parameter Real dehumSet(
    final min=0,
    final max=100)=60
    "Dehumidification set point.";

  parameter Real timThrDehDis(
    final unit="s",
    final quantity="Time")=600
    "Continuous time period for which measured relative humidity needs to fall below relative humidity threshold before dehumidification mode is disabled";

  parameter Real timDelDehEna(
    final unit="s",
    final quantity="Time")=120
    "Continuous time period for which supply fan needs to be on before enabling dehumidifaction mode";

  parameter Real timThrDehEna(
    final unit="s",
    final quantity="Time")=5
    "Continuous time period for which relative humidity rises above set point before dehumidifcation mode is enabled";
  Buildings.Controls.OBC.FDE.DOAS.CoolingCoil CoiCoo(
    erwDPadj(displayUnit="K") = erwDPadj,
    controllerTypeDeh=controllerTypeDeh,
    kDeh=kDeh,
    TiDeh=TiDeh,
    TdDeh=TdDeh,
    controllerTypeRegOpe=controllerTypeRegOpe,
    kRegOpe=kRegOpe,
    TiRegOpe=TiRegOpe,
    TdRegOpe=TdRegOpe)
    annotation (Placement(transformation(extent={{48,-24},{68,-4}})));

  Buildings.Controls.OBC.CDL.Logical.Sources.Pulse SFproof(
    width=0.8,
    period=5760,
    shift=300)
    annotation (Placement(transformation(extent={{-62,60},{-42,80}})));

   Buildings.Controls.OBC.CDL.Reals.Sources.Sin saTGen(
    amplitude=3,
    freqHz=1/4800,
    phase=0.87266462599716,
    offset=295,
    startTime=0)
    annotation (Placement(transformation(extent={{-26,40},{-6,60}})));

   Buildings.Controls.OBC.CDL.Reals.Sources.Sin TCoiCoo(
    amplitude=3,
    freqHz=1/4800,
    phase=0,
    offset=293,
    startTime=0) "Cooling coil temperature signal"
    annotation (Placement(transformation(extent={{-60,-28},{-40,-8}})));

  Buildings.Controls.OBC.CDL.Logical.Sources.Pulse dehumMode(
    width=0.5,
    period=4700,
    shift=1000)
    annotation (Placement(transformation(extent={{-26,-6},{-6,14}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Sin supCooGen(
    amplitude=2,
    freqHz=1/3100,
    offset=293,
    startTime=12)
    annotation (Placement(transformation(extent={{-62,16},{-42,36}})));

   Buildings.Controls.OBC.CDL.Reals.Sources.Sin erwTGen(
    amplitude=3,
    freqHz=1/4800,
    phase=0,
    offset=294,
    startTime=0)
    annotation (Placement(transformation(extent={{-60,-66},{-40,-46}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Sin erwHumGen(
    amplitude=0.25,
    freqHz=1/3600,
    offset=0.5,
    startTime=1250)
    annotation (Placement(transformation(extent={{-26,-48},{-6,-28}})));

equation
  connect(SFproof.y, CoiCoo.uFanSupPro) annotation (Line(points={{-40,70},{18,70},
          {18,-5.6},{45.8,-5.6}}, color={255,0,255}));

  connect(saTGen.y, CoiCoo.TAirSup) annotation (Line(points={{-4,50},{12,50},{12,
          -8.4},{45.8,-8.4}}, color={0,0,127}));

  connect(TCoiCoo.y, CoiCoo.TAirDis) annotation (Line(points={{-38,-18},{14,-18},
          {14,-17.2},{45.8,-17.2}}, color={0,0,127}));

  connect(dehumMode.y, CoiCoo.uDeh) annotation (Line(points={{-4,4},{0,4},{0,-14.6},
          {45.8,-14.6}}, color={255,0,255}));

  connect(supCooGen.y, CoiCoo.TAirSupSetCoo) annotation (Line(points={{-40,26},{
          6,26},{6,-11.2},{45.8,-11.2}}, color={0,0,127}));

  connect(erwHumGen.y, CoiCoo.phiAirEneRecWhe) annotation (Line(points={{-4,-38},
          {14,-38},{14,-19.8},{45.8,-19.8}}, color={0,0,127}));

  connect(erwTGen.y, CoiCoo.TAirEneRecWhe) annotation (Line(points={{-38,-56},{18,
          -56},{18,-22.4},{45.8,-22.4}}, color={0,0,127}));

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(lineColor = {75,138,73},
fillColor={255,255,255},
            fillPattern=
FillPattern.Solid,extent={{-100,-100},{100,100}}),
Polygon(lineColor = {0,0,255},fillColor = {75,138,73},
pattern = LinePattern.None,
            fillPattern=
FillPattern.Solid,
points={{-36,60},{64,0},{-36,-60},{-36,60}})}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(revisions="<html>
<ul>
<li>
September 17, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info="<html>
<p>
This example simulates
<a href=\"modelica://Buildings.Controls.OBC.FDE.DOAS.CoolingCoil\">
Buildings.Controls.OBC.FDE.DOAS.CoolingCoil</a>.
</p>
</html>"),
    experiment(StopTime=5760, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Controls/OBC/FDE/DOAS/Validation/CoolingCoil.mos"
    "Simulate and plot"));
end CoolingCoil;
