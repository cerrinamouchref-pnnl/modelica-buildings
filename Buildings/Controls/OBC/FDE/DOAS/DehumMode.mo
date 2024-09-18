within Buildings.Controls.OBC.FDE.DOAS;
block DehumMode
  "This block calculates when dehumidification mode is active."

  parameter Real dehumSet(
    final min=0,
    final max=100)=0.6
   "Dehumidification set point.";


  parameter Real dehumOff(
    final min=0,
    final max=100)=0.05
   "Relative humidity offset";

  parameter Real timThrDehDis(
    final unit="s",
    final quantity="Time")=600
    "Continuous time period for which measured relative humidity needs to fall below relative humidity threshold before dehumidification mode is disabled";

  parameter Real timDelDehEna(
    final unit="s",
    final quantity="Time")=180
    "Continuous time period for which supply fan needs to be on before enabling dehumidifaction mode";

  parameter Real timThrDehEna(
    final unit="s",
    final quantity="Time")=300
    "Continuous time period for which relative humidity rises above set point before dehumidifcation mode is enabled";


// ---inputs---

 Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when supply fan is proven on" annotation (Placement(transformation(
          extent={{-142,18},{-102,58}}), iconTransformation(extent={{-142,52},{-102,
            92}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirRet(
    final unit="1",
    final min=0,
    final max=100) "Return air relative humidity sensor." annotation (Placement(
        transformation(extent={{-142,-20},{-102,20}}), iconTransformation(
          extent={{-142,-20},{-102,20}})));

// ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yDehMod
    "True when dehumidification mode is active." annotation (Placement(
        transformation(extent={{102,-12},{142,28}}), iconTransformation(extent={
            {102,-20},{142,20}})));

  CDL.Reals.Hysteresis hys(uLow=0.55, uHigh=0.65)
    annotation (Placement(transformation(extent={{-40,-24},{-20,-4}})));

protected
  Buildings.Controls.OBC.CDL.Logical.And andDehOpe "Logical AND; true when minimum fan runtime is met and 
      return humidity set point conditions are met."
    annotation (Placement(transformation(extent={{58,-2},{78,18}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay minimumRunDeh(delayTime=
        timDelDehEna, delayOnInit=true)
    "Minimum supply fan runtime before enabling dehum mode."
    annotation (Placement(transformation(extent={{-44,28},{-24,48}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDelDehEna(final delayTime=
        timThrDehEna,    final delayOnInit=true) "Delays the initial trigger for latch to correctly capture true state 
      when CDL starts with humidity above set point."
    annotation (Placement(transformation(extent={{20,-14},{40,6}})));


equation

  connect(yDehMod, yDehMod)
    annotation (Line(points={{122,8},{122,8}}, color={255,0,255}));

  connect(andDehOpe.y, yDehMod)
    annotation (Line(points={{80,8},{122,8}}, color={255,0,255}));

  connect(uFanSupPro, minimumRunDeh.u)
    annotation (Line(points={{-122,38},{-46,38}}, color={255,0,255}));

  connect(minimumRunDeh.y, andDehOpe.u1) annotation (Line(points={{-22,38},{48,38},
          {48,8},{56,8}}, color={255,0,255}));

  connect(uFanSupPro, minimumRunDeh.u)
    annotation (Line(points={{-122,38},{-46,38}}, color={255,0,255}));

  connect(phiAirRet, hys.u) annotation (Line(points={{-122,0},{-50,0},{-50,-14},
          {-42,-14}},color={0,0,127}));
  connect(hys.y, truDelDehEna.u) annotation (Line(points={{-18,-14},{10,-14},{
          10,-4},{18,-4}},
                        color={255,0,255}));
  connect(truDelDehEna.y, andDehOpe.u2) annotation (Line(points={{42,-4},{42,
          -12},{56,-12},{56,0}}, color={255,0,255}));
  annotation (defaultComponentName="DehumMod",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(extent={{-90,180},{90,76}}, lineColor={28,108,200}, textStyle={TextStyle.Bold}, textString="%name"),Rectangle(extent={{-100,100},{100,-100}}, lineColor={179,151,128}, radius=10, fillColor={255,255,255},
            fillPattern=
FillPattern.Solid),Text(extent={{42,10},{96,-8}},lineColor={28,108,200},textString="dehumMode"),Text(extent={{-100,8},{-56,-6}},lineColor={28,108,200},textString="retHum"),Rectangle(extent={{4,60},{8,-2}},lineColor={28,108,200},fillColor={0,0,255},
            fillPattern=
FillPattern.Solid),Rectangle(extent={{4,-2},{8,-64}},lineColor={244,125,35},fillColor={244,125,35},
            fillPattern=
FillPattern.Solid),Text(extent={{-2,42},{36,30}},lineColor={28,108,200},textString="On"),Text(extent={{-2,-38},{36,-50}},lineColor={28,108,200},textString="Off"),Text(extent={{-36,4},{2,-8}},lineColor={28,108,200},textString
            =                                                                                                                                                                                           "%dehumSet"),Text(extent={{-96,80},{-52,66}},lineColor={28,108,200},textString
            =                                                                                                                                                                                           "supFanProof")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<h4>Dehumidification Mode</h4>
<p>This block enables dehumidification mode (yDehMod) when the supply air fan is proven (uFanSupPro) for a minimum runtime (<span style=\"font-family: Courier New;\">minimumRunDeh</span>) and return air humidity (<span style=\"font-family: Courier New;\">phiAirRet</span>) is above set point <span style=\"font-family: Courier New;\">phiAirDehSe</span>t). Dehumidification mode is disabled when return air humidity falls below set point for a minimum delay period (<span style=\"font-family: Courier New;\">delaytimeDeh</span>). </p>
</html>", revisions="<html>
<ul>
<li>
September 15, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>"),
    experiment(
      StartTime=19300000,
      StopTime=19600000,
      Interval=600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"));
end DehumMode;
