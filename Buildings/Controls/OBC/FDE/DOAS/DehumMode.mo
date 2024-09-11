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

  CDL.Reals.AddParameter phiOffSet(p=-dehumOff)
    "Calculate cooling setpoint temperature for air in dehumidification mode"
    annotation (Placement(visible=true, transformation(
        origin={-78,18},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  CDL.Reals.AddParameter phiOffSet1(p=dehumOff)
    "Calculate cooling setpoint temperature for air in dehumidification mode"
    annotation (Placement(visible=true, transformation(
        origin={-74,-20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
protected
  Buildings.Controls.OBC.CDL.Logical.Latch lat
    "Latches true when retHum > dehumSet; resets when 
     retHum < dehumSet for dehumDelay time"
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));

  Buildings.Controls.OBC.CDL.Reals.Greater gre
    "True when return humidity is greater than set point"
    annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));

  Buildings.Controls.OBC.CDL.Reals.Less les
    "True when return humidity is less than set point"
    annotation (Placement(transformation(extent={{-44,-42},{-24,-22}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant phiAirDehSet(final k=
        dehumSet) "Dehumidification set point"
    annotation (Placement(transformation(extent={{-90,-72},{-70,-52}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDelDehDis(delayTime=
        timThrDehDis)
    "Minimum dehumidification time before disable when below set point."
    annotation (Placement(transformation(extent={{-14,-42},{6,-22}})));

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
    annotation (Placement(transformation(extent={{-14,-10},{6,10}})));


equation

  connect(phiAirDehSet.y, les.u2)
    annotation (Line(points={{-68,-62},{-54,-62},{-54,-40},{-46,-40}},
                                                   color={0,0,127}));

  connect(phiAirDehSet.y, gre.u2) annotation (Line(points={{-68,-62},{-54,-62},{
          -54,-8},{-46,-8}}, color={0,0,127}));

  connect(les.y,truDelDehDis. u)
    annotation (Line(points={{-22,-32},{-16,-32}}, color={255,0,255}));

  connect(truDelDehDis.y, lat.clr) annotation (Line(points={{8,-32},{14,-32},{14,
          -6},{20,-6}}, color={255,0,255}));

  connect(lat.y, andDehOpe.u2)
    annotation (Line(points={{44,0},{56,0}}, color={255,0,255}));

  connect(yDehMod, yDehMod)
    annotation (Line(points={{122,8},{122,8}}, color={255,0,255}));

  connect(andDehOpe.y, yDehMod)
    annotation (Line(points={{80,8},{122,8}}, color={255,0,255}));

  connect(uFanSupPro, minimumRunDeh.u)
    annotation (Line(points={{-122,38},{-46,38}}, color={255,0,255}));

  connect(minimumRunDeh.y, andDehOpe.u1) annotation (Line(points={{-22,38},{48,38},
          {48,8},{56,8}}, color={255,0,255}));

  connect(gre.y,truDelDehEna. u)
    annotation (Line(points={{-22,0},{-16,0}}, color={255,0,255}));

  connect(truDelDehEna.y, lat.u)
    annotation (Line(points={{8,0},{20,0}}, color={255,0,255}));

  connect(uFanSupPro, minimumRunDeh.u)
    annotation (Line(points={{-122,38},{-46,38}}, color={255,0,255}));

  connect(phiAirRet, phiOffSet.u) annotation (Line(points={{-122,0},{-96,0},{-96,
          10},{-98,10},{-98,18},{-90,18}}, color={0,0,127}));
  connect(phiOffSet.y, gre.u1) annotation (Line(points={{-66,18},{-56,18},{-56,0},
          {-46,0}}, color={0,0,127}));
  connect(phiAirRet, phiOffSet1.u) annotation (Line(points={{-122,0},{-92,0},{-92,
          -20},{-86,-20}}, color={0,0,127}));
  connect(phiOffSet1.y, les.u1) annotation (Line(points={{-62,-20},{-56,-20},{-56,
          -32},{-46,-32}}, color={0,0,127}));
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
