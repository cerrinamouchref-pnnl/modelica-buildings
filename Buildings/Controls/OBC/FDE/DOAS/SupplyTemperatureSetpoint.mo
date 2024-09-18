within Buildings.Controls.OBC.FDE.DOAS;
block SupplyTemperatureSetpoint
  "This block caclulates the DOAS supply air temperature set point."

  parameter Real TSupLowSet(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=273.15+20
   "Minimum primary supply air temperature reset value";
   //final displayUnit="degC",

  parameter Real TSupHigSet(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=273.15+24
   "Maximum primary supply air temperature reset value";
   //final displayUnit="degC",

  parameter Real THigZon(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=273.15+25
   "Maximum zone temperature reset value";
   //final displayUnit="degC",

  parameter Real TLowZon(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=273.15+21
   "Minimum zone temperature reset value";
   //final displayUnit="degC",

  parameter Real TSupCooOff(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=2
   "Supply air temperature cooling set point offset.";

  parameter Real TSupHeaOff(
   final unit="K",
   final displayUnit="K",
   final quantity="ThermodynamicTemperature")=2
   "Supply air temperature heating set point offset.";

  // ---inputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uDehMod
    "True when dehumidification mode is active." annotation (Placement(
        transformation(extent={{-142,-102},{-102,-62}}), iconTransformation(
          extent={{-140,30},{-100,70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirHig(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Highest space temperature reported from all terminal units." annotation (
      Placement(transformation(extent={{-140,-28},{-100,12}}),
        iconTransformation(extent={{-140,-36},{-100,4}})));


  // ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput ySupCooSet
    "Supply air temperature cooling set point." annotation (Placement(
        transformation(extent={{100,28},{140,68}}), iconTransformation(extent={{100,28},
            {140,68}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput ySupHeaSet
    "Supply air temperature heating set point" annotation (Placement(
        transformation(extent={{100,-58},{140,-18}}), iconTransformation(extent={{100,-58},
            {140,-18}})));

  CDL.Interfaces.RealOutput ySupSet
    annotation (Placement(transformation(extent={{100,-6},{140,34}})));
  CDL.Interfaces.RealInput TZonHeaSet "Zone heating Setpoint"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  CDL.Interfaces.RealInput TZonCooSet "Zone Cooling Setpoint" annotation (
      Placement(transformation(extent={{-140,10},{-100,50}}),
        iconTransformation(extent={{-140,10},{-100,50}})));
protected
  Buildings.Controls.OBC.CDL.Reals.Line lin
    "Linear converter resets primary supply set point."
    annotation(Placement(transformation(extent={{-42,-8},{-22,12}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant TAirSupLowSet(final k=
        TSupLowSet)    "Low primary supply temperature set point reset value."
    annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant TAirSupHigSet(final k=
        TSupHigSet)    "High primary supply temperature set point reset value."
    annotation (Placement(transformation(extent={{-90,10},{-70,30}})));

  Buildings.Controls.OBC.CDL.Reals.Add addCooSet
    "Adds the cooling set point adjustment to the primary set point."
    annotation (Placement(transformation(extent={{22,26},{42,46}})));

  Buildings.Controls.OBC.CDL.Reals.Subtract addHeaSet
    "Subtracts the heating set point adjustment from the primary set point."
    annotation (Placement(transformation(extent={{22,-50},{42,-30}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant TAirSupCooOff(final k=
        TSupCooOff)
    "Supply air temperature cooling set point offset."
    annotation (Placement(transformation(extent={{-14,44},{6,64}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant TAirSupHeaSetOff(final k=
        TSupHeaOff)
               "Supply air temperature heating set point offset."
    annotation (Placement(transformation(extent={{-14,-66},{6,-46}})));
  Buildings.Controls.OBC.CDL.Reals.Switch swiDeh
    "Logical switch changes heating set point based on dehumidification mode."
    annotation (Placement(transformation(extent={{66,-42},{86,-22}})));


equation
  connect(lin.u, TAirHig)
    annotation (Line(points={{-44,2},{-94,2},{-94,-8},{-120,-8}},
                                                color={0,0,127}));

  connect(TAirSupHeaSetOff.y, addHeaSet.u2) annotation (Line(points={{8,-56},{14,
          -56},{14,-46},{20,-46}}, color={0,0,127}));

  connect(TAirSupCooOff.y, addCooSet.u1) annotation (Line(points={{8,54},{14,54},
          {14,42},{20,42}}, color={0,0,127}));

  connect(lin.y, addCooSet.u2)
    annotation (Line(points={{-20,2},{2,2},{2,30},{20,30}}, color={0,0,127}));

  connect(lin.y, addHeaSet.u1) annotation (Line(points={{-20,2},{2,2},{2,-34},{20,
          -34}}, color={0,0,127}));

  connect(addHeaSet.y, swiDeh.u3)
    annotation (Line(points={{44,-40},{64,-40}}, color={0,0,127}));

  connect(addCooSet.y, ySupCooSet)
    annotation (Line(points={{44,36},{82,36},{82,48},{120,48}},
                                                color={0,0,127}));

  connect(swiDeh.y, ySupHeaSet)
    annotation (Line(points={{88,-32},{106,-32},{106,-38},{120,-38}},
                                                  color={0,0,127}));

  connect(addCooSet.y, swiDeh.u1) annotation (Line(points={{44,36},{54,36},{54,-24},
          {64,-24}}, color={0,0,127}));

  connect(swiDeh.u2, uDehMod) annotation (Line(points={{64,-32},{54,-32},{54,-82},
          {-122,-82}}, color={255,0,255}));

  connect(TAirSupHigSet.y, lin.f1) annotation (Line(points={{-68,20},{-56,20},{
          -56,6},{-44,6}}, color={0,0,127}));

  connect(TAirSupLowSet.y, lin.f2) annotation (Line(points={{-68,-50},{-52,-50},
          {-52,-6},{-44,-6}}, color={0,0,127}));

  connect(lin.y, ySupSet) annotation (Line(points={{-20,2},{42,2},{42,14},{120,
          14}}, color={0,0,127}));
  connect(TZonHeaSet, lin.x2) annotation (Line(points={{-120,-40},{-96,-40},{
          -96,-12},{-64,-12},{-64,-2},{-44,-2}}, color={0,0,127}));
  connect(TZonCooSet, lin.x1) annotation (Line(points={{-120,30},{-92,30},{-92,
          48},{-50,48},{-50,10},{-44,10}}, color={0,0,127}));
  annotation (defaultComponentName="TSupSetpt",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(extent={{-90,180},{90,76}},lineColor={28,108,200},textStyle={TextStyle.Bold},textString="%name"),Rectangle(extent={{
              -100,98},{100,-102}},                                                                                                                                                                            lineColor={179,151,128},radius=10,fillColor={255,255,255},
            fillPattern=
FillPattern.Solid),Text(extent={{-94,-40},{-40,-58}},lineColor={28,108,200},textString="highSpaceT"),Text(extent={{-94,60},{-40,42}},lineColor={28,108,200},textString="dehumMode"),Text(extent={{40,50},{94,32}},lineColor={28,108,200},
textString="supCooSP"),Text(extent={{42,-30},{96,-48}},
lineColor={28,108,200},textString="supHeaSP"),Rectangle(extent={{14,22},{18,-22}},lineColor={0,140,72},fillColor={0,140,72},
            fillPattern=
FillPattern.Solid),Rectangle(extent={{14,22},{18,60}},lineColor={238,46,47},fillColor={255,0,0},
            fillPattern=
FillPattern.Solid),Rectangle(extent={{14,-60},{18,-22}},lineColor={28,108,200},fillColor={28,108,200},
            fillPattern=
FillPattern.Solid),Rectangle(extent={{-40,-4},{6,-6}},lineColor={162,29,33},fillColor={162,29,33},
            fillPattern=
FillPattern.Solid),Polygon(points={{6,-4},{-8,6},{-12,-4},{6,-4}},lineColor={162,29,33},fillColor={162,29,33},
            fillPattern=
FillPattern.Solid),
Text(extent={{40,8},{94,-10}},lineColor={28,108,200},
          textString="supPrimSP")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(revisions="<html>
<ul>
<li>
September 14, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info="<html>
<h4>Supply Temperature Set Points</h4>
<p>This block calculates the primary, cooling (<span style=\"font-family: Courier New;\">ySupCooSet</span>), and heating (<span style=\"font-family: Courier New;\">ySupHeaSet</span>) supply air temperature set points. The primary supply air temperature set point is reset from <span style=\"font-family: Courier New;\">TSupLowSet</span> to<span style=\"font-family: Courier New;\"> TSupHigSet</span> as the highest space temperature falls from <span style=\"font-family: Courier New;\">THigZon</span> to<span style=\"font-family: Courier New;\"> TLowZon</span>.</p>
<p>The supply air cooling set point is equal to the primary air temperature set point plus <span style=\"font-family: Courier New;\">TAirSupCooOff</span>. The supply air heating set point is equal to the primary air temperature set point minus<span style=\"font-family: Courier New;\"> TAirSupHeaOff</span>. </p>
<h4>Dehumidification Set Point</h4>
<p>When dehumidification mode (<span style=\"font-family: Courier New;\">dehumMode</span>) is active the supply air temperature heating set point is changed to equal the supply air temperature cooling set point. </p>
</html>"));
end SupplyTemperatureSetpoint;
