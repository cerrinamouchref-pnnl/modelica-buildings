within Buildings.Controls.OBC.FDE.DOAS.Subsequences;
block EconomizerMode "This block calculates when economizer mode is active."
  parameter Real dTEcoThr(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 2
  "Threshold temperature difference between return air and outdoor air temperature above which economizer mode is enabled";

  // ---inputs---

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when supply fan is proven on" annotation (Placement(
      visible=true,
      transformation(
        origin={2,-40},
        extent={{-142,42},{-102,82}},
        rotation=0),
      iconTransformation(
        origin={0,0},
        extent={{-142,50},{-102,90}},
        rotation=0)));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirOut
    "Outside air temperature" annotation (Placement(
      visible=true,
      transformation(
        origin={2,-38},
        extent={{-142,6},{-102,46}},
        rotation=0),
      iconTransformation(
        origin={0,0},
        extent={{-142,-20},{-102,20}},
        rotation=0)));


  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupSetCoo
    "Supply air temperature cooling set point." annotation (Placement(
      visible=true,
      transformation(
        origin={2,-34},
        extent={{-142,-30},{-102,10}},
        rotation=0),
      iconTransformation(
        origin={0,0},
        extent={{-142,-90},{-102,-50}},
        rotation=0)));

  // ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yEcoMod "True when economizer mode is active."
  annotation (
    Placement(transformation(extent = {{104, -20}, {144, 20}}), iconTransformation(extent = {{102, -20}, {142, 20}})));

protected
  Buildings.Controls.OBC.CDL.Reals.Greater gre(h=dTEcoThr)
  "True if OAT > supCooSP."
  annotation (
    Placement(visible = true, transformation(origin = {-24, 34}, extent = {{-20, -46}, {0, -26}}, rotation = 0)));

  Buildings.Controls.OBC.CDL.Logical.And andEcoModEna
    "Logical AND; true when fan is proven on and temperature set point conditions are met."
    annotation (Placement(transformation(extent={{74,-10},{94,10}})));

  Buildings.Controls.OBC.CDL.Logical.Not not1
  annotation (
    Placement(visible = true, transformation(origin = {-4, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));


equation
  connect(andEcoModEna.u1, uFanSupPro) annotation (Line(points={{72,0},{66,0},{
          66,22},{-120,22}}, color={255,0,255}));

  connect(andEcoModEna.y, yEcoMod)
    annotation (Line(points={{96,0},{124,0}}, color={255,0,255}));

  connect(TAirOut, gre.u1) annotation (Line(points={{-120,-12},{-82,-12},{-82,-2},
          {-46,-2}}, color={0,0,127}));

  connect(TAirSupSetCoo, gre.u2) annotation (Line(points={{-120,-44},{-46,-44},
          {-46,-10}}, color={0,0,127}));

  connect(gre.y, not1.u) annotation (
    Line(points = {{-22, -2}, {-16, -2}}, color = {255, 0, 255}));

  connect(not1.y, andEcoModEna.u2) annotation (Line(points={{8,-2},{40,-2},{40,-8},
          {72,-8}}, color={255,0,255}));
  annotation (
    defaultComponentName = "EconMod",
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(fillColor = {255, 255, 255},
    fillPattern=
FillPattern.Solid, extent={{-100,100},{100,-100}},     radius=0),    Text(textColor = {28, 108, 200}, extent={{-88,186},
              {92,82}},                                                                                                                textString = "%name", textStyle = {TextStyle.Bold}), Text(textColor = {28, 108, 200}, extent = {{-96, 78}, {-52, 64}}, textString = "supFanProof"), Text(textColor = {28, 108, 200}, extent = {{-108, 8}, {-64, -6}}, textString = "oaT"), Text(textColor = {28, 108, 200}, extent = {{-96, -64}, {-56, -76}}, textString = "supCooSP"), Text(textColor = {28, 108, 200}, extent = {{58, 6}, {96, -6}}, textString = "ecoMode"), Rectangle(lineColor = {28, 108, 200},fillColor = {0, 0, 255},
      fillPattern=
FillPattern.Solid, extent = {{18, -2}, {22, -64}}), Text(textColor = {28, 108, 200}, extent = {{16, -32}, {54, -44}}, textString = "On"), Text(textColor = {28, 108, 200}, extent = {{14, 38}, {52, 26}}, textString = "Off"), Rectangle(lineColor = {244, 125, 35}, fillColor = {244, 125, 35},
      fillPattern=
FillPattern.Solid, extent = {{18, 60}, {22, -2}}), Line(points = {{-38, 46}, {6, 46}}), Line(points = {{-38, 18}, {6, 18}}), Ellipse(fillColor = {255, 255, 255},
      fillPattern=
 FillPattern.Solid, extent = {{-18, 34}, {-14, 30}}), Line(points = {{-16, 34}, {-16, 46}}), Line(points = {{-16, 18}, {-16, 30}}), Line(points = {{-36, -24}, {8, -24}}), Line(points = {{-12, -38}, {2, -38}}), Ellipse(fillColor = {255, 255, 255},
      fillPattern=
FillPattern.Solid, extent = {{-16, -36}, {-12, -40}}), Line(points = {{-36, -52}, {8, -52}}), Line(points = {{-30, -38}, {-16, -38}}), Text(textColor = {28, 108, 200}, extent = {{-32, 4}, {6, -8}}, textString = "%add2.y")}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Documentation(info="<html>
<h4>Economizer Mode</h4>
<p>This block enables economizer mode (<span style=\"font-family: Courier New;\">yEcoMod</span>) when the supply air fan is proven (<span style=\"font-family: Courier New;\">uFanSupPro</span>) and outside air temperature (<span style=\"font-family: Courier New;\">TAirOut</span>) is below the supply air temperature cooling set point (<span style=\"font-family: Courier New;\">TAirSupSetCoo</span>) minus an offset (<span style=\"font-family: Courier New;\">dTEcoThr</span>). Economizer mode is disabled when outside air temperature rises above the supply air temperature cooling set point. </p>
</html>", revisions = "<html>
<ul>
<li>
September 15, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>"));
end EconomizerMode;
