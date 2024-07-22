within Buildings.Controls.OBC.FDE.DOAS;
block SupplyFanController "This block manages start, stop, status, and speed of the supply fan."

   parameter Boolean is_vav = true
  "True: System has zone terminals with variable damper position. False: System has zone terminals with constant damper position.";

  parameter Real yMinDamSet(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 125
  "Minimum down duct static pressure reset value" annotation(Dialog(group = "DDSP range"));

  parameter Real yMaxDamSet(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 500
  "Maximum down duct static pressure reset value" annotation(Dialog(group = "DDSP range"));

  parameter Real damSet(
  min = 0,
  max = 1,
  final unit = "1") = 0.9
  "DDSP terminal damper percent open set point";

  parameter Real kDam(
   final unit= "1") = 0.5
  "Damper position setpoint PI gain value k.";

  parameter Real TiDam(
   final unit= "s") = 60
  "Damper position setpoint PI time constant value Ti.";

  parameter Real TdDam(
   final unit= "s") = 0.1 "Time constant of derivative block for conPIDDam";

  parameter CDL.Types.SimpleController controllerTypeDam=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "Type of controller";

  parameter Real dPDucSetCV(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 250 "Constant volume down duct static pressure set point";

  parameter Real fanSpeMin(
   final unit= "m/s") = 0.0000001
  "Minimum Fan Speed";

  parameter Real kFanSpe(
   final unit= "1") = 0.005 "
  Fan speed set point SAT PI gain value k.";

  parameter Real TdFanSpe(
   final unit= "s") = 0 "Time constant of derivative block for conPIDFanSpe";

  parameter Real TiFanSpe(
   final unit= "s") = 10
  "Fan speed set point SAT PI time constant value Ti.";

  parameter CDL.Types.SimpleController controllerTypeFanSpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "Type of controller";



  // ---inputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput Occ
  "True when occupied mode is active"
  annotation(Placement(transformation(extent = {{-142, 60}, {-102, 100}}), iconTransformation(extent = {{-140, 50}, {-100, 90}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when supply fan is proven on." annotation (Placement(transformation(
          extent={{-142,22},{-102,62}}), iconTransformation(extent={{-140,-56},
            {-100,-16}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uDamMaxOpe  if is_vav
    "Maximum damper opening position from all terminal units served"
    annotation (Placement(transformation(extent={{-142,-24},{-102,16}}),
        iconTransformation(extent={{-140,14},{-100,54}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dPAirDucSta
    "Down duct static pressure measurement." annotation (Placement(
        transformation(extent={{-142,-106},{-102,-66}}), iconTransformation(
          extent={{-140,-92},{-100,-52}})));

  // ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yFanSup
    "Supply fan enable signal" annotation (Placement(transformation(extent={{
            102,60},{142,100}}), iconTransformation(extent={{100,32},{140,72}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yFanSupSpe
    "Supply fan speed signal" annotation (Placement(transformation(extent={{102,
            -50},{142,-10}}), iconTransformation(extent={{100,-64},{140,-24}})));
// protected
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant DamSet(k=damSet)
               if is_vav "Most open damper position set point."
    annotation (Placement(transformation(extent={{-98,4},{-78,24}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conPDucSetCV(final k=
        dPDucSetCV)    if not is_vav
    "DDSP set point for constant volume systems."
    annotation (Placement(transformation(extent={{-72,-64},{-52,-44}})));
  Buildings.Controls.OBC.CDL.Reals.Switch swiFanSpe
    "Swtich passes fan speed set point when true; 0 when false."
    annotation (Placement(transformation(extent={{60,-42},{80,-22}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conFanMinSpe(k=fanSpeMin)
    "Minimum fan speed"
    annotation (Placement(transformation(extent={{-26,-64},{-6,-44}})));

  Buildings.Controls.OBC.CDL.Reals.PID conPIDDam(
    controllerType=controllerTypeDam,
    Ti=TiDam,
    k=kDam,
    Td=TdDam,
    yMax=yMaxDamSet,
    yMin=yMinDamSet,
    reverseActing=false)
                 if is_vav "PID for most open damper" annotation (Placement(
        visible=true, transformation(
        origin={-38,22},
        extent={{-10,-10},{10,10}},
        rotation=0)));

  CDL.Reals.PIDWithReset               conPIDFanSpe(
    controllerType=controllerTypeFanSpe,
    Ti=TiFanSpe,
    k=kFanSpe,
    Td=TdFanSpe)
  annotation(Placement(visible = true, transformation(origin={20,-10},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Buildings.Controls.OBC.CDL.Logical.Timer
  tim(t = 300)
  annotation(Placement(visible = true, transformation(origin = {-8, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));


equation
  connect(conFanMinSpe.y, swiFanSpe.u3) annotation (Line(points={{-4,-54},{4,-54},
          {4,-40},{58,-40}}, color={0,0,127}));

  connect(uFanSupPro, swiFanSpe.u2) annotation (Line(points={{-122,42},{0,42},{0,
          -32},{58,-32}},   color={255,0,255}));

  connect(DamSet.y, conPIDDam.u_s) annotation (Line(points={{-76,14},{-71,14},{
          -71,22},{-50,22}}, color={0,0,127}));

  connect(uDamMaxOpe, conPIDDam.u_m)
    annotation (Line(points={{-122,-4},{-38,-4},{-38,10}}, color={0,0,127}));

  connect(dPAirDucSta, conPIDFanSpe.u_m)
    annotation (Line(points={{-122,-86},{20,-86},{20,-22}}, color={0,0,127}));

  connect(Occ, tim.u) annotation (
    Line(points = {{-122, 80}, {-69, 80}, {-69, 88}, {-20, 88}}, color = {255, 0, 255}));

  connect(tim.passed, yFanSup)
    annotation (Line(points={{4,80},{122,80}}, color={255,0,255}));

  connect(conPDucSetCV.y, conPIDFanSpe.u_s) annotation (Line(points={{-50,-54},{
          -44,-54},{-44,-10},{8,-10}}, color={0,0,127}));
  connect(conPIDDam.y, conPIDFanSpe.u_s) annotation (Line(points={{-26,22},{-12,
          22},{-12,-10},{8,-10}}, color={0,0,127}));
  connect(conPIDFanSpe.y, swiFanSpe.u1) annotation (Line(points={{32,-10},{30,-10},
          {30,-24},{58,-24}}, color={0,0,127}));
  connect(swiFanSpe.y, yFanSupSpe) annotation (Line(points={{82,-32},{88,-32},{88,
          -30},{122,-30}}, color={0,0,127}));
  connect(uFanSupPro, conPIDFanSpe.trigger) annotation (Line(points={{-122,42},
          {0,42},{0,-32},{14,-32},{14,-22}}, color={255,0,255}));
  annotation (
    defaultComponentName = "SFcon",
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(lineColor = {179, 151, 128}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 10), Rectangle(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-8, 40}, {54, -14}}), Text(textColor = {28, 108, 200}, extent = {{-90, 180}, {90, 76}}, textString = "%name", textStyle = {TextStyle.Bold}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-54, 40}, {28, -42}}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-24, 12}, {0, -12}}), Text(textColor = {28, 108, 200}, extent = {{-108, 78}, {-64, 62}}, textString = "occ"), Text(textColor = {28, 108, 200}, extent = {{-100, -30}, {-58, -44}}, textString = "Status"), Text(textColor = {28, 108, 200}, extent = {{62, 60}, {106, 44}}, textString = "Start"), Text(textColor = {28, 108, 200}, extent = {{56, 8}, {106, -8}}, textString = "Proof"), Text(textColor = {28, 108, 200}, extent = {{-102, -66}, {-60, -78}}, textString = "DDSP"), Text(textColor = {28, 108, 200}, extent = {{56, -36}, {106, -52}}, textString = "Speed"), Line(points = {{-54, 54}, {-10, 52}, {-50, 52}}, color = {0, 127, 0}, pattern = LinePattern.None), Line(points = {{-52, 50}, {-2, 48}, {30, 54}, {-26, 68}, {134, 70}, {148, 22}, {152, 42}}, color = {0, 127, 0}, pattern = LinePattern.None), Line(points = {{-68, 72}, {6, 72}}, color = {0, 127, 0}, pattern = LinePattern.None), Line(points = {{-52, 70}, {6, 66}, {-28, 66}}, color = {0, 127, 0}, pattern = LinePattern.None), Line(points = {{42, 60}, {36, 54}, {28, 70}, {42, 62}}, color = {0, 127, 0}, pattern = LinePattern.None), Text(textColor = {28, 108, 200}, extent = {{-96, 44}, {-52, 28}}, textString = "openDam")}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Documentation(revisions = "<html>
<ul>
<li>
September 11, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info="<html>
<h4>Supply Fan Start/Stop.</h4>
<p>This block commands the supply fan to start (<span style=\"font-family: Courier New;\">yFanSup</span>) when the unit enters occupied (<span style=\"font-family: Courier New;\">Occ</span>) mode. When supply fan status (<span style=\"font-family: Courier New;\">uFanSupPro</span>) is proven, fan speed control <span style=\"font-family: Courier New;\">(yFanSupSpe</span>) is enabled and the supply fan proof (<span style=\"font-family: Courier New;\">uFanSupPro</span>) is turned on.</p>
<h4>Down Duct Static Pressure Control</h4>
<p>The supply fan speed (<span style=\"font-family: Courier New;\">yFanSupSpe</span>) is modulated to maintain the down duct static pressure (<span style=\"font-family: Courier New;\">conPDucSetCV</span>) at set point. The down duct set point is reset between minimum (<span style=\"font-family: Courier New;\">yMinDamSet</span>) and maximum (<span style=\"font-family: Courier New;\">yMaxDamSet</span>) values determined by TAB. The reset is based on the most open damper (<span style=\"font-family: Courier New;\">uDamMaxOpe</span>) remaining at a specific position (<span style=\"font-family: Courier New;\">DamSet</span>) (i.e. The terminal unit air flow set point is satisfied with its primary air damper 90&percnt; open).</p>
<p>There is also an option for a fixed down duct static pressure set point (<span style=\"font-family: Courier New;\">PDucSetVV</span>) when the variable volume parameter <span style=\"font-family: Courier New;\">is_vav</span> is false. </p>
</html>"));
end SupplyFanController;
