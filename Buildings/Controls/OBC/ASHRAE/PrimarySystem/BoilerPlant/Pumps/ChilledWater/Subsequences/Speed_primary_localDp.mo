within Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Pumps.ChilledWater.Subsequences;

block Speed_primary_localDp
 
    "Pump speed control for primary-only plants where the remote DP sensor(s) is not hardwired to the plant controller, but a local DP sensor is hardwired"
  parameter Integer nSen = 2
    "Total number of remote differential pressure sensors";

  parameter Integer nPum = 2
    "Total number of hot water pumps";

  parameter Modelica.SIunits.PressureDifference minLocDp(
    final min=0,
    final displayUnit="Pa")=5*6894.75
    "Minimum hot water loop local differential pressure setpoint";

  parameter Modelica.SIunits.PressureDifference maxLocDp(
    final min=minLocDp,
    final displayUnit="Pa") = 15*6894.75
    "Maximum hot water loop local differential pressure setpoint";

  parameter Real minPumSpe = 0.1
    "Minimum pump speed";

  parameter Real maxPumSpe = 1
    "Maximum pump speed";

  parameter Buildings.Controls.OBC.CDL.Types.SimpleController controllerType=
    Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "Type of controller"
    annotation(Dialog(group="Speed controller"));

  parameter Real k=1
    "Gain of controller"
    annotation(Dialog(group="Speed controller"));

  parameter Modelica.SIunits.Time Ti=0.5
    "Time constant of integrator block"
    annotation(Dialog(group="Speed controller"));

  parameter Modelica.SIunits.Time Td=0.1
    "Time constant of derivative block"
    annotation (Dialog(group="Speed controller",
      enable=
      controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PD or
      controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PID));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWat_local(
    final unit="Pa",
    final quantity="PressureDifference")
    "Hot water differential static pressure from local sensor"
    annotation (Placement(transformation(extent={{-180,80},{-140,120}}),
      iconTransformation(extent={{-140,60},{-100,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uChiWatPum[nPum]
    "Hot water pump status"
    annotation (Placement(transformation(extent={{-180,-40},{-140,0}}),
      iconTransformation(extent={{-140,20},{-100,60}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWat_remote[nSen](
    final unit=fill("Pa", nSen),
    final quantity=fill("PressureDifference", nSen))
    "Hot water differential static pressure from remote sensor"
    annotation (Placement(transformation(extent={{-180,-110},{-140,-70}}),
      iconTransformation(extent={{-140,-60},{-100,-20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWatSet(
    final unit="Pa",
    final quantity="PressureDifference")
    "Hot water differential static pressure setpoint"
    annotation (Placement(transformation(extent={{-180,-140},{-140,-100}}),
      iconTransformation(extent={{-140,-100},{-100,-60}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yChiWatPumSpe(
    final min=minPumSpe,
    final max=maxPumSpe,
    final unit="1")
    "Hot water pump speed"
    annotation (Placement(transformation(extent={{140,100},{180,140}}),
      iconTransformation(extent={{100,-20},{140,20}})));

  Buildings.Controls.OBC.CDL.Continuous.LimPID conPID1(
    final controllerType=controllerType,
    final k=k,
    final Ti=Ti,
    final Td=Td,
    final yMax=1,
    final yMin=0,
    final reverseAction=true,
    final reset=Buildings.Controls.OBC.CDL.Types.Reset.Parameter,
    final y_reset=0)
    "Pump speed controller"
    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));

  Buildings.Controls.OBC.CDL.Continuous.Line pumSpe
    "Pump speed"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));

  Buildings.Controls.OBC.CDL.Continuous.MultiMax maxRemDP(
    final nin=nSen)
    "Highest output from differential pressure control loops"
    annotation (Placement(transformation(extent={{40,-30},{60,-10}})));

  Buildings.Controls.OBC.CDL.Continuous.Line locDpSet
    "Local differential pressure setpoint"
    annotation (Placement(transformation(extent={{100,-30},{120,-10}})));

  Buildings.Controls.OBC.CDL.Continuous.LimPID conPID[nSen](
    final controllerType=fill(controllerType, nSen),
    final k=fill(k, nSen),
    final Ti=fill(Ti, nSen),
    final Td=fill(Td, nSen),
    final yMax=fill(1, nSen),
    final yMin=fill(0, nSen),
    final reverseAction=fill(true, nSen),
    final reset=fill(Buildings.Controls.OBC.CDL.Types.Reset.Parameter, nSen),
    final y_reset=fill(0, nSen))
    "Pump speed controller"
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));

protected
  Buildings.Controls.OBC.CDL.Routing.RealReplicator reaRep(
    final nout=nSen)
    "Replicate real input"
    annotation (Placement(transformation(extent={{-120,-130},{-100,-110}})));

  Buildings.Controls.OBC.CDL.Routing.BooleanReplicator booRep(
    final nout=nSen)
    "Replicate boolean input"
    annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant zer(
    final k=0)
    "Constant zero"
    annotation (Placement(transformation(extent={{60,10},{80,30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant locDp_min(
    final k=minLocDp)
    "Minimum local differential pressure"
    annotation (Placement(transformation(extent={{40,-70},{60,-50}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant one(
    final k=1)
    "Constant one"
    annotation (Placement(transformation(extent={{-120,10},{-100,30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant locDp_max(
    final k=maxLocDp)
    "Maximum local differential pressure "
    annotation (Placement(transformation(extent={{40,-130},{60,-110}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant pumSpe_min(
    final k=minPumSpe)
    "Minimum pump speed"
    annotation (Placement(transformation(extent={{40,90},{60,110}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant pumSpe_max(
    final k=maxPumSpe)
    "Maximum pump speed"
    annotation (Placement(transformation(extent={{-20,90},{0,110}})));

  Buildings.Controls.OBC.CDL.Logical.Not not2[nPum]
    "Logical not"
    annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));

  Buildings.Controls.OBC.CDL.Logical.MultiAnd mulAnd(
    final nu=nPum)
    "Multiple logical and"
    annotation (Placement(transformation(extent={{-120,-70},{-100,-50}})));

  Buildings.Controls.OBC.CDL.Continuous.Division div[nSen]
    "Normalized pressure difference"
    annotation (Placement(transformation(extent={{-40,-110},{-20,-90}})));

  Buildings.Controls.OBC.CDL.Continuous.Division div1
    "Normalized pressure difference"
    annotation (Placement(transformation(extent={{-100,70},{-80,90}})));

  Buildings.Controls.OBC.CDL.Routing.RealReplicator reaRep1(
    final nout=nSen)
    "Replicate real input"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));

  Buildings.Controls.OBC.CDL.Logical.Not pumOn
    "Check if there is any pump is ON"
    annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));

  Buildings.Controls.OBC.CDL.Logical.Switch swi
    "Logical switch"
    annotation (Placement(transformation(extent={{100,110},{120,130}})));

equation
  connect(conPID.y, maxRemDP.u)
    annotation (Line(points={{22,-20},{38,-20}}, color={0,0,127}));

  connect(booRep.y, conPID.trigger)
    annotation (Line(points={{-18,-60},{2,-60},{2,-32}}, color={255,0,255}));

  connect(dpChiWatSet, reaRep.u)
    annotation (Line(points={{-160,-120},{-122,-120}}, color={0,0,127}));

  connect(maxRemDP.y, locDpSet.u)
    annotation (Line(points={{62,-20},{98,-20}}, color={0,0,127}));

  connect(zer.y, locDpSet.x1)
    annotation (Line(points={{82,20},{90,20},{90,-12},{98,-12}}, color={0,0,127}));

  connect(locDp_min.y, locDpSet.f1)
    annotation (Line(points={{62,-60},{70,-60},{70,-16},{98,-16}},
      color={0,0,127}));

  connect(one.y, locDpSet.x2)
    annotation (Line(points={{-98,20},{40,20},{40,0},{80,0},{80,-24},{98,-24}},
      color={0,0,127}));

  connect(locDp_max.y, locDpSet.f2)
    annotation (Line(points={{62,-120},{80,-120},{80,-28},{98,-28}},
      color={0,0,127}));

  connect(zer.y, pumSpe.x1)
    annotation (Line(points={{82,20},{90,20},{90,68},{98,68}}, color={0,0,127}));

  connect(pumSpe_min.y, pumSpe.f1)
    annotation (Line(points={{62,100},{70,100},{70,64},{98,64}}, color={0,0,127}));

  connect(conPID1.y, pumSpe.u)
    annotation (Line(points={{-18,60},{98,60}}, color={0,0,127}));

  connect(one.y, pumSpe.x2)
    annotation (Line(points={{-98,20},{40,20},{40,56},{98,56}},color={0,0,127}));

  connect(pumSpe_max.y, pumSpe.f2)
    annotation (Line(points={{2,100},{20,100},{20,52},{98,52}}, color={0,0,127}));

  connect(uChiWatPum, not2.u)
    annotation (Line(points={{-160,-20},{-122,-20}},
      color={255,0,255}));

  connect(not2.y, mulAnd.u)
    annotation (Line(points={{-98,-20},{-80,-20},{-80,-40},{-130,-40},
      {-130,-60},{-122,-60}}, color={255,0,255}));

  connect(dpChiWat_remote, div.u1)
    annotation (Line(points={{-160,-90},{-80,-90},{-80,-94},{-42,-94}},
      color={0,0,127}));

  connect(reaRep.y, div.u2)
    annotation (Line(points={{-98,-120},{-80,-120},{-80,-106},{-42,-106}},
      color={0,0,127}));

  connect(locDpSet.y, div1.u2)
    annotation (Line(points={{122,-20},{130,-20},{130,40},{-120,40},{-120,74},
      {-102,74}}, color={0,0,127}));

  connect(dpChiWat_local, div1.u1)
    annotation (Line(points={{-160,100},{-120,100},{-120,86},{-102,86}},
      color={0,0,127}));

  connect(mulAnd.y, pumOn.u)
    annotation (Line(points={{-98,-60},{-82,-60}}, color={255,0,255}));

  connect(pumOn.y, booRep.u)
    annotation (Line(points={{-58,-60},{-42,-60}},
      color={255,0,255}));

  connect(pumOn.y, conPID1.trigger)
    annotation (Line(points={{-58,-60},{-50,-60},{-50,-20},{-38,-20},{-38,48}},
      color={255,0,255}));

  connect(one.y, reaRep1.u)
    annotation (Line(points={{-98,20},{-90,20},{-90,0},{-82,0}}, color={0,0,127}));

  connect(reaRep1.y, conPID.u_s)
    annotation (Line(points={{-58,0},{-20,0},{-20,-20},{-2,-20}}, color={0,0,127}));

  connect(div.y, conPID.u_m)
    annotation (Line(points={{-18,-100},{10,-100},{10,-32}}, color={0,0,127}));

  connect(one.y, conPID1.u_s)
    annotation (Line(points={{-98,20},{-90,20},{-90,60},{-42,60}}, color={0,0,127}));

  connect(div1.y, conPID1.u_m)
    annotation (Line(points={{-78,80},{-60,80},{-60,32},{-30,32},{-30,48}},
      color={0,0,127}));

  connect(pumSpe.y, swi.u1)
    annotation (Line(points={{122,60},{130,60},{130,100},{80,100},{80,128},{98,128}},
      color={0,0,127}));

  connect(pumOn.y, swi.u2)
    annotation (Line(points={{-58,-60},{-50,-60},{-50,120},{98,120}},
      color={255,0,255}));

  connect(zer.y, swi.u3)
    annotation (Line(points={{82,20},{90,20},{90,112},{98,112}},
      color={0,0,127}));

  connect(swi.y, yChiWatPumSpe)
    annotation (Line(points={{122,120},{160,120}}, color={0,0,127}));

annotation (
  defaultComponentName="chiPumSpe",
  Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
       graphics={
        Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,150},{100,110}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-98,52},{-44,30}},
          lineColor={255,0,255},
          pattern=LinePattern.Dash,
          textString="uChiWatPum"),
        Text(
          extent={{-98,-30},{-30,-52}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="dpChiWat_remote"),
        Text(
          extent={{22,12},{98,-10}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="yChiWatPumSpe"),
        Text(
          extent={{-98,-68},{-34,-90}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="dpChiWatSet"),
        Text(
          extent={{-98,92},{-30,70}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="dpChiWat_local")}),
  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})),
  Documentation(info="<html>
<p>
Block that control speed of enabled hot water pumps for primary-only plants where
the remote pressure differential (DP) sensor(s) is not hardwired to the plant controller,
but a local DP sensor is hardwired to the plant controller, 
according to ASHRAE RP-1711 (Draft 6 on July 25, 2019), 
section 5.2.6 Primary hot water pumps, part 5.2.6.9, 5.2.6.8.10 and 5.2.6.11.
</p>
<ol>
<li>
Remote DP shall be maintained at setpoint <code>dpChiWatSet</code> by a reverse
acting PID loop running in the controller to which the remote sensor is wired.
The loop output shall be a DP setpoint for the local primary loop DP sensor
hardwired to the plant controller. Reset local DP from <code>minLocDp</code>, 
e.g. 5 psi (34473.8 Pa), at 0% loop output to <code>maxLocDp</code> at 100%
loop output.
</li>
<li>
When any pump is proven on, pump speed shall be controlled by a reverse acting
PID loop maintaining the local primary DP signal at the DP setpoint output
from the remote sensor control loop. All pumps receive the same speed signal. 
PID loop output shall be mapped from minimum pump speed (<code>minPumSpe</code>) 
at 0% to maximum pump speed (<code>maxPumSpe</code>) at 100%.
</li>
<li>
Where multiple remote DP sensors exist, a PID loop shall run for each sensor.
The DP setpoint for the local DP sensor shall be the highest DP setpoint output
from each of the remote loops.
</li>
</ol>
</html>", revisions="<html>
<ul>
<li>
August 1, 2019, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"));

end Speed_primary_localDp;