within Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.BypassValve;
block BypassValvePosition
  "Sequence for controlling minimum flow bypass valve position"

  parameter Integer nPum = 2
    "Number of pumps";

  parameter Real k(
    final min=0,
    final unit="1",
    displayUnit="1") = 1
    "Gain of controller";

  parameter Real Ti(
    final min=0,
    final unit="s",
    displayUnit="s",
    final quantity="time") = 0.5
    "Time constant of integrator block";

  parameter Real Td(
    final min=0,
    final unit="s",
    displayUnit="s",
    final quantity="time") = 0.1
    "Time constant of derivative block";

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPumSta[nPum]
    "Input signals indicating pump statuses"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}}),
      iconTransformation(extent={{-140,-40},{-100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VHotWat_flow(
    final unit="m3/s",
    displayUnit="m3/s",
    final quantity="VolumeFlowRate")
    "Measured hot-water flow-rate through primary circuit"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}}),
      iconTransformation(extent={{-140,0},{-100,40}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VHotWatMinSet_flow(
    final unit="m3/s",
    displayUnit="m3/s",
    final quantity="VolumeFlowRate")
    "Calculated hot water minimum flow setpoint through boilers"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}}),
      iconTransformation(extent={{-140,40},{-100,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uMinBypValPos(
    final unit="1",
    displayUnit="1",
    final min=0)
    "Minimum bypass valve position for condensation control in non-condensing boilers"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
      iconTransformation(extent={{-140,-80},{-100,-40}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yBypValPos(
    final unit="1",
    displayUnit="1",
    final min=0)
    "Bypass valve opening position"
    annotation (Placement(transformation(extent={{100,-50},{140,-10}}),
      iconTransformation(extent={{100,-20},{140,20}})));

  Buildings.Controls.OBC.CDL.Logical.Switch swi
    "Check if bypass valve should be modulated"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant opeVal(
    final k=1)
    "Bypass valve fully open when pumps are off"
    annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));

  Buildings.Controls.OBC.CDL.Continuous.LimPID conPID(
    final controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PID,
    final k=k,
    final Ti=Ti,
    final Td=Td,
    final yMax=1,
    final yMin=0,
    final xi_start=1,
    final reset=Buildings.Controls.OBC.CDL.Types.Reset.Parameter)
    "Bypass circuit flow-rate controller to satisfy boiler minimum flow-rate"
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));

  Buildings.Controls.OBC.CDL.Logical.MultiOr mulOr(
    final nu=nPum)
    "Block to detect if any of the pumps are proved ON"
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Division div
    "Normalize measured hot water flowrate"
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con(
    final k=1)
    "Constant Real source"
    annotation (Placement(transformation(extent={{-50,70},{-30,90}})));

  Buildings.Controls.OBC.CDL.Continuous.Max max
    "Ensure bypass valve position is greater than lower limit for condensation control"
    annotation (Placement(transformation(extent={{70,-40},{90,-20}})));

  Buildings.Controls.OBC.CDL.Continuous.AddParameter addPar(
    final p=1e-6,
    final k=1)
    "Prevent division by zero"
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));

equation
  connect(opeVal.y, swi.u3) annotation (Line(points={{12,-20},{20,-20},{20,-8},{
          28,-8}}, color={0,0,127}));

  connect(conPID.y, swi.u1)
    annotation (Line(points={{12,60},{20,60},{20,8},{28,8}}, color={0,0,127}));

  connect(conPID.trigger, swi.u2)
    annotation (Line(points={{-6,48},{-6,0},{28,0}}, color={255,0,255}));

  connect(mulOr.u[1:nPum], uPumSta) annotation (Line(points={{-72,0},{-120,0}},
                                  color={255,0,255}));

  connect(mulOr.y, swi.u2) annotation (Line(points={{-48,0},{28,0}},
                   color={255,0,255}));

  connect(VHotWat_flow, div.u1)
    annotation (Line(points={{-120,40},{-70,40},{-70,46},{-42,46}},
                                                  color={0,0,127}));
  connect(div.y, conPID.u_m)
    annotation (Line(points={{-18,40},{0,40},{0,48}}, color={0,0,127}));
  connect(con.y, conPID.u_s) annotation (Line(points={{-28,80},{-20,80},{-20,60},
          {-12,60}}, color={0,0,127}));
  connect(swi.y, max.u1) annotation (Line(points={{52,0},{60,0},{60,-24},{68,-24}},
        color={0,0,127}));
  connect(uMinBypValPos, max.u2) annotation (Line(points={{-120,-60},{60,-60},{60,
          -36},{68,-36}}, color={0,0,127}));
  connect(max.y, yBypValPos)
    annotation (Line(points={{92,-30},{120,-30}}, color={0,0,127}));
  connect(VHotWatMinSet_flow, addPar.u)
    annotation (Line(points={{-120,80},{-92,80}}, color={0,0,127}));
  connect(addPar.y, div.u2) annotation (Line(points={{-68,80},{-60,80},{-60,34},
          {-42,34}}, color={0,0,127}));
  annotation (defaultComponentName="bypValPos",
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),
  graphics={Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-70,20},{70,-20}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.None,
              textString="bypValPos"),
            Text(
              extent={{-100,146},{100,108}},
              lineColor={0,0,255},
              textString="%name")}),
  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
Documentation(info="<html>
<p>
Control sequence for bypass circuit valve position <code>yBypValPos</code>
for boiler plant loop.
</p>
<ol>
<li>The bypass valve is enabled when any of the hot-water supply pumps are proven on
<code>uPumSta = true</code>, and disabled otherwise.</li>
<li>When enabled, a PID control loop modulates the bypass valve to maintain
a primary circuit flow rate of <code>VHotWatMinSet_flow</code>, calculated in class
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.SetPoints.MinimumFlowSetpoint\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.SetPoints.MinimumFlowSetpoint</a>.
</li>
<li>
When all the pumps are not proved on<code>uPumSta = false</code>, the valve is fully opened.
</li>
<li>
When a non-condensing boiler is enabled, the bypass valve position is set to the
higher value between the signal generated by the PID loop, and the minimum valve
position for condensation control <code>uMinBypValPos</code>.
</li>
</ol>
</html>", revisions="<html>
<ul>
<li>
August 17, 2020, by Karthik Devaprasad:<br/>
First implementation.
</li>
</ul>
</html>"),
    experiment(
      StartTime=-1814400,
      StopTime=1814400,
      Interval=1,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end BypassValvePosition;
