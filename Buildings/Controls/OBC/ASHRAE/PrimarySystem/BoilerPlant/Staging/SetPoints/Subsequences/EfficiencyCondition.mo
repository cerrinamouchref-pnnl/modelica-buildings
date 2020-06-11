within Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences;
block EfficiencyCondition
  "Efficiency condition used in staging up and down"

  parameter Integer nSta = 5
    "Number of stages in the boiler plant";

  parameter Real fraNonConBoi(
    final min=0,
    final unit="1",
    final displayUnit="1") = 0.9
    "Fraction of design capacity of current stage at which the efficiency condition
    is satisfied for non-condensing boilers";

  parameter Real fraConBoi(
    final min=0,
    final unit="1",
    final displayUnit="1") = 1.5
    "Fraction of B-Stage minimum of next higher stage at which the efficiency
    condition is satisfied for condensing boilers";

  parameter Real sigDif(
    final min=0,
    final unit="1",
    final max=1,
    final displayUnit="1") = 0.1
    "Signal hysteresis deadband"
    annotation (Dialog(tab="Advanced"));

  parameter Real delCapReq(
    final unit="s",
    final displayUnit="s",
    final quantity="Time") = 600
    "Enable delay for heating requirement condition";

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uTyp[nSta]
    "Vector of boiler types for all stages"
    annotation (Placement(transformation(extent={{-160,-120},{-120,-80}}),
      iconTransformation(extent={{-140,-80},{-100,-40}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uAvaUp
    "Stage number of next available higher stage"
    annotation (Placement(transformation(extent={{-160,-160},{-120,-120}}),
      iconTransformation(extent={{-140,-110},{-100,-70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCapReq(
    final unit="W",
    final displayUnit="W",
    final quantity="Power")
    "Heating capacity required"
    annotation (Placement(transformation(extent={{-160,40},{-120,80}}),
      iconTransformation(extent={{-140,70},{-100,110}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCapDes(
    final unit="W",
    final displayUnit="W",
    final quantity="Power")
    "Design heating capacity of current stage"
    annotation (Placement(transformation(extent={{-160,80},{-120,120}}),
      iconTransformation(extent={{-140,40},{-100,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCapUpMin(
    final unit="W",
    final displayUnit="W",
    final quantity="Power")
    "Minimum capacity of the next available higher stage"
    annotation (Placement(transformation(extent={{-160,0},{-120,40}}),
      iconTransformation(extent={{-140,10},{-100,50}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VHotWat_flow(
    final unit="m3/s",
    final displayUnit="m3/s",
    final quantity="VolumeFlowRate")
    "Measured hot-water flow-rate"
    annotation (Placement(transformation(extent={{-160,-40},{-120,0}}),
      iconTransformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VUpMinSet_flow(
    final unit="m3/s",
    final displayUnit="m3/s",
    final quantity="VolumeFlowRate")
    "Minimum flow setpoint for the next available higher stage"
    annotation (Placement(transformation(extent={{-160,-80},{-120,-40}}),
      iconTransformation(extent={{-140,-50},{-100,-10}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yEffCon
    "Efficiency condition for boiler staging"
    annotation (Placement(transformation(extent={{120,-20},{160,20}}),
      iconTransformation(extent={{100,-20},{140,20}})));

protected
  Buildings.Controls.OBC.CDL.Continuous.Division div
    "Divider to get relative value of required heating capacity"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));

  Buildings.Controls.OBC.CDL.Continuous.Division div1
    "Divider to get relative value of required heating capacity"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

  Buildings.Controls.OBC.CDL.Continuous.Division div2
    "Divider to get relative value of flow-rate"
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys(
    final uLow=-sigDif,
    final uHigh=0)
    "Hysteresis loop for flow-rate condition"
    annotation (Placement(transformation(extent={{0,-60},{20,-40}})));

  Buildings.Controls.OBC.CDL.Logical.Sources.Constant con1(
    final k=true)
    "Constant boolean source"
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys1(final uLow=fraNonConBoi -
        sigDif, final uHigh=fraNonConBoi)
    "Hysteresis loop for heating capacity condition of non-condensing boilers"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys2(final uLow=fraConBoi -
        sigDif, final uHigh=fraConBoi)
    "Hysteresis loop for heating capacity condition of condensing boilers"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel(
    final delayTime=delCapReq,
    final delayOnInit=true)
    "Enable delay"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel1(
    final delayTime=delCapReq,
    final delayOnInit=true)
    "Enable delay"
    annotation (Placement(transformation(extent={{0,60},{20,80}})));

  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea[nSta]
    "Convert integers in stage-type vector to real data-type"
    annotation (Placement(transformation(extent={{-80,-110},{-60,-90}})));

  Buildings.Controls.OBC.CDL.Routing.RealExtractor extIndSig(
    final nin=nSta)
    "Pick out stage-type for next stage from vector"
    annotation (Placement(transformation(extent={{-40,-110},{-20,-90}})));

  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold greThr(
    final threshold=1)
    "Check for non-condensing boilers"
    annotation (Placement(transformation(extent={{0,-110},{20,-90}})));

  Buildings.Controls.OBC.CDL.Logical.LogicalSwitch logSwi
    "Switch for heating capacity condition based on stage type"
    annotation (Placement(transformation(extent={{40,40},{60,60}})));

  Buildings.Controls.OBC.CDL.Logical.And and2
    "Logical And"
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  Buildings.Controls.OBC.CDL.Logical.LogicalSwitch logSwi1
    "Switch for flow-rate condition"
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));

  Buildings.Controls.OBC.CDL.Continuous.Add add1(
    final k2=-1)
    "Adder"
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));

equation
  connect(div.u2,uCapUpMin)
    annotation (Line(points={{-82,24},{-110,24},{-110,20},{-140,20}},
      color={0,0,127}));
  connect(div1.u2, uCapDes) annotation (Line(points={{-82,64},{-90,64},{-90,70},
          {-114,70},{-114,100},{-140,100}}, color={0,0,127}));
  connect(add1.u1, VHotWat_flow) annotation (Line(points={{-82,-34},{-110,-34},{
          -110,-20},{-140,-20}},  color={0,0,127}));
  connect(add1.u2, VUpMinSet_flow) annotation (Line(points={{-82,-46},{-110,-46},
          {-110,-60},{-140,-60}}, color={0,0,127}));
  connect(add1.y, div2.u1)
    annotation (Line(points={{-58,-40},{-50,-40},{-50,-44},{-42,-44}},
      color={0,0,127}));
  connect(div2.u2, VUpMinSet_flow) annotation (Line(points={{-42,-56},{-50,-56},
          {-50,-60},{-140,-60}}, color={0,0,127}));
  connect(div2.y, hys.u)
    annotation (Line(points={{-18,-50},{-2,-50}},
      color={0,0,127}));
  connect(div1.y, hys1.u)
    annotation (Line(points={{-58,70},{-42,70}},
      color={0,0,127}));
  connect(div.y, hys2.u)
    annotation (Line(points={{-58,30},{-42,30}},
      color={0,0,127}));
  connect(truDel1.u, hys1.y)
    annotation (Line(points={{-2,70},{-18,70}},
      color={255,0,255}));
  connect(truDel.u, hys2.y)
    annotation (Line(points={{-2,30},{-18,30}},
      color={255,0,255}));
  connect(intToRea.u, uTyp)
    annotation (Line(points={{-82,-100},{-140,-100}},
      color={255,127,0}));
  connect(extIndSig.u, intToRea.y)
    annotation (Line(points={{-42,-100},{-58,-100}},
      color={0,0,127}));
  connect(extIndSig.index, uAvaUp)
    annotation (Line(points={{-30,-112},{-30,-140},{-140,-140}},
      color={255,127,0}));
  connect(greThr.u, extIndSig.y)
    annotation (Line(points={{-2,-100},{-18,-100}},
      color={0,0,127}));
  connect(greThr.y, logSwi.u2)
    annotation (Line(points={{22,-100},{30,-100},{30,50},{38,50}},
      color={255,0,255}));
  connect(truDel.y, logSwi.u3)
    annotation (Line(points={{22,30},{36,30},{36,42},{38,42}},
      color={255,0,255}));
  connect(truDel1.y, logSwi.u1)
    annotation (Line(points={{22,70},{36,70},{36,58},{38,58}},
      color={255,0,255}));
  connect(and2.y, yEffCon)
    annotation (Line(points={{102,0},{140,0}},
      color={255,0,255}));
  connect(logSwi.y, and2.u1)
    annotation (Line(points={{62,50},{70,50},{70,0},{78,0}},
      color={255,0,255}));
  connect(logSwi1.y, and2.u2)
    annotation (Line(points={{62,-30},{70,-30},{70,-8},{78,-8}},
      color={255,0,255}));
  connect(con1.y, logSwi1.u1)
    annotation (Line(points={{22,-10},{36,-10},{36,-22},{38,-22}},
      color={255,0,255}));
  connect(hys.y, logSwi1.u3)
    annotation (Line(points={{22,-50},{36,-50},{36,-38},{38,-38}},
      color={255,0,255}));
  connect(logSwi1.u2, greThr.y)
    annotation (Line(points={{38,-30},{30,-30},{30,-100},{22,-100}},
      color={255,0,255}));
  connect(div.u1, uCapReq) annotation (Line(points={{-82,36},{-100,36},{-100,60},
          {-140,60}}, color={0,0,127}));
  connect(div1.u1, uCapReq) annotation (Line(points={{-82,76},{-100,76},{-100,60},
          {-140,60}}, color={0,0,127}));

annotation (
  defaultComponentName = "effCon",
  Icon(
    coordinateSystem(extent={{-100,-100},{100,100}}),
    graphics={Rectangle(
                extent={{-100,-100},{100,100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-120,146},{100,108}},
                lineColor={0,0,255},
                textString="%name")}),
  Diagram(
    coordinateSystem(
      preserveAspectRatio=false,
      extent={{-120,-160},{120,120}})),
  Documentation(
    info="<html>
    <p>
    Efficiency condition used in staging up and down for boiler plants with both
    condensing and non-condensing boilers. Implemented according to the
    specification provided in 5.3.3.10, subsections 6.b, 8.b, 10.b and 12.b in
    RP-1711, March 2020 Draft.
    </p>
    <p align=\"center\">
    <img alt=\"State-machine chart for EfficiencyCondition\"
    src=\"modelica://Buildings/Resources/Images/Controls/OBC/ASHRAE/PrimarySystem/BoilerPlant/Staging/SetPoints/Subsequences/EfficiencyCondition_stateMachineChart_v2.png\"/>
    <br/>
    State-machine chart for the sequence defined in RP-1711
    </p>
    <p align=\"center\">
    <img alt=\"Validation plot for EfficiencyCondition1\"
    src=\"modelica://Buildings/Resources/Images/Controls/OBC/ASHRAE/PrimarySystem/BoilerPlant/Staging/SetPoints/Subsequences/EfficiencyCondition1.png\"/>
    <br/>
    Validation plot generated from model <a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Validation.EfficiencyCondition\">
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Validation.EfficiencyCondition</a> with the next higher stage type as condensing.
    </p>
    <p align=\"center\">
    <img alt=\"Validation plot for EfficiencyCondition2\"
    src=\"modelica://Buildings/Resources/Images/Controls/OBC/ASHRAE/PrimarySystem/BoilerPlant/Staging/SetPoints/Subsequences/EfficiencyCondition2.png\"/>
    <br/>
    Validation plot generated from model <a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Validation.EfficiencyCondition\">
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Validation.EfficiencyCondition</a> with the next higher stage type as non-condensing.
    </p>
    </html>",
    revisions="<html>
    <ul>
    <li>
    May 21, 2020, by Karthik Devaprasad:<br/>
    First implementation.
    </li>
    </ul>
    </html>"));
end EfficiencyCondition;
