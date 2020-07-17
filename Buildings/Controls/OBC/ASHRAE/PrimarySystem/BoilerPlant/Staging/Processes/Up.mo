﻿within Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes;
block Up
    "Sequence for control devices when there is stage-up command"

  parameter Boolean primaryOnly = false
    "True: The boiler plant is primary-only";

  parameter Boolean isHeadered = true
    "True: Headered hot water pumps; False: Dedicated hot water pumps";

  parameter Integer nBoi=3
    "Total number of boilers in the plant";

  parameter Integer nSta=5
    "Total number of stages";

  parameter Real delBoiEna(
    final unit="s",
    final displayUnit="s",
    final quantity="time") = 180
    "Time delay after boiler change process has been completed before turning off excess valves and pumps";

  parameter Real delEnaMinFloSet(
    final unit="s",
    final displayUnit="s",
    final quantity="time")=60
    "Enable delay after minimum flow setpoint is achieved in bypass valve";

  parameter Real delProSupTemSet(
    final unit="s",
    final displayUnit="s",
    final quantity="time")=300
    "Process time-out for hot water supply temperature setpoint reset";

  parameter Real TMinSupNonConBoi(
    final unit="K",
    final displayUnit="K",
    final quantity="ThermodynamicTemperature") = 333.2
    "Minimum supply temperature required for non-condensing boilers";

  parameter Real sigDif(
    final unit="K",
    final displayUnit="K",
    final quantity="TemperatureDifference")=0.1
    "Significant difference based on minimum resolution of temperature sensor";

  parameter Real chaIsoValTim(
    final unit="s",
    final displayUnit="s",
    final quantity="time") = 60
    "Time to slowly change isolation valve, should be determined in the field";

  parameter Real boiChaProOnTim(
    final unit="s",
    final displayUnit="s",
    final quantity="time") = 300
    "Enabled boiler operation time to indicate if it is proven on during a staging
    process where one boiler is turned on and the other is turned off";

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uStaUpPro
    "Pulse indicating start of stage up process"
    annotation (Placement(transformation(extent={{-280,-20},{-240,20}}),
      iconTransformation(extent={{-140,-80},{-100,-40}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uBoi[nBoi]
    "Boiler status: true=ON"
    annotation (Placement(transformation(extent={{-280,60},{-240,100}}),
      iconTransformation(extent={{-140,0},{-100,40}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPumChaPro
    "Pulse indicating all pump change processes have been completed and pumps have been proved on"
    annotation (Placement(transformation(extent={{-280,-210},{-240,-170}}),
      iconTransformation(extent={{-140,-200},{-100,-160}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uBoiSet[nBoi]
    "Boiler status setpoint: true=ON"
    annotation (Placement(transformation(extent={{-280,20},{-240,60}}),
      iconTransformation(extent={{-140,-40},{-100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uStaTyp[nSta]
    "Boiler plant stage type vector"
    annotation (Placement(transformation(extent={{-280,-60},{-240,-20}}),
      iconTransformation(extent={{-140,-120},{-100,-80}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uStaSet
    "Boiler stage setpoint index"
    annotation (Placement(transformation(extent={{-280,-100},{-240,-60}}),
      iconTransformation(extent={{-140,-160},{-100,-120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VMinHotWatSet_flow(
    final quantity="VolumeFlowRate",
    final unit="m3/s",
    final displayUnit="m3/s",
    final min=0) if primaryOnly
    "Minimum hot water flow rate setpoint"
    annotation (Placement(transformation(extent={{-280,180},{-240,220}}),
      iconTransformation(extent={{-140,120},{-100,160}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VHotWat_flow(
    final min=0,
    final quantity="VolumeFlowRate",
    final unit="m3/s",
    final displayUnit="m3/s") if primaryOnly
    "Measured hot water flow rate through the minimum flow bypass valve"
    annotation (Placement(transformation(extent={{-280,220},{-240,260}}),
      iconTransformation(extent={{-140,160},{-100,200}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput THotWatSup(
    final unit="K",
    final displayUnit="K",
    final quantity="ThermodynamicTemperature") if not primaryOnly
    "Measured hot water supply temperature"
    annotation (Placement(transformation(extent={{-280,140},{-240,180}}),
      iconTransformation(extent={{-140,80},{-100,120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uHotWatIsoVal[nBoi](
    final min=fill(0, nBoi),
    final max=fill(1, nBoi),
    final unit=fill("1", nBoi)) if isHeadered
    "Hot water isolation valve position"
    annotation (Placement(transformation(extent={{-280,100},{-240,140}}),
      iconTransformation(extent={{-140,40},{-100,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yBoi[nBoi]
    "Boiler enabling status"
    annotation (Placement(transformation(extent={{240,90},{280,130}}),
      iconTransformation(extent={{100,160},{140,200}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yStaChaPro
    "Pulse indicating end of stage change process"
    annotation (Placement(transformation(extent={{240,30},{280,70}}),
      iconTransformation(extent={{100,90},{140,130}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yPumChaPro
    "Pulse indicating start of pump change process"
    annotation (Placement(transformation(extent={{240,-260},{280,-220}}),
      iconTransformation(extent={{100,-190},{140,-150}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yOnOff
    "Signal indicating whether stage change involves simultaneous turning on
    and turning off of boilers"
    annotation (Placement(transformation(extent={{240,-130},{280,-90}}),
      iconTransformation(extent={{100,-50},{140,-10}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput yNexEnaBoi
    "Boiler index of next boiler being enabled"
    annotation (Placement(transformation(extent={{240,-170},{280,-130}}),
      iconTransformation(extent={{100,-120},{140,-80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yHotWatIsoVal[nBoi](
    final min=fill(0, nBoi),
    final max=fill(1, nBoi),
    final unit=fill("1", nBoi)) if isHeadered
    "Boiler hot water isolation valve position"
    annotation (Placement(transformation(extent={{240,-90},{280,-50}}),
      iconTransformation(extent={{100,20},{140,60}})));

  Subsequences.ResetMinBypass minBypRes(final delEna=delEnaMinFloSet, relFloDif=
       relFloDif) if                                                     primaryOnly
    "Reset process for minimum flow bypass valve setpoint"
    annotation (Placement(transformation(extent={{-170,10},{-150,30}})));

  Subsequences.ResetHotWaterSupplyTemperature hotWatSupTemRes(final nSta=nSta,
    final delPro=delProSupTemSet,
    final TMinSupNonConBoi=TMinSupNonConBoi,
    final sigDif=sigDif) if not primaryOnly
    "Reset process for hot water supply temperature setpoint"
    annotation (Placement(transformation(extent={{-170,-30},{-150,-10}})));

  Subsequences.NextBoiler nexBoi(final nBoi=nBoi)
    "Identify boiler indices to  be turned on and off during the stage change process"
    annotation (Placement(transformation(extent={{-170,-70},{-150,-50}})));

  Subsequences.HWIsoVal enaHotWatIsoVal(
    final nBoi=nBoi,
    final chaHotWatIsoTim=chaIsoValTim,
    final iniValPos=0,
    final endValPos=1) if isHeadered
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));

  Buildings.Controls.OBC.CDL.Logical.And and1
    annotation (Placement(transformation(extent={{0,-8},{20,12}})));

  Subsequences.EnableBoiler enaBoi(
    final nBoi=nBoi,
    final proOnTim=boiChaProOnTim)
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));

  Subsequences.HWIsoVal disHotWatIsoVal1(
    final nBoi=nBoi,
    final chaHotWatIsoTim=chaIsoValTim,
    final iniValPos=1,
    final endValPos=0) if isHeadered
    annotation (Placement(transformation(extent={{150,-10},{170,10}})));

  Buildings.Controls.OBC.CDL.Logical.And and3
    annotation (Placement(transformation(extent={{190,-10},{210,10}})));

  Buildings.Controls.OBC.CDL.Logical.Latch lat
    "Latch to retain stage-up edge signal till the stage change process is completed"
    annotation (Placement(transformation(extent={{-222,-10},{-202,10}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel(final delayTime=delBoiEna)
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  Buildings.Controls.OBC.CDL.Logical.Pre pre
    annotation (Placement(transformation(extent={{200,-60},{220,-40}})));

  Buildings.Controls.OBC.CDL.Logical.Latch lat1 if isHeadered
    annotation (Placement(transformation(extent={{-100,-120},{-80,-100}})));

  Buildings.Controls.OBC.CDL.Logical.Pre pre1 if isHeadered
    annotation (Placement(transformation(extent={{-140,-126},{-120,-106}})));

  Buildings.Controls.OBC.CDL.Logical.Latch lat2 if isHeadered
    annotation (Placement(transformation(extent={{-30,-120},{-10,-100}})));

  Buildings.Controls.OBC.CDL.Logical.Latch lat3
    annotation (Placement(transformation(extent={{-140,-200},{-120,-180}})));

  Buildings.Controls.OBC.CDL.Logical.Latch lat4
    annotation (Placement(transformation(extent={{160,-200},{180,-180}})));

  Buildings.Controls.OBC.CDL.Logical.And and4
    annotation (Placement(transformation(extent={{132,-200},{152,-180}})));

  Buildings.Controls.OBC.CDL.Logical.Edge edg
    annotation (Placement(transformation(extent={{210,40},{230,60}})));

  Buildings.Controls.OBC.CDL.Logical.Edge edg1
    annotation (Placement(transformation(extent={{-100,-260},{-80,-240}})));

  Buildings.Controls.OBC.CDL.Logical.Edge edg2
    annotation (Placement(transformation(extent={{140,-246},{160,-226}})));

  Buildings.Controls.OBC.CDL.Logical.Or or2
    annotation (Placement(transformation(extent={{210,-250},{230,-230}})));

  parameter Real relFloDif=0.05
    "Relative error to the flow setpoint for checking if it has been achieved";
  CDL.Logical.Latch                        lat5 if not isHeadered
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  CDL.Logical.Latch                        lat6 if not isHeadered
    annotation (Placement(transformation(extent={{150,30},{170,50}})));
  CDL.Logical.Switch swi[nBoi]
    annotation (Placement(transformation(extent={{40,-140},{60,-120}})));
  CDL.Routing.BooleanReplicator booRep(nout=nBoi)
    annotation (Placement(transformation(extent={{0,-140},{20,-120}})));
  CDL.Logical.Switch swi1[nBoi]
    annotation (Placement(transformation(extent={{100,-120},{120,-100}})));
  CDL.Routing.BooleanReplicator booRep1(nout=nBoi)
    annotation (Placement(transformation(extent={{70,-160},{90,-140}})));
  CDL.Logical.Not not1
    annotation (Placement(transformation(extent={{140,60},{160,80}})));
  CDL.Logical.Or or1
    annotation (Placement(transformation(extent={{180,60},{200,80}})));
equation

  connect(and1.y, enaBoi.uUpsDevSta)
    annotation (Line(points={{22,2},{58,2}},               color={255,0,255}));

  connect(nexBoi.yNexEnaBoi, enaHotWatIsoVal.nexChaBoi) annotation (Line(points={{-148,
          -51},{-90,-51},{-90,8},{-72,8}},       color={255,127,0}));

  connect(lat.y, minBypRes.uUpsDevSta) annotation (Line(points={{-200,0},{-196,
          0},{-196,28},{-172,28}},
                                color={255,0,255}));

  connect(lat.y, minBypRes.chaPro) annotation (Line(points={{-200,0},{-196,0},{
          -196,24},{-172,24}},
                          color={255,0,255}));

  connect(lat.y, hotWatSupTemRes.uStaUp) annotation (Line(points={{-200,0},{
          -196,0},{-196,-13},{-172,-13}},
                                     color={255,0,255}));

  connect(lat.y, nexBoi.chaPro) annotation (Line(points={{-200,0},{-196,0},{
          -196,-67},{-172,-67}},
                            color={255,0,255}));

  connect(enaBoi.yBoiEnaPro, truDel.u) annotation (Line(points={{82,-8},{90,-8},
          {90,0},{98,0}}, color={255,0,255}));

  connect(truDel.y,disHotWatIsoVal1. uUpsDevSta) annotation (Line(points={{122,0},
          {126,0},{126,-5},{148,-5}}, color={255,0,255}));

  connect(nexBoi.yDisSmaBoi,disHotWatIsoVal1. nexChaBoi) annotation (Line(
        points={{-148,-56},{140,-56},{140,8},{148,8}}, color={255,127,0}));

  connect(nexBoi.yOnOff,disHotWatIsoVal1. chaPro) annotation (Line(points={{-148,
          -60},{144,-60},{144,-8},{148,-8}}, color={255,0,255}));

  connect(and3.y, pre.u) annotation (Line(points={{212,0},{220,0},{220,-30},{
          190,-30},{190,-50},{198,-50}},
                                     color={255,0,255}));

  connect(pre.y, lat.clr) annotation (Line(points={{222,-50},{230,-50},{230,-80},
          {-230,-80},{-230,-6},{-224,-6}}, color={255,0,255}));

  connect(lat.y, enaHotWatIsoVal.chaPro) annotation (Line(points={{-200,0},{-196,
          0},{-196,-34},{-80,-34},{-80,-8},{-72,-8}}, color={255,0,255}));

  connect(lat.y, enaBoi.uStaUp) annotation (Line(points={{-200,0},{-196,0},{
          -196,-34},{52,-34},{52,6},{58,6}},
                                        color={255,0,255}));

  connect(nexBoi.yOnOff, enaBoi.uOnOff) annotation (Line(points={{-148,-60},{42,
          -60},{42,-6},{58,-6}}, color={255,0,255}));

  connect(nexBoi.yDisSmaBoi, enaBoi.nexDisBoi) annotation (Line(points={{-148,
          -56},{46,-56},{46,-9},{58,-9}},
                                     color={255,127,0}));

  connect(nexBoi.yNexEnaBoi, enaBoi.nexEnaBoi) annotation (Line(points={{-148,
          -51},{-90,-51},{-90,-40},{38,-40},{38,9},{58,9}},
                                                       color={255,127,0}));

  connect(VHotWat_flow, minBypRes.VHotWat_flow) annotation (Line(points={{-260,
          240},{-176,240},{-176,16},{-172,16}},
                                           color={0,0,127}));

  connect(VMinHotWatSet_flow, minBypRes.VMinHotWatSet_flow) annotation (Line(
        points={{-260,200},{-180,200},{-180,12},{-172,12}}, color={0,0,127}));

  connect(THotWatSup, hotWatSupTemRes.THotWatSup) annotation (Line(points={{
          -260,160},{-186,160},{-186,-17},{-172,-17}}, color={0,0,127}));

  connect(uHotWatIsoVal, enaHotWatIsoVal.uHotWatIsoVal) annotation (Line(points={{-260,
          120},{-80,120},{-80,5},{-72,5}},       color={0,0,127}));

  connect(lat1.y, enaHotWatIsoVal.uUpsDevSta) annotation (Line(points={{-78,
          -110},{-74,-110},{-74,-5},{-72,-5}}, color={255,0,255}));

  connect(pre1.y, lat1.clr)
    annotation (Line(points={{-118,-116},{-102,-116}}, color={255,0,255}));

  connect(pre1.u, enaHotWatIsoVal.yEnaHotWatIsoVal) annotation (Line(points={{
          -142,-116},{-146,-116},{-146,-130},{-44,-130},{-44,6},{-48,6}}, color=
         {255,0,255}));

  connect(enaHotWatIsoVal.yEnaHotWatIsoVal, lat2.u) annotation (Line(points={{
          -48,6},{-44,6},{-44,-110},{-32,-110}}, color={255,0,255}));

  connect(pre.y, lat2.clr) annotation (Line(points={{222,-50},{230,-50},{230,
          -80},{-40,-80},{-40,-116},{-32,-116}}, color={255,0,255}));

  connect(lat2.y, and1.u1) annotation (Line(points={{-8,-110},{-6,-110},{-6,2},
          {-2,2}}, color={255,0,255}));

  connect(uHotWatIsoVal, disHotWatIsoVal1.uHotWatIsoVal) annotation (Line(
        points={{-260,120},{128,120},{128,5},{148,5}}, color={0,0,127}));

  connect(nexBoi.uBoiSet, uBoiSet) annotation (Line(points={{-172,-63},{-190,
          -63},{-190,40},{-260,40}}, color={255,0,255}));

  connect(uBoi, enaBoi.uBoi) annotation (Line(points={{-260,80},{46,80},{46,-2},
          {58,-2}}, color={255,0,255}));

  connect(uStaSet, nexBoi.uStaSet) annotation (Line(points={{-260,-80},{-234,
          -80},{-234,-53},{-172,-53}}, color={255,127,0}));

  connect(uStaSet, hotWatSupTemRes.uStaSet) annotation (Line(points={{-260,-80},
          {-234,-80},{-234,-27},{-172,-27}}, color={255,127,0}));

  connect(uStaTyp, hotWatSupTemRes.uStaTyp) annotation (Line(points={{-260,-40},
          {-186,-40},{-186,-23},{-172,-23}}, color={255,127,0}));

  connect(uStaUpPro, lat.u)
    annotation (Line(points={{-260,0},{-224,0}}, color={255,0,255}));

  connect(lat3.y, and1.u2) annotation (Line(points={{-118,-190},{-48,-190},{-48,
          -30},{-10,-30},{-10,-6},{-2,-6}}, color={255,0,255}));

  connect(uPumChaPro, lat3.u)
    annotation (Line(points={{-260,-190},{-142,-190}}, color={255,0,255}));

  connect(and4.y, lat4.u)
    annotation (Line(points={{154,-190},{158,-190}}, color={255,0,255}));

  connect(truDel.y, and4.u1) annotation (Line(points={{122,0},{126,0},{126,-190},
          {130,-190}}, color={255,0,255}));

  connect(uPumChaPro, and4.u2) annotation (Line(points={{-260,-190},{-160,-190},
          {-160,-210},{120,-210},{120,-198},{130,-198}}, color={255,0,255}));

  connect(lat4.y, and3.u2) annotation (Line(points={{182,-190},{184,-190},{184,
          -8},{188,-8}}, color={255,0,255}));

  connect(pre.y, lat3.clr) annotation (Line(points={{222,-50},{230,-50},{230,
          -216},{-150,-216},{-150,-196},{-142,-196}}, color={255,0,255}));

  connect(pre.y, lat4.clr) annotation (Line(points={{222,-50},{230,-50},{230,
          -216},{154,-216},{154,-196},{158,-196}}, color={255,0,255}));

  connect(and3.y, edg.u) annotation (Line(points={{212,0},{220,0},{220,30},{200,
          30},{200,50},{208,50}}, color={255,0,255}));

  connect(edg.y, yStaChaPro)
    annotation (Line(points={{232,50},{260,50}}, color={255,0,255}));

  connect(edg2.y, or2.u1) annotation (Line(points={{162,-236},{200,-236},{200,
          -240},{208,-240}}, color={255,0,255}));

  connect(edg1.y, or2.u2) annotation (Line(points={{-78,-250},{66,-250},{66,
          -248},{208,-248}}, color={255,0,255}));

  connect(enaBoi.yBoi, yBoi) annotation (Line(points={{82,8},{90,8},{90,110},{
          260,110}}, color={255,0,255}));

  connect(or2.y, yPumChaPro)
    annotation (Line(points={{232,-240},{260,-240}}, color={255,0,255}));

  connect(nexBoi.yOnOff, yOnOff) annotation (Line(points={{-148,-60},{144,-60},
          {144,-110},{260,-110}}, color={255,0,255}));

  connect(nexBoi.yNexEnaBoi, yNexEnaBoi) annotation (Line(points={{-148,-51},{
          160,-51},{160,-150},{260,-150}}, color={255,127,0}));

  connect(truDel.y, edg2.u) annotation (Line(points={{122,0},{126,0},{126,-236},
          {138,-236}}, color={255,0,255}));

  connect(minBypRes.yMinBypRes, lat1.u) annotation (Line(points={{-148,20},{-110,
          20},{-110,-110},{-102,-110}}, color={255,0,255}));
  connect(hotWatSupTemRes.yHotWatSupTemRes, lat1.u) annotation (Line(points={{-148,
          -20},{-110,-20},{-110,-110},{-102,-110}}, color={255,0,255}));
  connect(minBypRes.yMinBypRes, edg1.u) annotation (Line(points={{-148,20},{-110,
          20},{-110,-250},{-102,-250}}, color={255,0,255}));
  connect(hotWatSupTemRes.yHotWatSupTemRes, edg1.u) annotation (Line(points={{-148,
          -20},{-110,-20},{-110,-250},{-102,-250}}, color={255,0,255}));
  connect(minBypRes.yMinBypRes, lat5.u) annotation (Line(points={{-148,20},{
          -110,20},{-110,40},{-72,40}},
                                   color={255,0,255}));
  connect(hotWatSupTemRes.yHotWatSupTemRes, lat5.u) annotation (Line(points={{-148,
          -20},{-110,-20},{-110,40},{-72,40}}, color={255,0,255}));
  connect(pre.y, lat5.clr) annotation (Line(points={{222,-50},{230,-50},{230,
          -80},{-100,-80},{-100,34},{-72,34}},
                                          color={255,0,255}));
  connect(lat5.y, and1.u1) annotation (Line(points={{-48,40},{-6,40},{-6,2},{-2,
          2}}, color={255,0,255}));
  connect(truDel.y, lat6.u) annotation (Line(points={{122,0},{126,0},{126,40},{148,
          40}}, color={255,0,255}));
  connect(lat6.y, and3.u1) annotation (Line(points={{172,40},{180,40},{180,0},{188,
          0}}, color={255,0,255}));
  connect(pre.y, lat6.clr) annotation (Line(points={{222,-50},{230,-50},{230,
          -80},{136,-80},{136,34},{148,34}},
                                        color={255,0,255}));
  connect(enaHotWatIsoVal.yHotWatIsoVal, swi.u1) annotation (Line(points={{-48,
          -6},{-30,-6},{-30,-90},{26,-90},{26,-122},{38,-122}}, color={0,0,127}));
  connect(uHotWatIsoVal, swi.u3) annotation (Line(points={{-260,120},{30,120},{
          30,-138},{38,-138}}, color={0,0,127}));
  connect(booRep.y, swi.u2)
    annotation (Line(points={{22,-130},{38,-130}}, color={255,0,255}));
  connect(lat1.y, booRep.u) annotation (Line(points={{-78,-110},{-74,-110},{-74,
          -124},{-20,-124},{-20,-130},{-2,-130}}, color={255,0,255}));
  connect(swi.y, swi1.u3) annotation (Line(points={{62,-130},{92,-130},{92,-118},
          {98,-118}}, color={0,0,127}));
  connect(disHotWatIsoVal1.yHotWatIsoVal, swi1.u1) annotation (Line(points={{
          172,-6},{180,-6},{180,-70},{92,-70},{92,-102},{98,-102}}, color={0,0,
          127}));
  connect(truDel.y, booRep1.u) annotation (Line(points={{122,0},{126,0},{126,
          -170},{60,-170},{60,-150},{68,-150}}, color={255,0,255}));
  connect(booRep1.y, swi1.u2) annotation (Line(points={{92,-150},{96,-150},{96,
          -110},{98,-110}}, color={255,0,255}));
  connect(swi1.y, yHotWatIsoVal) annotation (Line(points={{122,-110},{134,-110},
          {134,-90},{210,-90},{210,-70},{260,-70}}, color={0,0,127}));
  connect(nexBoi.yOnOff, not1.u) annotation (Line(points={{-148,-60},{132,-60},
          {132,70},{138,70}}, color={255,0,255}));
  connect(not1.y, or1.u1)
    annotation (Line(points={{162,70},{178,70}}, color={255,0,255}));
  connect(disHotWatIsoVal1.yEnaHotWatIsoVal, or1.u2) annotation (Line(points={{
          172,6},{176,6},{176,62},{178,62}}, color={255,0,255}));
  connect(or1.y, and3.u1) annotation (Line(points={{202,70},{206,70},{206,54},{
          180,54},{180,0},{188,0}}, color={255,0,255}));
annotation (
  defaultComponentName="upProCon",
  Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-240,-260},{240,260}})),
    Icon(coordinateSystem(extent={{-100,-200},{100,200}}), graphics={
        Rectangle(
        extent={{-100,-200},{100,200}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid,
        borderPattern=BorderPattern.Raised),
        Text(
          extent={{-120,260},{120,200}},
          lineColor={0,0,255},
          textString="%name"),
        Rectangle(
          extent={{-10,120},{10,-140}},
          lineColor={200,200,200},
          fillColor={207,207,207},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,160},{-40,120},{0,120},{40,120},{0,160}},
          lineColor={200,200,200},
          fillColor={207,207,207},
          fillPattern=FillPattern.Solid)}),
Documentation(info="<html>
<p>
Block that controls devices when there is a stage-up command. This sequence is for
water-cooled primary-only parallel boiler plants with headered hot water pumps
and headered condenser water pumps, or air-cooled primary-only parallel boiler
plants with headered hot water pumps.
This development is based on ASHRAE RP-1711 Advanced Sequences of Operation for 
HVAC Systems Phase II – Central Plants and Hydronic Systems (Draft version, March 2020),
section 5.2.4.16, which specifies the step-by-step control of
devices during boiler staging up process.
</p>
<ol>
<li>
Identify the boiler(s) that should be enabled (and disabled, if <code>have_PonyBoiler=true</code>). 
This is implemented in block <code>nexChi</code>. See
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.NextBoiler\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.NextBoiler</a>
for more decriptions.
</li>
<li>
Command operating boilers to reduce demand to 75% (<code>chiDemRedFac</code>) of 
their current load (<code>uChiLoa</code>). Wait until actual demand &lt;

 80% of 
current load up to a maximum of 5 minutes (<code>holChiDemTim</code>) before proceeding.
This is implemented in block <code>chiDemRed</code>. See
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.ReduceDemand\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.ReduceDemand</a>
for more decriptions.
</li>
<li>
Reset the minimum hot water flow setpoint,
<ul>
<li>
For any stage change during which a smaller boiler is disabled and a larger boiler
is enabled, slowly change (<code>byPasSetTim</code>) the minimum hot water flow 
setpoint to the one that includes both boilers are enabled. After new setpoint is 
achieved, wait 1 minute (<code>aftByPasSetTim</code>) to allow loop to stabilize.
</li>
<li>
For any other stage change, reset ((<code>byPasSetTim</code>)) the minimum chilled 
water flow setpoint to the one that includes the new boiler. After new setpoint is 
achieved, wait 1 minute (<code>aftByPasSetTim</code>) to allow loop to stabilize.
</li>
</ul>
The minimum flow setpoint is reset in block <code>minBoiWatFlo</code>
(<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.MinimumFlowBypass.Subsequences.FlowSetpoint\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.MinimumFlowBypass.Subsequences.FlowSetpoint</a>).
Block <code>minBypSet</code> checks if the new setpoint is achieved
(<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.ResetMinBypass\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.ResetMinBypass</a>).
</li>
<li>
Start the next condenser water pump and/or change condenser water pump speed 
to that required of the new stage. Wait 10 seconds (<code>thrTimEnb</code>).
Block <code>enaNexCWP</code> identifies boiler stage for the condenser water pump
control
(<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.EnableCWPump\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.EnableCWPump</a>)
and block <code>conWatPumCon</code> checks if the condenser water pumps have been reset
(<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Pumps.CondenserWater.Controller\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Pumps.CondenserWater.Controller</a>).
</li>
<li>
Enabled head pressure control for the boiler being enabled. Wait 30 seconds (<code>waiTim</code>).
This is implemented in block <code>enaHeaCon</code>. See
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.HeadControl\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.HeadControl</a>
for more decriptions.
</li>
<li>
Slowly (<code>chaChiWatIsoTim</code>) open hot water isolation valve of the boiler 
being enabled. The valve timing should be determined in the fields.
This is implemented in block <code>enaChiIsoVal</code>. See
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.CHWIsoVal\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.CHWIsoVal</a>
for more decriptions.
</li>
<li>
End the staging up process:
<ul>
<li>
If the stage change does not require one boiler enabled and another boiler disabled, 
start the next stage boiler after the isolation valve is fully open.
</li>
<li>
If the stage change does require one boiler enabled and another boiler disabled, 
starting the next stage boiler after the isolation valve is fully open, then shut off
the smaller boiler, close the boiler's hot water isolation valve, disable
the head pressure control loop, and change the minimum hot water flow setpoint 
to the one for the new stage.
</li>
<li>
Release the demand limit, which marks the end of the staging process.
</li>
</ul>
These are implemented in block <code>endUp</code>. See
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.UpEnd\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Processes.Subsequences.UpEnd</a>
for more decriptions.
</li>
</ol>
</html>", revisions="<html>
<ul>
<li>
September 23, 2019, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"));
end Up;