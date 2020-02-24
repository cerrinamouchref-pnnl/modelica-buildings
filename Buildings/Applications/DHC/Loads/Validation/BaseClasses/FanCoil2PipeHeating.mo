within Buildings.Applications.DHC.Loads.Validation.BaseClasses;
model FanCoil2PipeHeating
  "Model of a purely sensible two-pipe fan coil unit computing a required heating water mass flow rate"
  extends Buildings.Applications.DHC.Loads.BaseClasses.PartialTerminalUnit(
    redeclare package Medium1 = Buildings.Media.Water,
    redeclare package Medium2 = Buildings.Media.Air,
    final have_heaPor=false,
    final have_fluPor=false,
    final have_fan=true,
    final have_watHea=true,
    final have_watCoo=false,
    final have_QReq_flow=true,
    final allowFlowReversal=false,
    final have_chaOve=false,
    final have_eleHea=false,
    final have_eleCoo=false,
    final have_TSen=false,
    final have_weaBus=false,
    final have_pum=false,
    final mHeaWat_flow_nominal=abs(QHea_flow_nominal/cpHeaWat_nominal/(
      T_aHeaWat_nominal - T_bHeaWat_nominal)));
  import hexConfiguration = Buildings.Fluid.Types.HeatExchangerConfiguration;
  final parameter hexConfiguration hexConHea=
    hexConfiguration.CounterFlow
    "Heating heat exchanger configuration";
  final parameter hexConfiguration hexConCoo=
    hexConfiguration.CounterFlow
    "Cooling heat exchanger configuration";
  parameter Boolean have_speVar = true
    "Set to true for a variable speed fan (otherwise fan is always on)";
  Buildings.Fluid.Movers.FlowControlled_m_flow fan(
    redeclare final package Medium=Medium2,
    energyDynamics=energyDynamics,
    m_flow_nominal=mLoaHea_flow_nominal,
    redeclare Fluid.Movers.Data.Generic per,
    nominalValuesDefineDefaultPressureCurve=true,
    dp_nominal=200,
    final allowFlowReversal=allowFlowReversal)
    annotation (Placement(transformation(extent={{90,-10},{70,10}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID con(
    Ti=10,
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    reverseAction=false,
    yMin=0) "PI controller"
    annotation (Placement(transformation(extent={{-10,210},{10,230}})));
  Buildings.Fluid.HeatExchangers.DryCoilEffectivenessNTU hex(
    redeclare final package Medium1 = Medium1,
    redeclare final package Medium2 = Medium2,
    final configuration=hexConHea,
    final m1_flow_nominal=mHeaWat_flow_nominal,
    final m2_flow_nominal=mLoaHea_flow_nominal,
    final dp1_nominal=0,
    dp2_nominal=200,
    final Q_flow_nominal=QHea_flow_nominal,
    final T_a1_nominal=T_aHeaWat_nominal,
    final T_a2_nominal=T_aLoaHea_nominal,
    final allowFlowReversal1=allowFlowReversal,
    final allowFlowReversal2=allowFlowReversal)
    annotation (Placement(transformation(extent={{-80,4},{-60,-16}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gaiMasFlo(k=mHeaWat_flow_nominal)
    annotation (Placement(transformation(extent={{40,210},{60,230}})));
  Modelica.Blocks.Sources.RealExpression Q_flowHea(y=hex.Q2_flow)
    annotation (Placement(transformation(extent={{120,210},{140,230}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gaiFloNom2(k=mLoaHea_flow_nominal)
    annotation (Placement(transformation(extent={{56,170},{76,190}})));
  Fluid.Sources.Boundary_pT sinAir(
    redeclare package Medium = Medium2,
    use_T_in=false,
    nPorts=1)
    "Sink for supply air"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-152,0})));
  Fluid.Sources.Boundary_pT retAir(
    redeclare package Medium = Medium2,
    p(displayUnit="Pa"),
    use_T_in=true,
    nPorts=1)
    "Source for return air"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={150,0})));
  Buildings.Applications.DHC.Loads.BaseClasses.SimpleRoom TLoaODE(
    TOutHea_nominal=273.15 - 5,
    TIndHea_nominal=T_aLoaHea_nominal,
    QHea_flow_nominal=QHea_flow_nominal)
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gaiHeaFlo(k=1/QHea_flow_nominal)
    annotation (Placement(transformation(extent={{-40,210},{-20,230}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gaiHeaFlo1(k=1/QHea_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,190})));
  Buildings.Controls.OBC.CDL.Logical.Switch swi
    "Logical switch"
    annotation (Placement(transformation(extent={{30,170},{50,190}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant one(k=1)
    "One constant"
    annotation (Placement(transformation(extent={{0,130},{20,150}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant con1(k=have_speVar)
    annotation (Placement(transformation(extent={{-60,160},{-40,180}})));
equation
  if have_fluPor then
  end if;
  if not have_QReq_flow then
  end if;
  connect(gaiFloNom2.y, fan.m_flow_in)
    annotation (Line(points={{78,180},{80,180},{80,12}}, color={0,0,127}));

  connect(con.y, gaiMasFlo.u)
    annotation (Line(points={{12,220},{38,220}}, color={0,0,127}));

  connect(gaiMasFlo.y, scaMasFloReqHeaWat.u) annotation (Line(points={{62,220},
          {100,220},{100,100},{158,100}},color={0,0,127}));
  connect(fan.P, scaPFan.u) annotation (Line(points={{69,9},{60,9},{60,20},{150,
          20},{150,140},{158,140}},
                     color={0,0,127}));
  connect(Q_flowHea.y, scaQActHea_flow.u) annotation (Line(points={{141,220},
          {150,220},{150,220},{158,220}}, color={0,0,127}));
  connect(fan.port_b, hex.port_a2)
    annotation (Line(points={{70,0},{-60,0}}, color={0,127,255}));
  connect(hex.port_b2, sinAir.ports[1])
    annotation (Line(points={{-80,0},{-142,0}}, color={0,127,255}));
  connect(TSetHea, TLoaODE.TSet)
    annotation (Line(points={{-220,220},{-120,220},{-120,48},{-12,48}},
                                                     color={0,0,127}));
  connect(TLoaODE.TAir, retAir.T_in) annotation (Line(points={{12,40},{180,40},{
          180,4},{162,4}}, color={0,0,127}));
  connect(gaiHeaFlo.y, con.u_s)
    annotation (Line(points={{-18,220},{-12,220}}, color={0,0,127}));
  connect(con.u_m, gaiHeaFlo1.y) annotation (Line(points={{0,208},{0,207},{
          8.88178e-16,207},{8.88178e-16,202}}, color={0,0,127}));
  connect(retAir.ports[1], fan.port_a)
    annotation (Line(points={{140,0},{90,0}}, color={0,127,255}));
  connect(swi.y, gaiFloNom2.u)
    annotation (Line(points={{52,180},{54,180}}, color={0,0,127}));
  connect(con.y, swi.u1) annotation (Line(points={{12,220},{24,220},{24,188},{28,
          188}}, color={0,0,127}));
  connect(one.y, swi.u3) annotation (Line(points={{22,140},{24,140},{24,172},{28,
          172}}, color={0,0,127}));
  connect(con1.y, swi.u2) annotation (Line(points={{-38,170},{20,170},{20,180},{
          28,180}}, color={255,0,255}));
  connect(scaQReqHea_flow.y, gaiHeaFlo.u) annotation (Line(points={{-158,100},{
          -80,100},{-80,220},{-42,220}}, color={0,0,127}));
  connect(scaQReqHea_flow.y, TLoaODE.QReq_flow) annotation (Line(points={{-158,
          100},{-80,100},{-80,40},{-12,40}}, color={0,0,127}));
  connect(Q_flowHea.y, gaiHeaFlo1.u) annotation (Line(points={{141,220},{150,
          220},{150,160},{0,160},{0,178}}, color={0,0,127}));
  connect(Q_flowHea.y, TLoaODE.QAct_flow) annotation (Line(points={{141,220},{
          150,220},{150,160},{-20,160},{-20,32},{-12,32}}, color={0,0,127}));
  connect(scaHeaWatFloInl.port_b, hex.port_a1) annotation (Line(points={{-160,
          -220},{-120,-220},{-120,-12},{-80,-12}}, color={0,127,255}));
  connect(hex.port_b1, scaHeaWatFloOut.port_a) annotation (Line(points={{-60,-12},
          {-38,-12},{-38,-220},{160,-220}},      color={0,127,255}));
annotation (
Documentation(
info="<html>
<p>
This is a simplified model of a two-pipe fan coil unit for heating. It is 
intended to be used:
</p>
<ul>
<li>
in a case where the room thermal loads are provided as time series: it thus
takes the load as an input,
</li>
<li>
in conjunction with
<a href=\"modelica://Buildings.Applications.DHC.Loads.BaseClasses.FlowDistribution\">
Buildings.Applications.DHC.Loads.BaseClasses.FlowDistribution</a>: 
it thus computes the water mass flow rate required to meet the load.
</li>
</ul>
<p>
For the sake of computational performance, a PI controller is used instead of an inverse 
model of the heat exchanger to assess the required water mass flow rate.
The controller output signal is mapped linearly to both: 
</p>
<ul>
<li>
the water mass flow rate, from zero to its nominal value,
</li>
<li>
the air mass flow rate, from zero to its nominal value.
</li>
</ul>
<p>
The controller tracks the load while the impact of an unmet load on the room 
air temperature is assessed with
<a href=\"modelica://Buildings.Applications.DHC.Loads.BaseClasses.SimpleRoom\">
Buildings.Applications.DHC.Loads.BaseClasses.SimpleRoom</a>.
</p>
</html>"));
end FanCoil2PipeHeating;