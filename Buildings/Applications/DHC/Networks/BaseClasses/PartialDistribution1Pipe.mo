within Buildings.Applications.DHC.Networks.BaseClasses;
partial model PartialDistribution1Pipe
  "Partial model for one-pipe distribution network"
  extends PartialDistribution;

  replaceable model Model_pipDis = Fluid.Interfaces.PartialTwoPortInterface (
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal)
    "Model for distribution pipe";
  parameter Integer iConPreRel(min=0, max=nCon) = 0
    "Index of the connection where the pressure drop is sensed (0 for no sensor)"
    annotation(Dialog(tab="General"), Evaluate=true);
  parameter Modelica.SIunits.MassFlowRate mDis_flow_nominal
    "Nominal mass flow rate in the distribution line"
    annotation(Dialog(tab="General", group="Nominal condition"));
  parameter Modelica.SIunits.MassFlowRate mCon_flow_nominal[nCon]
    "Nominal mass flow rate in each connection line"
    annotation(Dialog(tab="General", group="Nominal condition"));
  // IO CONNECTORS
  Modelica.Blocks.Interfaces.RealOutput dp(
    final quantity="PressureDifference", final displayUnit="Pa") if iConPreRel > 0
    "Pressure difference at given location (sensed)"
    annotation (Placement(transformation(extent={{100,40},{140,80}}),
      iconTransformation(extent={{200,50}, {220,70}})));
  // COMPONENTS
  replaceable PartialConnection1Pipe con[nCon](
    redeclare each final package Medium = Medium,
    each final mDis_flow_nominal=mDis_flow_nominal,
    final mCon_flow_nominal=mCon_flow_nominal,
    each final allowFlowReversal=allowFlowReversal)
    "Connection to agent"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Model_pipDis pipEnd(
    redeclare final package Medium = Medium,
    final m_flow_nominal=mDis_flow_nominal,
    final allowFlowReversal=allowFlowReversal)
    "Pipe representing the end of the distribution line (after last connection)"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  connect(con.port_bCon, ports_bCon)
    annotation (Line(points={{0,10},{0,40},{-80,
          40},{-80,100}}, color={0,127,255}));
  connect(ports_aCon, con.port_aCon)
    annotation (Line(points={{80,100},{80,40},
          {6,40},{6,10}}, color={0,127,255}));
  // Connecting outlets to inlets for all instances of connection component
  if nCon >= 2 then
    for i in 2:nCon loop
      connect(con[i - 1].port_bDis, con[i].port_aDis);
    end for;
  end if;
  connect(port_aDisSup, con[1].port_aDis)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(con[nCon].port_bDis, pipEnd.port_a)
    annotation (Line(points={{10,0},{40,0}}, color={0,127,255}));
  connect(pipEnd.port_b, port_bDisSup)
    annotation (Line(points={{60,0},{100,0}}, color={0,127,255}));
  annotation (
    defaultComponentName="dis",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-6,-200},{6,200}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={0,0},
          rotation=90),
        Rectangle(
          extent={{-53,4},{53,-4}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={-120,47},
          rotation=90),
        Rectangle(
          extent={{-53,4},{53,-4}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={120,47},
          rotation=90)}),
    Diagram( coordinateSystem(preserveAspectRatio=false)));
end PartialDistribution1Pipe;