within Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses;
partial model PartialDXHeatingCoil
  "Partial model for DX heating coil"
  extends
    Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.EssentialParameters;
  extends Buildings.Fluid.Interfaces.TwoPortHeatMassExchanger(
    redeclare replaceable package Medium =
        Modelica.Media.Interfaces.PartialCondensingGases,
    redeclare final Buildings.Fluid.MixingVolumes.MixingVolume vol(
      prescribedHeatFlowRate=true),
    final m_flow_nominal = datCoi.sta[nSta].nomVal.m_flow_nominal);

  constant Boolean use_mCon_flow "Set to true to enable connector for the condenser mass flow rate";

  parameter String substanceName="water" "Name of species substance";

  Modelica.Blocks.Interfaces.RealInput TEvaIn(unit="K", displayUnit="degC")
    "Outside air dry bulb temperature for an air cooled condenser or wetbulb temperature for an evaporative cooled condenser"
    annotation (Placement(transformation(extent={{-120,20},{-100,40}})));

  Modelica.Blocks.Interfaces.RealInput mCon_flow(
    quantity="MassFlowRate",
    unit="kg/s") if use_mCon_flow
    "Water mass flow rate for condenser"
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));

  Modelica.Blocks.Interfaces.RealOutput P(
    quantity="Power",
    unit="W") "Electrical power consumed by the unit"
    annotation (Placement(transformation(extent={{100,80},{120,100}})));
  Modelica.Blocks.Interfaces.RealOutput QSen_flow(quantity="Power", unit="W")
    "Sensible heat flow rate"
    annotation (Placement(transformation(extent={{100,60},{120,80}})));

  Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.DryCoil dryCoi(
    redeclare final package Medium = Medium,
    datCoi=datCoi,
    use_mCon_flow=use_mCon_flow)  "DX cooling coil operation"
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));

  parameter Boolean computeReevaporation=false
    "Set to true to compute reevaporation of water that accumulated on coil"
    annotation (Evaluate=true, Dialog(tab="Dynamics", group="Moisture balance"));

  // Flow reversal is not needed. Also, if ff < ffMin/4, then
  // Q_flow and EIR are set the zero. Hence, it is safe to assume
  // forward flow, which will avoid an event
  Modelica.Blocks.Math.Product pro
    annotation (Placement(transformation(extent={{40,80},{60,100}})));
protected
  parameter Integer i_x(fixed=false) "Index of substance";

  Modelica.Units.SI.SpecificEnthalpy hConIn=inStream(port_a.h_outflow)
    "Enthalpy of air entering the cooling coil";
  Modelica.Units.SI.Temperature TConIn=Medium.temperature_phX(
      p=port_a.p,
      h=hConIn,
      X=XConIn) "Dry bulb temperature of air entering the cooling coil";
  Modelica.Units.SI.MassFraction XConIn[Medium.nXi]=inStream(port_a.Xi_outflow)
    "Mass fraction/absolute humidity of air entering the cooling coil";

  Modelica.Blocks.Sources.RealExpression T(final y=TConIn)
    "Inlet air temperature"
    annotation (Placement(transformation(extent={{-90,18},{-70,38}})));
  Modelica.Blocks.Sources.RealExpression m(final y=port_a.m_flow)
    "Inlet air mass flow rate"
    annotation (Placement(transformation(extent={{-90,34},{-70,54}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow q "Heat extracted by coil"
    annotation (Placement(transformation(extent={{42,44},{62,64}})));

  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TVol
    "Temperature of the control volume"
    annotation (Placement(transformation(extent={{66,16},{78,28}})));

initial algorithm
  // Make sure that |Q_flow_nominal[nSta]| >= |Q_flow_nominal[i]| for all stages because the data
  // of nSta are used in the evaporation model
  for i in 1:(nSta-1) loop
    assert(datCoi.sta[i].nomVal.Q_flow_nominal >= datCoi.sta[nSta].nomVal.Q_flow_nominal,
    "Error in DX coil performance data: Q_flow_nominal of the highest stage must have
    the biggest value in magnitude. Obtained " + Modelica.Math.Vectors.toString(
    {datCoi.sta[i].nomVal.Q_flow_nominal for i in 1:nSta}, "Q_flow_nominal"));
   end for;

  // Compute index of species vector that carries the substance name
  i_x :=-1;
    for i in 1:Medium.nXi loop
      if Modelica.Utilities.Strings.isEqual(string1=Medium.substanceNames[i],
                                            string2=substanceName,
                                            caseSensitive=false) then
        i_x :=i;
      end if;
    end for;
  assert(i_x > 0, "Substance '" + substanceName + "' is not present in medium '"
                  + Medium.mediumName + "'.\n"
                  + "Change medium model to one that has '" + substanceName + "' as a substance.");

equation
  connect(m.y, dryCoi.m_flow) annotation (Line(
      points={{-69,44},{-66,44},{-66,52.4},{-21,52.4}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(q.port, vol.heatPort) annotation (Line(
      points={{62,54},{66,54},{66,22},{-12,22},{-12,-10},{-9,-10}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(dryCoi.Q_flow, q.Q_flow) annotation (Line(
      points={{1,54},{42,54}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TVol.port, q.port) annotation (Line(
      points={{66,22},{66,54},{62,54}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(mCon_flow, dryCoi.mCon_flow) annotation (Line(points={{-110,-30},{-24,
          -30},{-24,40},{-21,40}}, color={0,0,127}));

  connect(TEvaIn, dryCoi.TEvaIn) annotation (Line(points={{-110,30},{-94,30},{-94,
          50},{-21,50}}, color={0,0,127}));
  connect(T.y, dryCoi.TConIn) annotation (Line(points={{-69,28},{-62,28},{-62,55},
          {-21,55}}, color={0,0,127}));
  connect(dryCoi.Q_flow, QSen_flow) annotation (Line(points={{1,54},{26,54},{26,
          70},{110,70}}, color={0,0,127}));
  connect(pro.y, P)
    annotation (Line(points={{61,90},{110,90}}, color={0,0,127}));
  connect(dryCoi.EIR, pro.u1) annotation (Line(points={{1,58},{18,58},{18,96},{38,
          96}}, color={0,0,127}));
  connect(dryCoi.Q_flow, pro.u2) annotation (Line(points={{1,54},{22,54},{22,84},
          {38,84}}, color={0,0,127}));
  annotation (
defaultComponentName="dxCoi",
Documentation(info="<html>
<p>
This partial model is the base class for
<a href=\"modelica://Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.SingleSpeed\">
Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.SingleSpeed</a>
<a href=\"modelica://Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.MultiStage\">
Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.MultiStage</a> and
<a href=\"modelica://Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.VariableSpeed\">
Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.VariableSpeed</a>.
</p>
<p>
See
<a href=\"modelica://Buildings.Fluid.HeatExchangers.DXCoils.UsersGuide\">
Buildings.Fluid.HeatExchangers.DXCoils.UsersGuide</a>
for an explanation of the model.
</p>
</html>",
revisions="<html>
<ul>
<li>
June 19, 2017, by Michael Wetter:<br/>
Added missing <code>replaceable</code> to the medium declaration.<br/>
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/810\">Buildings #810</a>.
</li>
<li>
April 12, 2017, by Michael Wetter:<br/>
Removed temperature connection that is no longer needed.<br/>
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/704\">Buildings #704</a>.
</li>
<li>
February 27, 2017 by Yangyang Fu:<br/>
Added <code>redeclare</code> for the type of <code>cooCap</code> in <code>dxCoo</code>.
</li>
<li>
May 6, 2015 by Michael Wetter:<br/>
Added <code>prescribedHeatFlowRate=true</code> for <code>vol</code>.
</li>
<li>
August 31, 2013, by Michael Wetter:<br/>
Updated model due to change in
<code>Buildings.Fluid.BaseClasses.IndexMassFraction</code>.
</li>
<li>
September 24, 2012 by Michael Wetter:<br/>
Revised documentation.
</li>
<li>
September 4, 2012 by Michael Wetter:<br/>
Moved assignments to declaration section to avoid mixing graphical modeling with textual
modeling in <code>equation</code> section.
Redeclare medium model as <code>Modelica.Media.Interfaces.PartialCondensingGases</code>
to remove errors during model check.
Added output connectors for sensible and latent heat flow rate.
</li>
<li>
April 12, 2012 by Kaustubh Phalak:<br/>
First implementation.
</li>
</ul>

</html>"),
    Icon(graphics={Text(
          extent={{-138,64},{-80,46}},
          textColor={0,0,127},
          textString="TConIn"), Text(
          extent={{58,98},{102,78}},
          textColor={0,0,127},
          textString="P"),      Text(
          extent={{54,60},{98,40}},
          textColor={0,0,127},
          textString="QLat"),   Text(
          extent={{54,80},{98,60}},
          textColor={0,0,127},
          textString="QSen")}));
end PartialDXHeatingCoil;