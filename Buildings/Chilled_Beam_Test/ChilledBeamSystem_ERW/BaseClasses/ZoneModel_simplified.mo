within Buildings.Chilled_Beam_Test.ChilledBeamSystem_ERW.BaseClasses;
block ZoneModel_simplified
  "Zone model containing components for zone mixed air volume, thermal capacitance and a chilled beam manifold with air circulation fan"
  extends Buildings.ThermalZones.Detailed.BaseClasses.ConstructionRecords;

  replaceable package MediumA =Buildings.Media.Air
    "Air medium model";

  replaceable package MediumW =Buildings.Media.Water
    "Water medium model";

  Buildings.Controls.OBC.CDL.Interfaces.RealInput QFlo "Power gained by zone"
    annotation (Placement(transformation(extent={{-240,-20},{-200,20}}),
        iconTransformation(extent={{-140,20},{-100,60}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TZon(
    final unit="K",
    displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Measured zone temperature"
    annotation (Placement(transformation(extent={{200,-20},{240,20}}),
        iconTransformation(extent={{100,20},{140,60}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yRelHumZon(
    final unit="1",
    displayUnit="1")
    "Zone relative humidity"
    annotation (Placement(transformation(extent={{200,70},{240,110}}),
      iconTransformation(extent={{100,60},{140,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput VDisAir_flow "Discharge air volume flow rate"
    annotation (Placement(transformation(extent={{200,-100},{240,-60}}),
        iconTransformation(extent={{100,-20},{140,20}})));

  Buildings.Fluid.Sensors.Temperature zonTem(redeclare package Medium = MediumA)
    "Zone temperature sensor"
    annotation (Placement(transformation(extent={{160,-10},{180,10}})));

  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCap(C=
        zonTheCap)
    "Heat capacity for furniture and walls"
    annotation (Placement(transformation(extent={{6,14},{26,34}})));

  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = MediumA,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    X_start={0.006,1},
    m_flow_nominal=mAChiBea_flow_nominal + mA_flow_nominal,
    V=1.2*V,
    nPorts=6) "Zone air volume"
              annotation (Placement(transformation(extent={{6,-16},{26,4}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHea
    "Prescribed heat flow"
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));

  parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal = 4359751.36
    "Nominal heat flow rate of radiator"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.Units.SI.Temperature TRadSup_nominal = 273.15+70
    "Radiator nominal supply water temperature"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.Units.SI.Temperature TRadRet_nominal = 273.15+50
    "Radiator nominal return water temperature"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.Units.SI.MassFlowRate mRad_flow_nominal=0.000604*1000
    "Radiator nominal mass flow rate"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.Units.SI.Volume V=1200
    "Room volume"
    annotation(dialog(group="Zone parameters"));

  parameter Real zonTheCap(
    final unit="J/K",
    displayUnit="J/K",
    final quantity="HeatCapacity") = 2*V*1.2*1500
    "Zone thermal capacitance"
    annotation(dialog(group="Zone parameters"));

  parameter Modelica.Units.SI.Temperature TAir_nominal=273.15 + 23.9
    "Air temperature at nominal condition"
    annotation(dialog(group="Zone parameters"));

  parameter Modelica.Units.SI.MassFlowRate mA_flow_nominal = V*1.2*6/3600
    "Nominal mass flow rate"
    annotation(dialog(group="Zone parameters"));

  parameter Modelica.Units.SI.MassFlowRate mAChiBea_flow_nominal = V*1.2*6/3600
    "Nominal mass flow rate"
    annotation(dialog(group="Zone parameters"));

  Modelica.Fluid.Interfaces.FluidPort_a portChiWat_a(redeclare package Medium =
        MediumW) "CHW inlet port" annotation (Placement(transformation(extent={{
            -50,-210},{-30,-190}}), iconTransformation(extent={{-50,-110},{-30,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_b portChiWat_b(redeclare package Medium =
        MediumW) "CHW outlet port" annotation (Placement(transformation(extent={
            {30,-210},{50,-190}}), iconTransformation(extent={{30,-110},{50,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_a portAir_a(redeclare package Medium =
        MediumA) "Air inlet port" annotation (Placement(transformation(extent={{
            -210,-60},{-190,-40}}), iconTransformation(extent={{90,-50},{110,
            -30}})));
  Modelica.Fluid.Interfaces.FluidPort_b portAir_b(redeclare package Medium =
        MediumA) "Air outlet port" annotation (Placement(transformation(extent={
            {190,-60},{210,-40}}), iconTransformation(extent={{-110,-50},{-90,
            -30}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
  Fluid.Sensors.RelativeHumidity senRelHum(redeclare package Medium = MediumA)
    annotation (Placement(transformation(extent={{80,80},{100,100}})));
  Fluid.Sensors.VolumeFlowRate senVolFlo(redeclare package Medium = MediumA,
      m_flow_nominal=mA_flow_nominal)
    annotation (Placement(transformation(extent={{-140,-60},{-120,-40}})));

  Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumW,
      m_flow_nominal=mRad_flow_nominal)
    annotation (Placement(transformation(extent={{-40,-160},{-20,-140}})));
  Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium = MediumW,
      m_flow_nominal=mRad_flow_nominal)
    annotation (Placement(transformation(extent={{20,-160},{40,-140}})));
  Fluid.HeatExchangers.ActiveBeams.Cooling actBea(
    redeclare package MediumWat = MediumW,
    redeclare package MediumAir = MediumA,
    redeclare
      Fluid.HeatExchangers.ActiveBeams.Data.Trox.DID632A_nozzleH_length6ft_cooling
      perCoo)
    annotation (Placement(transformation(extent={{-14,-168},{14,-144}})));
  Fluid.Movers.FlowControlled_m_flow fan(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAChiBea_flow_nominal,
    redeclare Fluid.Movers.Data.Generic per,
    addPowerToMedium=false,
    dp_nominal=5000)
    annotation (Placement(transformation(extent={{-20,-110},{0,-90}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uVal "Chilled water valve signal" annotation (
      Placement(transformation(extent={{-240,-100},{-200,-60}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=mA_flow_nominal) "Gain"
    annotation (Placement(transformation(extent={{-180,-90},{-160,-70}})));
  ThermalZones.Detailed.Constructions.Construction conBou[nConBou](
    A=datConBou.A,
    til=datConBou.til,
    final layers=datConBou.layers,
    steadyStateInitial=datConBou.steadyStateInitial,
    T_a_start=datConBou.T_a_start,
    T_b_start=datConBou.T_b_start,
    final stateAtSurface_a=datConBou.stateAtSurface_a,
    final stateAtSurface_b=datConBou.stateAtSurface_b) if   haveConBou
    "Heat conduction through opaque constructions that have the boundary conditions of the other side exposed"
    annotation (Placement(transformation(extent={{-6,34},{-46,74}})));
  parameter ThermalZones.Detailed.BaseClasses.ParameterConstruction           datConBou[nConBou](
    each A=0,
    each layers=dummyCon,
    each til=0,
    each azi=0) "Data for construction boundary"
    annotation (Placement(transformation(extent={{-180,160},{-160,180}})),  HideResult=true);
equation
  connect(zonTem.T, TZon)
    annotation (Line(points={{177,0},{220,0}}, color={0,0,127}));
  connect(vol.ports[1], zonTem.port) annotation (Line(points={{12.6667,-16},{
          12.6667,-20},{170,-20},{170,-10}},
                                color={0,127,255}));
  connect(QFlo, preHea.Q_flow)
    annotation (Line(points={{-220,0},{-100,0}}, color={0,0,127}));
  connect(heaCap.port, vol.heatPort)
    annotation (Line(points={{16,14},{0,14},{0,-6},{6,-6}}, color={191,0,0}));
  connect(preHea.port, vol.heatPort)
    annotation (Line(points={{-80,0},{0,0},{0,-6},{6,-6}}, color={191,0,0}));
  connect(portAir_b, vol.ports[2])
    annotation (Line(points={{200,-50},{14,-50},{14,-16}}, color={0,127,255}));
  connect(senRelHum.port, vol.ports[3])
    annotation (Line(points={{90,80},{90,-16},{15.3333,-16}},
                                                         color={0,127,255}));
  connect(portAir_a, senVolFlo.port_a)
    annotation (Line(points={{-200,-50},{-140,-50}}, color={0,127,255}));
  connect(senVolFlo.port_b, vol.ports[4]) annotation (Line(points={{-120,-50},{
          16.6667,-50},{16.6667,-16}},
                          color={0,127,255}));
  connect(senRelHum.phi, yRelHumZon) annotation (Line(points={{101,90},{220,90}},
                              color={0,0,127}));
  connect(senVolFlo.V_flow, VDisAir_flow) annotation (Line(points={{-130,-39},{
          -130,-20},{-6,-20},{-6,-80},{220,-80}},
                                               color={0,0,127}));
  connect(portChiWat_a, senTem.port_a)
    annotation (Line(points={{-40,-200},{-40,-150}}, color={0,127,255}));
  connect(senTem1.port_b, portChiWat_b)
    annotation (Line(points={{40,-150},{40,-200}}, color={0,127,255}));
  connect(senTem.port_b, actBea.watCoo_a)
    annotation (Line(points={{-20,-150},{-14,-150}}, color={0,127,255}));
  connect(actBea.watCoo_b, senTem1.port_a)
    annotation (Line(points={{14,-150},{20,-150}}, color={0,127,255}));
  connect(uVal, gai.u)
    annotation (Line(points={{-220,-80},{-182,-80}},   color={0,0,127}));
  connect(gai.y, fan.m_flow_in) annotation (Line(points={{-158,-80},{-10,-80},{
          -10,-88}},   color={0,0,127}));
  connect(actBea.air_a, vol.ports[5]) annotation (Line(points={{14,-162},{60,-162},
          {60,-40},{18,-40},{18,-16}},     color={0,127,255}));
  connect(fan.port_a, actBea.air_b) annotation (Line(points={{-20,-100},{-36,
          -100},{-36,-162},{-14,-162}},
                                  color={0,127,255}));
  connect(fan.port_b, vol.ports[6]) annotation (Line(points={{0,-100},{19.3333,
          -100},{19.3333,-16}},                     color={0,127,255}));
  connect(actBea.heaPor, vol.heatPort) annotation (Line(points={{0,-168},{0,-180},
          {80,-180},{80,-30},{0,-30},{0,-6},{6,-6}}, color={191,0,0}));
  connect(vol.heatPort, conBou[1].opa_a) annotation (Line(points={{6,-6},{-10,
          -6},{-10,67.3333},{-6,67.3333}}, color={191,0,0}));
  connect(conBou[1].opa_b, port_a) annotation (Line(points={{-46.1333,67.3333},
          {-46.1333,84},{0,84},{0,100}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}), graphics={          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-120,20},{120,-20}},
          textString="%name",
          lineColor={0,0,255})}),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-200,-200},{200,200}})),
    experiment(
      StartTime=6393600,
      StopTime=6566400,
      Interval=60,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"));
end ZoneModel_simplified;
