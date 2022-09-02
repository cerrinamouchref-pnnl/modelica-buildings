within Buildings.Fluid.ZoneEquipment.FanCoilUnit.Validation;
model CoolingMode

  extends Modelica.Icons.Example;

  replaceable package MediumA = Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialCondensingGases
    "Medium model for air";

  replaceable package MediumW = Buildings.Media.Water
    "Medium model for water";

  Buildings.Fluid.Sources.Boundary_pT sinCoo(
    redeclare package Medium = MediumW,
    final T=279.15,
    final nPorts=1)
    "Sink for chilled water"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,origin={40,-80})));

  Buildings.Fluid.Sources.Boundary_pT sinHea(
    redeclare package Medium = MediumW,
    final T=318.15,
    final nPorts=1)
    "Sink for heating hot water"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,origin={-40,-80})));

  Buildings.Fluid.ZoneEquipment.FanCoilUnit.FanCoilUnit fanCoiUni(
    final heatingCoilType=Buildings.Fluid.ZoneEquipment.FanCoilUnit.Types.heatingCoil.heatingHotWater,
    final dpAirTot_nominal(displayUnit="Pa") = 100,
    final mAirOut_flow_nominal=fCUSizing.mAirOut_flow_nominal,
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    final mAir_flow_nominal=fCUSizing.mAir_flow_nominal,
    final QHeaCoi_flow_nominal=13866,
    final mHotWat_flow_nominal=fCUSizing.mHotWat_flow_nominal,
    final UAHeaCoi_nominal=fCUSizing.UAHeaCoi_nominal,
    final mChiWat_flow_nominal=fCUSizing.mChiWat_flow_nominal,
    final UACooCoi_nominal=fCUSizing.UACooCoiTot_nominal,
    redeclare Data.customFCUFan fanPer)
    "Fan coil system model"
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));

  Buildings.Fluid.Sources.MassFlowSource_T souCoo(
    redeclare package Medium = MediumW,
    final use_m_flow_in=true,
    final use_T_in=true,
    final nPorts=1)
    "Source for chilled water"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={70,-80})));

  Buildings.Fluid.Sources.MassFlowSource_T souHea(
    redeclare package Medium = MediumW,
    final use_m_flow_in=true,
    final use_T_in=true,
    final nPorts=1)
    "Source for heating hot water"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={10,-90})));

  Buildings.Fluid.ZoneEquipment.FanCoilUnit.Validation.Data.FCUSizing fCUSizing
    "Sizing parameters for fan coil unit"
    annotation (Placement(transformation(extent={{-140,60},{-120,80}})));

  Modelica.Blocks.Sources.CombiTimeTable datRea(
    final tableOnFile=true,
    final fileName=ModelicaServices.ExternalReferences.loadResource(
      "./Buildings/Resources/Data/Fluid/ZoneEquipment/FanCoilAutoSize_ConstantFlowVariableFan.dat"),
    final columns=2:19,
    final tableName="EnergyPlus",
    final smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Reader for EnergyPlus simulation results"
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));

  Buildings.Fluid.Sources.Boundary_pT souAir(
    redeclare package Medium = MediumA,
    final use_Xi_in=true,
    final p(displayUnit="Pa") = 101325 + 100,
    final use_T_in=true,
    final T=279.15,
    final nPorts=1)
    "Source for zone air"
    annotation (Placement(transformation(extent={{20,20},{40,40}})));

  Buildings.Fluid.Sources.Boundary_pT sinAir(
    redeclare package Medium = MediumA,
    final p=101325 + 100,
    final T=279.15,
    final nPorts=1)
    "Sink for zone air"
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con(
    final k=0.2)
    "Constant real signal of 0.2 for the outdoor air economizer"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));

  Buildings.Controls.OBC.CDL.Continuous.AddParameter addPar[3](
    final p=fill(273.15, 3))
    "Add 273.15 to temperature values from EPlus to convert it to Kelvin from Celsius"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    final filNam=ModelicaServices.ExternalReferences.loadResource(
      "./Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    "Outdoor weather data"
    annotation (Placement(transformation(extent={{-80,100},{-60,120}})));

  Modelica.Blocks.Sources.RealExpression PModCoo(
    final y=-fanCoiUni.cooCoiCHW.Q2_flow)
    "Cooling power consumption in Modelica model"
    annotation (Placement(transformation(extent={{20,130},{40,150}})));

  Modelica.Blocks.Sources.RealExpression PModCooSen(
    final y=-fanCoiUni.cooCoiCHW.QSen2_flow)
    "Sensible cooling power consumption in Modelica model"
    annotation (Placement(transformation(extent={{60,130},{80,150}})));

  Modelica.Blocks.Sources.RealExpression PModCooLat(
    final y=-fanCoiUni.cooCoiCHW.QLat2_flow)
    "Latent cooling power consumption in Modelica model"
    annotation (Placement(transformation(extent={{100,130},{120,150}})));

  Modelica.Blocks.Sources.RealExpression PModCooCal(
    final y=-1000*4200*fanCoiUni.VChiWat_flow.V_flow*(fanCoiUni.TChiWatSupSen.T - fanCoiUni.TChiWatRetSen.T))
    "Cooling power consumption in Modelica model calculated from measured temperatures and flowrate in chillled water loop"
    annotation (Placement(transformation(extent={{20,110},{40,130}})));

  Buildings.Controls.OBC.CDL.Continuous.Divide div
    "Calculate mass fractions of constituents"
    annotation (Placement(transformation(extent={{-60,-130},{-40,-110}})));

  Buildings.Controls.OBC.CDL.Continuous.AddParameter addPar1(
    final p=1)
    "Add 1 to humidity ratio value to find total mass of moist air"
    annotation (Placement(transformation(extent={{-120,-150},{-100,-130}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con1(
    final k=1)
    "Constant real signal of 1 for holding the hot water and chilled water control valves fully open"
    annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));

equation

  connect(fanCoiUni.port_CCW_outlet, sinCoo.ports[1])
    annotation (Line(points={{12,-10},{12,-70},{40,-70}},
                                                        color={0,127,255}));

  connect(fanCoiUni.port_HHW_outlet, sinHea.ports[1]) annotation (Line(points={{4,-10},
          {4,-60},{-40,-60},{-40,-70}},           color={0,127,255}));

  connect(souCoo.ports[1], fanCoiUni.port_CCW_inlet) annotation (Line(points={{70,-70},
          {70,-60},{16,-60},{16,-10}},       color={0,127,255}));

  connect(souHea.ports[1], fanCoiUni.port_HHW_inlet) annotation (Line(points={{10,-80},
          {10,-10},{8,-10}},                   color={0,127,255}));

  connect(souAir.ports[1], fanCoiUni.port_return) annotation (Line(points={{40,30},
          {50,30},{50,0},{20,0}},     color={0,127,255}));

  connect(sinAir.ports[1], fanCoiUni.port_supply) annotation (Line(points={{40,-30},
          {50,-30},{50,-4},{20,-4}},      color={0,127,255}));

  connect(con.y, fanCoiUni.uOA) annotation (Line(points={{-58,30},{-20,30},
          {-20,6},{-2,6}},
                       color={0,0,127}));

  connect(addPar[1].y, souAir.T_in) annotation (Line(points={{-58,70},{-16,
          70},{-16,34},{18,34}},
                             color={0,0,127}));

  connect(addPar[2].y, souHea.T_in) annotation (Line(points={{-58,70},{-16,70},{
          -16,-120},{6,-120},{6,-102}},
                               color={0,0,127}));

  connect(addPar[3].y, souCoo.T_in) annotation (Line(points={{-58,70},{-16,70},{
          -16,-120},{66,-120},{66,-92}},  color={0,0,127}));

  connect(weaDat.weaBus, fanCoiUni.weaBus) annotation (Line(
      points={{-60,110},{-10,110},{-10,8},{2,8}},
      color={255,204,51},
      thickness=0.5));

  connect(datRea.y[5], addPar[1].u) annotation (Line(points={{-119,0},{-110,0},{
          -110,70},{-82,70}}, color={0,0,127}));
  connect(datRea.y[7], addPar[3].u) annotation (Line(points={{-119,0},{-110,0},{
          -110,70},{-82,70}}, color={0,0,127}));
  connect(datRea.y[9], addPar[2].u) annotation (Line(points={{-119,0},{-110,0},{
          -110,70},{-82,70}}, color={0,0,127}));
  connect(datRea.y[6], fanCoiUni.uFan) annotation (Line(points={{-119,0},{
          -40,0},{-40,2},{-2,2}},                 color={0,0,127}));
  connect(datRea.y[16], addPar1.u) annotation (Line(points={{-119,0},{-110,0},{-110,
          -120},{-130,-120},{-130,-140},{-122,-140}}, color={0,0,127}));
  connect(datRea.y[16], div.u1) annotation (Line(points={{-119,0},{-110,0},{-110,
          -120},{-100,-120},{-100,-114},{-62,-114}}, color={0,0,127}));
  connect(addPar1.y, div.u2) annotation (Line(points={{-98,-140},{-70,-140},{-70,
          -126},{-62,-126}}, color={0,0,127}));
  connect(div.y, souAir.Xi_in[1]) annotation (Line(points={{-38,-120},{-26,
          -120},{-26,26},{18,26}},
                             color={0,0,127}));
  connect(con1.y, fanCoiUni.uCoo) annotation (Line(points={{-58,-20},{-30,
          -20},{-30,-2},{-2,-2}},
                              color={0,0,127}));
  connect(con1.y, fanCoiUni.uHea) annotation (Line(points={{-58,-20},{-30,
          -20},{-30,-6},{-2,-6}},
                              color={0,0,127}));
  connect(datRea.y[10], souHea.m_flow_in) annotation (Line(points={{-119,0},{-110,
          0},{-110,-100},{-20,-100},{-20,-112},{2,-112},{2,-102}},      color={
          0,0,127}));
  connect(datRea.y[8], souCoo.m_flow_in) annotation (Line(points={{-119,0},{
          -110,0},{-110,-100},{-20,-100},{-20,-112},{62,-112},{62,-92}}, color=
          {0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false,
      extent={{-100,-100},{100,100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false,
      extent={{-160,-160},{160,160}})),
    experiment(
      StopTime=86400,
      Interval=60,
      __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file= "modelica://Buildings/Resources/Scripts/Dymola/Fluid/ZoneEquipment/FanCoilUnit/Validation/FanCoilUnit_coolingMode.mos"
      "Simulate and plot"),
    Documentation(info="<html>
      <p>
      This is an open-loop validation model for the fan coil unit system model 
      implemented in class <a href=\"modelica://Buildings.Fluid.ZoneEquipment.FanCoilUnit.FanCoilUnit\">
      Buildings.Fluid.ZoneEquipment.FanCoilUnit.FanCoilUnit</a>. It consists of:
      <ul>
      <li>
      an instance of the fan coil unit system model <code>fanCoiUni</code>.
      </li>
      <li>
      mixed volume <code>souAir</code> for imposing the boundary conditions of 
      the zone air.
      </li>
      <li>
      ideal media sources <code>souCoo</code> and <code>souHea</code> for simulating 
      the supply of chilled water and heating hot-water respectively.
      </li>
      <li>
      data-table reader <code>datRea</code> for reading the simulation results from EnergyPlus.
      </li>
      </ul>
      </p>
      <p>
      The simulation model is set-up to replicate an EnergyPlus model <code>FanCoilAutoSize_ConstantFlowVariableFan.idf</code>
      (available in the <code>/Resources/Data</code> section for this subpackage.)
      An annual simulation was run on the above EnergyPlus model, and various output 
      variables were recorded. These were then inserted into a data-file that is 
      read by <code>datRea</code> in this model.
      <br>
      The data values are used to impose the boundary conditions on the simulation 
      as well as compare the performance of the Modelica model with the equivalent 
      EnergyPlus model. Once the simulation is complete, the values for the supply
      air temperature and supply air flowrate, as well as the power consumption 
      of the various components are compared against their counterparts from the 
      EnergyPlus model.
      <br>
      This model validates the cooling mode operation by running the simulation from 07/01
      through 07/10 with the weather file <code>USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos</code>.
      It then makes plots for the supply air temperature and flowrate, as well as 
      the energy consumption of the cooling coil, including sensible and latent components,
      and the energy consumption of the supply fan.
      </p>
      </html>", revisions="<html>
      <ul>
      <li>
      August 03, 2022 by Karthik Devaprasad:<br/>
      First implementation.
      </li>
      </ul>
      </html>"));
end CoolingMode;