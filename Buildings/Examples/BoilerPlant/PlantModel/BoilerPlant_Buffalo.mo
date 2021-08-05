within Buildings.Examples.BoilerPlant.PlantModel;
model BoilerPlant_Buffalo
  "Boiler plant model for closed loop testing"
  extends Modelica.Icons.Example;
  replaceable package MediumA =
      Buildings.Media.Air;

  replaceable package MediumW =
      Buildings.Media.Water
    "Medium model";

  parameter Modelica.SIunits.MassFlowRate mA_flow_nominal = V*1.2*6/3600
    "Nominal mass flow rate"
    annotation(dialog(group="Zone parameters"));

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = boiEff1[1]*boiCap1 + boiEff2[1]*boiCap2
    "Nominal heat flow rate of radiator"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.SIunits.HeatFlowRate boiCap1= 2200000
    "Boiler capacity for boiler-1"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.HeatFlowRate boiCap2= 2200000
    "Boiler capacity for boiler-2"
    annotation(dialog(group="Boiler parameters"));

  parameter Real boiEff1[6] = {0.6246, 0.7711, -1.2077*10e-15, 0.008576, -0.005933, 0.003156}
    "Efficiency for boiler-1"
    annotation(dialog(group="Boiler parameters"));

  parameter Real boiEff2[6] = {0.6246, 0.7711, -1.2077*10e-15, 0.008576, -0.005933, 0.003156}
    "Efficiency for boiler-2"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.Temperature TRadSup_nominal = 273.15+70
    "Radiator nominal supply water temperature"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.SIunits.Temperature TRadRet_nominal = 273.15+50
    "Radiator nominal return water temperature"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.SIunits.MassFlowRate mRad_flow_nominal=0.096323 * 1000
    "Radiator nominal mass flow rate"
    annotation(dialog(group="Radiator parameters"));

  parameter Modelica.SIunits.Temperature TBoiSup_nominal = 273.15+70
    "Boiler nominal supply water temperature"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.Temperature TBoiRet_min = 273.15+60
    "Boiler minimum return water temperature"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal1=mRad_flow_nominal
    "Boiler-1 nominal mass flow rate"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal2=mRad_flow_nominal
    "Boiler-2 nominal mass flow rate"
    annotation(dialog(group="Boiler parameters"));

  parameter Modelica.SIunits.Volume V=126016.35
    "Room volume"
    annotation(dialog(group="Zone parameters"));

  parameter Modelica.SIunits.Temperature TAir_nominal=273.15 + 23.9
    "Air temperature at nominal condition"
    annotation(dialog(group="Zone parameters"));

  parameter Real zonTheCap = 2*V*1.2*1500
    "Zone thermal capacitance"
    annotation(dialog(group="Zone parameters"));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uBoiSta[2]
    "Boiler status signal"
    annotation (Placement(transformation(extent={{-360,140},{-320,180}}),
      iconTransformation(extent={{-140,60},{-100,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPumSta[2]
    "Pump status signal"
    annotation (Placement(transformation(extent={{-360,20},{-320,60}}),
      iconTransformation(extent={{-140,0},{-100,40}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uHotIsoVal[2](
    final unit="1",
    displayUnit="1")
    "Hot water isolation valve signal"
    annotation (Placement(transformation(extent={{-360,60},{-320,100}}),
      iconTransformation(extent={{-140,30},{-100,70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uPumSpe(
    final unit="1",
    displayUnit="1")
    "Pump speed signal"
    annotation (Placement(transformation(extent={{-360,-20},{-320,20}}),
      iconTransformation(extent={{-140,-40},{-100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uBypValSig(
    final unit="1",
    displayUnit="1")
    "Bypass valve signal"
    annotation (Placement(transformation(extent={{-360,-60},{-320,-20}}),
      iconTransformation(extent={{-140,-70},{-100,-30}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput ySupTem(
    final unit="K",
    displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Measured supply temperature"
    annotation (Placement(transformation(extent={{320,80},{360,120}}),
      iconTransformation(extent={{100,70},{140,110}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yRetTem(
    final unit="K",
    displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Measured return temperature"
    annotation (Placement(transformation(extent={{320,40},{360,80}}),
      iconTransformation(extent={{100,40},{140,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yHotWatDp[1](
    final unit="Pa",
    displayUnit="Pa")
    "Hot water differential pressure between supply and return"
    annotation (Placement(transformation(extent={{320,-10},{360,30}}),
      iconTransformation(extent={{100,10},{140,50}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput VHotWat_flow(
    final unit="m3/s",
    displayUnit="m3/s",
    final quantity="VolumeFlowRate")
    "Measured flowrate in primary circuit"
    annotation (Placement(transformation(extent={{320,-50},{360,-10}}),
      iconTransformation(extent={{100,-20},{140,20}})));

  Buildings.Fluid.Boilers.BoilerPolynomial boi(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal=mBoi_flow_nominal1,
    final dp_nominal=5000,
    final Q_flow_nominal=boiCap1,
    final T_nominal=TBoiSup_nominal,
    final effCur=Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear,
    final a=boiEff1,
    final fue=Fluid.Data.Fuels.NaturalGasHigherHeatingValue(),
    final UA=boiCap1/39.81)
    "Boiler"
    annotation (Placement(transformation(extent={{110,-160},{90,-140}})));

  Buildings.Fluid.Sources.Boundary_pT preSou(
    redeclare package Medium = MediumW, nPorts=1)
    "Source for pressure and to account for thermal expansion of water"
    annotation (Placement(transformation(extent={{260,-210},{240,-190}})));

  Buildings.Fluid.Boilers.BoilerPolynomial           boi1(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal=mBoi_flow_nominal2,
    final dp_nominal=5000,
    final Q_flow_nominal=boiCap2,
    final T_nominal=TBoiSup_nominal,
    final effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    final a=boiEff2,
    final fue=Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue(),
    final UA=boiCap2/39.81)
    "Boiler"
    annotation (Placement(transformation(extent={{110,-220},{90,-200}})));

  Buildings.Fluid.Movers.SpeedControlled_y pum(
    redeclare package Medium = Media.Water,
    final allowFlowReversal=true,
    redeclare Fluid.Movers.Data.Pumps.Wilo.customPumpCurves_Buffalo per,
    final inputType=Buildings.Fluid.Types.InputType.Continuous,
    final addPowerToMedium=false,
    final riseTime=60)
    "Hot water primary pump-1"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={-70,-40})));

  Buildings.Fluid.FixedResistances.Junction spl1(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={mBoi_flow_nominal2,-mRad_flow_nominal,mBoi_flow_nominal1},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={-30,-150})));

  Buildings.Fluid.Movers.SpeedControlled_y pum1(
    redeclare package Medium =Media.Water,
    final allowFlowReversal=true,
    redeclare Fluid.Movers.Data.Pumps.Wilo.customPumpCurves_Buffalo per,
    final inputType=Buildings.Fluid.Types.InputType.Continuous,
    final addPowerToMedium=false,
    final riseTime=60)
    "Hot water primary pump-2"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={10,-40})));

  Buildings.Fluid.FixedResistances.Junction spl2(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={mRad_flow_nominal,-mRad_flow_nominal/2,-
        mRad_flow_nominal/2},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={-30,-100})));

  Buildings.Fluid.FixedResistances.Junction spl3(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={mRad_flow_nominal/2,-mRad_flow_nominal,
        mRad_flow_nominal/2},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={-30,0})));

  Buildings.Fluid.FixedResistances.Junction spl4(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={mRad_flow_nominal,-mRad_flow_nominal,-mRad_flow_nominal},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
      origin={-30,40})));

  Buildings.Fluid.Actuators.Valves.TwoWayLinear val(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mRad_flow_nominal,
    final dpValve_nominal=0.1)
    "Minimum flow bypass valve"
    annotation (Placement(transformation(extent={{80,30},{100,50}})));

  Buildings.Fluid.FixedResistances.Junction spl5(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={mRad_flow_nominal,mRad_flow_nominal,-mRad_flow_nominal},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=270,
      origin={210,40})));

  Buildings.Fluid.Actuators.Valves.TwoWayLinear val1(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mBoi_flow_nominal1,
    final dpValve_nominal=0.1,
    init=Modelica.Blocks.Types.Init.InitialState,
    y_start=0)
    "Isolation valve for boiler-2"
    annotation (Placement(transformation(extent={{0,-220},{20,-200}})));

  Buildings.Fluid.Actuators.Valves.TwoWayLinear val2(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mBoi_flow_nominal2,
    final dpValve_nominal=0.1,
    init=Modelica.Blocks.Types.Init.InitialState,
    y_start=0)
    "Isolation valve for boiler-1"
    annotation (Placement(transformation(extent={{0,-160},{20,-140}})));

  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea[2]
    "Boolean to Real conversion"
    annotation (Placement(transformation(extent={{-220,30},{-200,50}})));

  Buildings.Controls.OBC.CDL.Continuous.Product pro[2]
    "Element-wise product"
    annotation (Placement(transformation(extent={{-210,-20},{-190,0}})));

  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1[2]
    "Boolean to Real conversion"
    annotation (Placement(transformation(extent={{-160,150},{-140,170}})));

  Buildings.Fluid.Sensors.RelativePressure senRelPre(
    redeclare package Medium = Media.Water)
    "Differential pressure sensor between hot water supply and return"
    annotation (Placement(transformation(extent={{80,70},{100,90}})));

  Buildings.Fluid.Sensors.VolumeFlowRate senVolFlo(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mRad_flow_nominal/1000)
    "Volume flow-rate through primary circuit"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
        Media.Water, m_flow_nominal=1000*0.0006)
    annotation (Placement(transformation(extent={{-20,110},{0,130}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium =
        Media.Water, m_flow_nominal=1000*0.0006)
    annotation (Placement(transformation(extent={{180,110},{200,130}})));

  Buildings.Controls.OBC.CDL.Routing.RealReplicator reaRep(nout=1)
    "Real replicator"
    annotation (Placement(transformation(extent={{280,0},{300,20}})));

  Buildings.Fluid.Actuators.Valves.TwoWayLinear val4(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mRad_flow_nominal/2,
    final dpValve_nominal=0.1,
    riseTime=15)               "Valve to prevent reverse flow through pump"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
        origin={-70,-70})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear           val5(
    redeclare package Medium = Media.Water,
    final m_flow_nominal=mRad_flow_nominal/2,
    final dpValve_nominal=0.1,
    riseTime=15)               "Valve to prevent reverse flow through pump"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90,
        origin={10,-70})));
  Controls.OBC.CDL.Continuous.Hysteresis hys[2](uLow=fill(0.5, 2), uHigh=fill(
        0.8, 2)) "Check if pump isolation valve has opened"
    annotation (Placement(transformation(extent={{-280,0},{-260,20}})));
  Controls.OBC.CDL.Conversions.BooleanToReal           booToRea2
                                                               [2]
    "Boolean to Real conversion"
    annotation (Placement(transformation(extent={{-250,0},{-230,20}})));
  Controls.OBC.CDL.Interfaces.BooleanOutput yBoiSta[2] "Boiler status signal"
    annotation (Placement(transformation(extent={{320,-90},{360,-50}}),
        iconTransformation(extent={{100,-50},{140,-10}})));
  Controls.OBC.CDL.Continuous.Hysteresis hys2[2](uLow=fill(0.09, 2), uHigh=fill(
        0.1, 2))
            "Check if pumps are on"
    annotation (Placement(transformation(extent={{100,-20},{120,0}})));
  Controls.OBC.CDL.Logical.Timer tim1[2](t=fill(60, 2))
    "Check time for which pump status is on"
    annotation (Placement(transformation(extent={{140,-20},{160,0}})));
  Controls.OBC.CDL.Interfaces.BooleanOutput yPumSta[2] "Pump status signal"
    annotation (Placement(transformation(extent={{320,-130},{360,-90}}),
        iconTransformation(extent={{100,-80},{140,-40}})));
  Controls.OBC.CDL.Interfaces.RealOutput yHotWatIsoVal[2]
    "Measured boiler hot water isolation valve position" annotation (Placement(
        transformation(extent={{320,-170},{360,-130}}), iconTransformation(
          extent={{100,-110},{140,-70}})));
  Controls.OBC.CDL.Logical.Latch lat[2]
    "Hold pump enable status until change process is completed"
    annotation (Placement(transformation(extent={{-300,30},{-280,50}})));
  Controls.OBC.CDL.Logical.LogicalSwitch logSwi[2]
    "Switch to signal from controller once enabling process has been completed"
    annotation (Placement(transformation(extent={{-260,30},{-240,50}})));
  Controls.OBC.CDL.Logical.Pre pre[2] "Logical pre block"
    annotation (Placement(transformation(extent={{240,-140},{260,-120}})));
  Controls.OBC.CDL.Continuous.Hysteresis hys3[2](uLow=fill(0.9, 2), uHigh=fill(
        0.95, 2))                "Check for flow through boiler"
    annotation (Placement(transformation(extent={{120,-60},{140,-40}})));
  Controls.OBC.CDL.Continuous.Add add2[2](k1=fill(-1, 2))
    "Check difference between return temperature and boiler temperature"
    annotation (Placement(transformation(extent={{100,-100},{120,-80}})));
  Fluid.Sensors.TemperatureTwoPort           senTem2(redeclare package Medium =
        Media.Water, m_flow_nominal=mBoi_flow_nominal1)
    annotation (Placement(transformation(extent={{60,-160},{80,-140}})));
  Fluid.Sensors.TemperatureTwoPort           senTem3(redeclare package Medium =
        Media.Water, m_flow_nominal=mBoi_flow_nominal2)
    annotation (Placement(transformation(extent={{60,-220},{80,-200}})));
  Controls.OBC.CDL.Logical.LogicalSwitch logSwi1[2]
    "Switch to signal from controller once enabling process has been completed"
    annotation (Placement(transformation(extent={{-210,150},{-190,170}})));
  Controls.OBC.CDL.Logical.Latch lat1[2]
    "Hold pump enable status until change process is completed"
    annotation (Placement(transformation(extent={{-260,150},{-240,170}})));
  Controls.OBC.CDL.Logical.Pre pre1[2] "Logical pre block"
    annotation (Placement(transformation(extent={{-300,180},{-280,200}})));
  Controls.OBC.CDL.Interfaces.RealInput TBoiHotWatSupSet[2](
    final unit=fill("K", 2),
    displayUnit=fill("degC", 2),
    final quantity=fill("ThermodynamicTemperature", 2))
    "Boiler hot water supply temperature setpoint vector" annotation (Placement(
        transformation(extent={{-360,-150},{-320,-110}}), iconTransformation(
          extent={{-140,-100},{-100,-60}})));
  Controls.OBC.CDL.Continuous.PID conPID[2](
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    k=fill(10e-3, 2),
    Ti=fill(90, 2),
    yMax=fill(1, 2),
    yMin=fill(0.01, 2),
    xi_start=fill(1, 2))
    "PI controller for regulating hot water supply temperature from boiler"
    annotation (Placement(transformation(extent={{-240,-120},{-220,-100}})));
  Controls.OBC.CDL.Continuous.Product pro1[2]
    "Product of boiler power and current status"
    annotation (Placement(transformation(extent={{-120,-120},{-100,-100}})));
  Controls.OBC.CDL.Continuous.Add add1[2](k1=fill(-1, 2))
    "Find difference between setpoint and measured temperature"
    annotation (Placement(transformation(extent={{-290,-170},{-270,-150}})));
  Controls.OBC.CDL.Logical.Switch swi[2] "Switch"
    annotation (Placement(transformation(extent={{-90,-170},{-70,-150}})));
  Fluid.FixedResistances.Junction           spl6(
    redeclare package Medium = MediumW,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal={-mBoi_flow_nominal1,mRad_flow_nominal,-
        mBoi_flow_nominal2},
    final dp_nominal={0,0,-200})
    "Splitter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=0,
      origin={150,-150})));
  Controls.OBC.CDL.Continuous.GreaterThreshold greThr[2](h=0.3)
    "Check if supply temperature setpoint is not met"
    annotation (Placement(transformation(extent={{-260,-170},{-240,-150}})));
  Controls.OBC.CDL.Routing.BooleanReplicator booRep(nout=2)
    "Boolean replicator"
    annotation (Placement(transformation(extent={{-160,-170},{-140,-150}})));
  Fluid.Sensors.RelativePressure           senRelPre1(redeclare package Medium =
        Media.Water)
    "Differential pressure sensor between hot water supply and return"
    annotation (Placement(transformation(extent={{-90,-4},{-70,16}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        MediumW) annotation (Placement(transformation(extent={{30,230},{50,250}}),
                     iconTransformation(extent={{50,90},{70,110}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        MediumW) annotation (Placement(transformation(extent={{-50,230},{-30,250}}),
                     iconTransformation(extent={{-70,90},{-50,110}})));
  Controls.OBC.CDL.Interfaces.RealOutput yBypValPos(
    final unit="1",
    displayUnit="1")
    "Measured bypass valve position"
    annotation (Placement(transformation(extent={{320,130},{360,170}}),
        iconTransformation(extent={{100,100},{140,140}})));
  Controls.OBC.CDL.Logical.Timer tim2
                                    [2](t=fill(5, 2))
    "Check time for which boiler status on"
    annotation (Placement(transformation(extent={{160,-100},{180,-80}})));
  Controls.OBC.CDL.Continuous.GreaterThreshold greThr1[2](t=fill(0, 2))
    annotation (Placement(transformation(extent={{130,-100},{150,-80}})));
  Controls.OBC.CDL.Continuous.LessThreshold lesThr[2](t=fill(0.01, 2))
    annotation (Placement(transformation(extent={{130,-130},{150,-110}})));
  Controls.OBC.CDL.Logical.Pre pre2
                                  [2] "Logical pre block"
    annotation (Placement(transformation(extent={{240,-100},{260,-80}})));
  Controls.OBC.CDL.Logical.TrueDelay truDel[2](delayTime=60) "True delay"
    annotation (Placement(transformation(extent={{160,-60},{180,-40}})));
  Controls.OBC.CDL.Logical.Latch lat2[2] "Latch"
    annotation (Placement(transformation(extent={{280,-80},{300,-60}})));
  Controls.OBC.CDL.Logical.And and2[2] "Logical And"
    annotation (Placement(transformation(extent={{240,-70},{260,-50}})));
  Controls.OBC.CDL.Continuous.Product pro2[2]
    "Product of boiler power and current status"
    annotation (Placement(transformation(extent={{-280,-120},{-260,-100}})));
  Controls.OBC.CDL.Logical.Latch lat3
    annotation (Placement(transformation(extent={{-190,-170},{-170,-150}})));
  Controls.OBC.CDL.Logical.Edge edg[2]
    annotation (Placement(transformation(extent={{-280,110},{-260,130}})));
  Controls.OBC.CDL.Logical.MultiOr mulOr1(nu=2)
    annotation (Placement(transformation(extent={{-240,110},{-220,130}})));
  Controls.OBC.CDL.Logical.MultiAnd mulAnd(nu=2)
    annotation (Placement(transformation(extent={{-230,-170},{-210,-150}})));
  Controls.OBC.CDL.Continuous.GreaterThreshold greThr2[2](t=273.15 + 95)
    annotation (Placement(transformation(extent={{-230,-214},{-210,-194}})));
  Controls.OBC.CDL.Logical.MultiOr mulOr(nu=2)
    annotation (Placement(transformation(extent={{-200,-214},{-180,-194}})));
  Controls.OBC.CDL.Logical.Or or2
    annotation (Placement(transformation(extent={{-160,-214},{-140,-194}})));
equation
  connect(spl1.port_2, spl2.port_1)
    annotation (Line(points={{-30,-140},{-30,-110}},   color={0,127,255}));
  connect(spl4.port_3, val.port_a)
    annotation (Line(points={{-20,40},{80,40}},     color={0,127,255}));
  connect(val.port_b, spl5.port_3)
    annotation (Line(points={{100,40},{200,40}},  color={0,127,255}));
  connect(val2.port_a, spl1.port_3)
    annotation (Line(points={{0,-150},{-20,-150}},    color={0,127,255}));
  connect(val1.port_a, spl1.port_1) annotation (Line(points={{0,-210},{-30,-210},
          {-30,-160}},        color={0,127,255}));

  connect(uBypValSig, val.y) annotation (Line(points={{-340,-40},{-120,-40},{
          -120,64},{90,64},{90,52}},  color={0,0,127}));

  connect(uHotIsoVal[1], val1.y) annotation (Line(points={{-340,70},{-340,80},{
          -170,80},{-170,-130},{-10,-130},{-10,-180},{10,-180},{10,-198}},
                                                         color={0,0,127}));

  connect(uHotIsoVal[2], val2.y) annotation (Line(points={{-340,90},{-340,80},{
          -170,80},{-170,-130},{10,-130},{10,-138}},     color={0,0,127}));

  connect(spl4.port_2, senRelPre.port_a) annotation (Line(points={{-30,50},{-30,
          80},{80,80}},    color={0,127,255}));

  connect(senRelPre.port_b, spl5.port_1) annotation (Line(points={{100,80},{210,
          80},{210,50}},   color={0,127,255}));

  connect(spl3.port_2, senVolFlo.port_a) annotation (Line(points={{-30,10},{-30,
          16},{30,16},{30,0},{40,0}},              color={0,127,255}));

  connect(senVolFlo.port_b, spl4.port_1) annotation (Line(points={{60,0},{70,0},
          {70,20},{-30,20},{-30,30}},                color={0,127,255}));

  connect(senVolFlo.V_flow, VHotWat_flow) annotation (Line(points={{50,11},{50,16},
          {80,16},{80,-30},{340,-30}},             color={0,0,127}));

  connect(spl4.port_2, senTem.port_a) annotation (Line(points={{-30,50},{-30,120},
          {-20,120}},       color={0,127,255}));

  connect(senTem.T, ySupTem) annotation (Line(points={{-10,131},{-10,140},{270,
          140},{270,100},{340,100}},                color={0,0,127}));

  connect(senTem1.port_b, spl5.port_1) annotation (Line(points={{200,120},{210,
          120},{210,50}},  color={0,127,255}));

  connect(senTem1.T, yRetTem) annotation (Line(points={{190,131},{190,134},{260,
          134},{260,60},{340,60}},
                                color={0,0,127}));

  connect(reaRep.y, yHotWatDp)
    annotation (Line(points={{302,10},{340,10}},     color={0,0,127}));

  connect(senRelPre.p_rel, reaRep.u) annotation (Line(points={{90,71},{90,66},{240,
          66},{240,10},{278,10}},           color={0,0,127}));

  connect(pum.port_b, spl3.port_1) annotation (Line(points={{-70,-30},{-70,-20},
          {-30,-20},{-30,-10}}, color={0,127,255}));
  connect(pum1.port_b, spl3.port_3)
    annotation (Line(points={{10,-30},{10,0},{-20,0}}, color={0,127,255}));
  connect(spl2.port_2, val4.port_a) annotation (Line(points={{-30,-90},{-30,-86},
          {-70,-86},{-70,-80}}, color={0,127,255}));
  connect(val4.port_b, pum.port_a)
    annotation (Line(points={{-70,-60},{-70,-50}}, color={0,127,255}));
  connect(val5.port_b, pum1.port_a)
    annotation (Line(points={{10,-60},{10,-50}}, color={0,127,255}));
  connect(booToRea[1].y, val4.y) annotation (Line(points={{-198,40},{-100,40},{
          -100,-70},{-82,-70}}, color={0,0,127}));
  connect(booToRea[2].y, val5.y) annotation (Line(points={{-198,40},{-100,40},{
          -100,-16},{-20,-16},{-20,-70},{-2,-70}}, color={0,0,127}));
  connect(val4.y_actual, hys[1].u) annotation (Line(points={{-77,-65},{-77,-54},
          {-300,-54},{-300,10},{-282,10}}, color={0,0,127}));
  connect(val5.y_actual, hys[2].u) annotation (Line(points={{3,-65},{3,-54},{
          -300,-54},{-300,10},{-282,10}}, color={0,0,127}));
  connect(hys.y, booToRea2.u)
    annotation (Line(points={{-258,10},{-252,10}}, color={255,0,255}));
  connect(booToRea2.y, pro.u1) annotation (Line(points={{-228,10},{-220,10},{
          -220,-4},{-212,-4}}, color={0,0,127}));
  connect(pro[1].y, pum.y) annotation (Line(points={{-188,-10},{-110,-10},{-110,
          -40},{-82,-40}}, color={0,0,127}));
  connect(pro[2].y, pum1.y) annotation (Line(points={{-188,-10},{-40,-10},{-40,
          -40},{-2,-40}}, color={0,0,127}));
  connect(hys2.y, tim1.u)
    annotation (Line(points={{122,-10},{138,-10}}, color={255,0,255}));
  connect(tim1.passed, yPumSta) annotation (Line(points={{162,-18},{230,-18},{
          230,-110},{340,-110}},
                             color={255,0,255}));
  connect(yHotWatIsoVal[1], val1.y_actual) annotation (Line(points={{340,-160},
          {280,-160},{280,-174},{30,-174},{30,-203},{15,-203}},color={0,0,127}));
  connect(yHotWatIsoVal[2], val2.y_actual) annotation (Line(points={{340,-140},
          {280,-140},{280,-174},{30,-174},{30,-143},{15,-143}},color={0,0,127}));
  connect(spl2.port_3, val5.port_a) annotation (Line(points={{-20,-100},{10,-100},
          {10,-80}}, color={0,127,255}));
  connect(uPumSpe, pro[1].u2) annotation (Line(points={{-340,0},{-288,0},{-288,
          -16},{-212,-16}}, color={0,0,127}));
  connect(uPumSpe, pro[2].u2) annotation (Line(points={{-340,0},{-292,0},{-292,
          -16},{-212,-16}}, color={0,0,127}));
  connect(uPumSta, lat.u)
    annotation (Line(points={{-340,40},{-302,40}}, color={255,0,255}));
  connect(lat.y, logSwi.u2)
    annotation (Line(points={{-278,40},{-262,40}}, color={255,0,255}));
  connect(lat.y, logSwi.u1) annotation (Line(points={{-278,40},{-270,40},{-270,
          48},{-262,48}}, color={255,0,255}));
  connect(logSwi.y, booToRea.u)
    annotation (Line(points={{-238,40},{-222,40}}, color={255,0,255}));
  connect(uPumSta, logSwi.u3) annotation (Line(points={{-340,40},{-310,40},{
          -310,28},{-270,28},{-270,32},{-262,32}}, color={255,0,255}));
  connect(tim1.passed, pre.u) annotation (Line(points={{162,-18},{230,-18},{230,
          -130},{238,-130}}, color={255,0,255}));
  connect(pre.y, lat.clr) annotation (Line(points={{262,-130},{272,-130},{272,
          -220},{-314,-220},{-314,34},{-302,34}}, color={255,0,255}));
  connect(pum.y_actual, hys2[1].u) annotation (Line(points={{-77,-29},{-77,-24},
          {90,-24},{90,-10},{98,-10}}, color={0,0,127}));
  connect(pum1.y_actual, hys2[2].u) annotation (Line(points={{3,-29},{3,-24},{
          90,-24},{90,-10},{98,-10}}, color={0,0,127}));
  connect(senTem2.port_b, boi.port_b)
    annotation (Line(points={{80,-150},{90,-150}}, color={0,127,255}));
  connect(senTem3.port_b, boi1.port_b)
    annotation (Line(points={{80,-210},{90,-210}}, color={0,127,255}));
  connect(uBoiSta, lat1.u)
    annotation (Line(points={{-340,160},{-262,160}}, color={255,0,255}));
  connect(lat1.y, logSwi1.u2)
    annotation (Line(points={{-238,160},{-212,160}}, color={255,0,255}));
  connect(logSwi1.y, booToRea1.u)
    annotation (Line(points={{-188,160},{-162,160}}, color={255,0,255}));
  connect(uBoiSta, logSwi1.u3) annotation (Line(points={{-340,160},{-310,160},{
          -310,140},{-220,140},{-220,152},{-212,152}}, color={255,0,255}));
  connect(lat1.y, logSwi1.u1) annotation (Line(points={{-238,160},{-220,160},{
          -220,168},{-212,168}}, color={255,0,255}));
  connect(pre1.y, lat1.clr) annotation (Line(points={{-278,190},{-270,190},{
          -270,154},{-262,154}}, color={255,0,255}));
  connect(senTem3.T, add2[1].u1) annotation (Line(points={{70,-199},{70,-188},{
          56,-188},{56,-84},{98,-84}},            color={0,0,127}));
  connect(senTem2.T, add2[2].u1)
    annotation (Line(points={{70,-139},{70,-84},{98,-84}},  color={0,0,127}));
  connect(senTem3.T, conPID[1].u_m) annotation (Line(points={{70,-199},{70,-188},
          {-308,-188},{-308,-126},{-230,-126},{-230,-122}},
                                              color={0,0,127}));
  connect(senTem2.T, conPID[2].u_m) annotation (Line(points={{70,-139},{70,-126},
          {-230,-126},{-230,-122}}, color={0,0,127}));
  connect(conPID.y, pro1.u2) annotation (Line(points={{-218,-110},{-160,-110},{
          -160,-116},{-122,-116}},
                              color={0,0,127}));
  connect(booToRea1.y, pro1.u1) annotation (Line(points={{-138,160},{-130,160},
          {-130,-104},{-122,-104}},color={0,0,127}));
  connect(senTem3.T, add1[1].u2) annotation (Line(points={{70,-199},{70,-188},{
          -308,-188},{-308,-166},{-292,-166}},        color={0,0,127}));
  connect(senTem2.T, add1[2].u2) annotation (Line(points={{70,-139},{70,-126},{
          -308,-126},{-308,-166},{-292,-166}},
                                          color={0,0,127}));
  connect(boi.port_a, spl6.port_1)
    annotation (Line(points={{110,-150},{140,-150}}, color={0,127,255}));
  connect(boi1.port_a, spl6.port_3) annotation (Line(points={{110,-210},{150,
          -210},{150,-160}}, color={0,127,255}));
  connect(spl6.port_2, spl5.port_2) annotation (Line(points={{160,-150},{210,
          -150},{210,30}}, color={0,127,255}));
  connect(pro1.y, swi.u3) annotation (Line(points={{-98,-110},{-96,-110},{-96,
          -168},{-92,-168}}, color={0,0,127}));
  connect(booToRea1.y, swi.u1) annotation (Line(points={{-138,160},{-130,160},{
          -130,-152},{-92,-152}}, color={0,0,127}));
  connect(add1.y, greThr.u)
    annotation (Line(points={{-268,-160},{-262,-160}}, color={0,0,127}));
  connect(booRep.y, swi.u2)
    annotation (Line(points={{-138,-160},{-92,-160}}, color={255,0,255}));
  connect(swi[1].y, boi1.y) annotation (Line(points={{-68,-160},{-60,-160},{-60,
          -120},{120,-120},{120,-202},{112,-202}}, color={0,0,127}));
  connect(swi[2].y, boi.y) annotation (Line(points={{-68,-160},{-60,-160},{-60,
          -120},{120,-120},{120,-142},{112,-142}}, color={0,0,127}));
  connect(spl6.port_2, preSou.ports[1]) annotation (Line(points={{160,-150},{
          210,-150},{210,-200},{240,-200}}, color={0,127,255}));
  connect(senRelPre1.port_b, spl3.port_2) annotation (Line(points={{-70,6},{-50,
          6},{-50,10},{-30,10}}, color={0,127,255}));
  connect(senRelPre1.port_a, spl2.port_2) annotation (Line(points={{-90,6},{
          -104,6},{-104,-90},{-30,-90}}, color={0,127,255}));
  connect(senTem.port_b, port_b) annotation (Line(points={{0,120},{20,120},{20,200},
          {-40,200},{-40,240}}, color={0,127,255}));
  connect(senTem1.port_a, port_a)
    annotation (Line(points={{180,120},{40,120},{40,240}}, color={0,127,255}));
  connect(val.y_actual, yBypValPos) annotation (Line(points={{95,47},{120,47},{120,
          150},{340,150}}, color={0,0,127}));
  connect(senTem2.port_a, val2.port_b)
    annotation (Line(points={{60,-150},{20,-150}}, color={0,127,255}));
  connect(senTem3.port_a, val1.port_b)
    annotation (Line(points={{60,-210},{20,-210}}, color={0,127,255}));
  connect(val1.y_actual, hys3[1].u) annotation (Line(points={{15,-203},{30,-203},
          {30,-50},{118,-50}}, color={0,0,127}));
  connect(val2.y_actual, hys3[2].u) annotation (Line(points={{15,-143},{30,-143},
          {30,-50},{118,-50}}, color={0,0,127}));
  connect(boi1.T, add2[1].u2) annotation (Line(points={{89,-202},{84,-202},{84,
          -96},{98,-96}}, color={0,0,127}));
  connect(boi.T, add2[2].u2) annotation (Line(points={{89,-142},{84,-142},{84,
          -96},{98,-96}}, color={0,0,127}));
  connect(greThr1.y, tim2.u)
    annotation (Line(points={{152,-90},{158,-90}}, color={255,0,255}));
  connect(add2.y, greThr1.u)
    annotation (Line(points={{122,-90},{128,-90}}, color={0,0,127}));
  connect(swi.y, lesThr.u) annotation (Line(points={{-68,-160},{-60,-160},{-60,
          -120},{128,-120}}, color={0,0,127}));
  connect(hys3.y, truDel.u)
    annotation (Line(points={{142,-50},{158,-50}},
                                                 color={255,0,255}));
  connect(pre2.y, lat2.clr) annotation (Line(points={{262,-90},{270,-90},{270,
          -76},{278,-76}}, color={255,0,255}));
  connect(truDel.y, and2.u1)
    annotation (Line(points={{182,-50},{200,-50},{200,-60},{238,-60}},
                                                   color={255,0,255}));
  connect(tim2.passed, and2.u2) annotation (Line(points={{182,-98},{200,-98},{
          200,-68},{238,-68}}, color={255,0,255}));
  connect(and2.y, lat2.u) annotation (Line(points={{262,-60},{270,-60},{270,-70},
          {278,-70}}, color={255,0,255}));
  connect(lat2.y, yBoiSta)
    annotation (Line(points={{302,-70},{340,-70}}, color={255,0,255}));
  connect(lat2.y, pre1.u) annotation (Line(points={{302,-70},{310,-70},{310,220},
          {-310,220},{-310,190},{-302,190}}, color={255,0,255}));
  connect(lesThr.y, pre2.u) annotation (Line(points={{152,-120},{220,-120},{220,
          -90},{238,-90}}, color={255,0,255}));
  connect(TBoiHotWatSupSet, pro2.u2) annotation (Line(points={{-340,-130},{-300,
          -130},{-300,-116},{-282,-116}}, color={0,0,127}));
  connect(pro2.y, conPID.u_s)
    annotation (Line(points={{-258,-110},{-242,-110}}, color={0,0,127}));
  connect(booToRea1.y, pro2.u1) annotation (Line(points={{-138,160},{-130,160},
          {-130,-90},{-300,-90},{-300,-104},{-282,-104}}, color={0,0,127}));
  connect(booRep.u, lat3.y)
    annotation (Line(points={{-162,-160},{-168,-160}}, color={255,0,255}));
  connect(uBoiSta, edg.u) annotation (Line(points={{-340,160},{-310,160},{-310,
          120},{-282,120}}, color={255,0,255}));
  connect(edg.y, mulOr1.u[1:2]) annotation (Line(points={{-258,120},{-250,120},
          {-250,116.5},{-242,116.5}}, color={255,0,255}));
  connect(mulOr1.y, lat3.u) annotation (Line(points={{-218,120},{-180,120},{
          -180,-140},{-196,-140},{-196,-160},{-192,-160}}, color={255,0,255}));
  connect(greThr.y, mulAnd.u[1:2]) annotation (Line(points={{-238,-160},{-236,
          -160},{-236,-163.5},{-232,-163.5}}, color={255,0,255}));
  connect(senTem3.T, greThr2[1].u) annotation (Line(points={{70,-199},{70,-188},
          {-240,-188},{-240,-204},{-232,-204}}, color={0,0,127}));
  connect(senTem2.T, greThr2[2].u) annotation (Line(points={{70,-139},{70,-126},
          {-308,-126},{-308,-188},{-240,-188},{-240,-204},{-232,-204}}, color={
          0,0,127}));
  connect(greThr2.y, mulOr.u[1:2]) annotation (Line(points={{-208,-204},{-206,
          -204},{-206,-207.5},{-202,-207.5}}, color={255,0,255}));
  connect(mulOr.y, or2.u1)
    annotation (Line(points={{-178,-204},{-162,-204}}, color={255,0,255}));
  connect(mulAnd.y, or2.u2) annotation (Line(points={{-208,-160},{-204,-160},{
          -204,-184},{-170,-184},{-170,-212},{-162,-212}}, color={255,0,255}));
  connect(or2.y, lat3.clr) annotation (Line(points={{-138,-204},{-130,-204},{
          -130,-180},{-196,-180},{-196,-166},{-192,-166}}, color={255,0,255}));
  connect(pro2.y, add1.u1) annotation (Line(points={{-258,-110},{-250,-110},{
          -250,-140},{-300,-140},{-300,-154},{-292,-154}}, color={0,0,127}));
  annotation (defaultComponentName="boiPla",
    Documentation(info="<html>
      <p>
      This model implements a primary-only, condensing boiler plant with headered, 
      variable-speed primary pumps, as defined in ASHRAE RP-1711, March 2020 draft.
      </p>
      </html>", revisions="<html>
      <ul>
      <li>
      October 28, 2020, by Karthik Devaprasad:<br/>
      First implementation.
      </li>
      </ul>
      </html>"),
    Diagram(coordinateSystem(preserveAspectRatio=false,
      extent={{-320,-240},{320,240}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
      graphics={
        Text(
          extent={{-100,160},{100,120}},
          lineColor={0,0,255},
          textString="%name")}),
    experiment(
      StopTime=10500,
      Interval=60,
      __Dymola_Algorithm="Cvode"));
end BoilerPlant_Buffalo;