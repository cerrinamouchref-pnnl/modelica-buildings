within Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChilledBeamSystem.Testbed;
block TestBed_v2

  replaceable package MediumA = Buildings.Media.Air
    "Medium model";

  replaceable package MediumW = Buildings.Media.Water
    "Medium model";

  parameter Real hRoo=2.74
    "Height of room";

  parameter Modelica.SIunits.PressureDifference dpValve_nominal=6000
    "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint";

  parameter Modelica.SIunits.PressureDifference dpFixed_nominal=0
    "Pressure drop of pipe and other resistances that are in series";

  parameter Real TChiWatSup_nominal = 273.15 + 15.56
    "Nominal chilled water supply temperature into zone";

  parameter Real TChiWatRet_nominal = 273.15 + 19
    "Nominal chilled water return temperature from zone";

  parameter Real mChiWatTot_flow_nominal = mChiWatSou_flow_nominal +
                                           mChiWatEas_flow_nominal +
                                           mChiWatNor_flow_nominal +
                                           mChiWatWes_flow_nominal +
                                           mChiWatCor_flow_nominal
    "Total nominal chilled water flow rate through all five zones";

  parameter Real mAirTot_flow_nominal = mAirSou_flow_nominal +
                                        mAirEas_flow_nominal +
                                        mAirNor_flow_nominal +
                                        mAirWes_flow_nominal +
                                        mAirCor_flow_nominal
    "Total nominal air flow rate through all five zones";

  parameter Real mHotWatCoi_nominal = 96
    "Hot water mass flow rate through AHU heating coil";

  parameter Real mChiWatCoi_nominal = 96
    "Chilled water mass flow rate through AHU cooling coil";

  // South zone
  parameter Real QSou_flow_nominal = -100000
    "Nominal heat flow into south zone"
    annotation(Dialog(group="South zone"));

  parameter Real VRooSou = 500
    "Volume of zone air in south zone"
    annotation(Dialog(group="South zone"));

  parameter Real mChiWatSou_flow_nominal = 0.2
    "Chilled water volume flow rate into zone"
    annotation(Dialog(group="South zone"));

  parameter Real mAirSou_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="South zone"));

  parameter Real mAChiBeaSou_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="South zone"));

  // East zone
  parameter Real QEas_flow_nominal = -100000
    "Nominal heat flow into east zone"
    annotation(Dialog(group="East zone"));

  parameter Real VRooEas = 500
    "Volume of zone air in east zone"
    annotation(Dialog(group="East zone"));

  parameter Real mChiWatEas_flow_nominal = 0.2
    "Chilled water volume flow rate into zone"
    annotation(Dialog(group="East zone"));

  parameter Real mAirEas_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="East zone"));

  parameter Real mAChiBeaEas_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="East zone"));

  // North zone
  parameter Real QNor_flow_nominal = -100000
    "Nominal heat flow into north zone"
    annotation(Dialog(group="North zone"));

  parameter Real VRooNor = 500
    "Volume of zone air in north zone"
    annotation(Dialog(group="North zone"));

  parameter Real mChiWatNor_flow_nominal = 0.2
    "Chilled water volume flow rate into zone"
    annotation(Dialog(group="North zone"));

  parameter Real mAirNor_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="North zone"));

  parameter Real mAChiBeaNor_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="North zone"));

  // West zone
  parameter Real QWes_flow_nominal = -100000
    "Nominal heat flow into west zone"
    annotation(Dialog(group="West zone"));

  parameter Real VRooWes = 500
    "Volume of zone air in west zone"
    annotation(Dialog(group="West zone"));

  parameter Real mChiWatWes_flow_nominal = 0.2
    "Chilled water volume flow rate into zone"
    annotation(Dialog(group="West zone"));

  parameter Real mAirWes_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="West zone"));

  parameter Real mAChiBeaWes_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="West zone"));

  // Core zone
  parameter Real QCor_flow_nominal = -100000
    "Nominal heat flow into core zone"
    annotation(Dialog(group="Core zone"));

  parameter Real VRooCor = 500
    "Volume of zone air in core zone"
    annotation(Dialog(group="Core zone"));

  parameter Real mChiWatCor_flow_nominal = 0.2
    "Chilled water volume flow rate into zone"
    annotation(Dialog(group="Core zone"));

  parameter Real mAirCor_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="Core zone"));

  parameter Real mAChiBeaCor_flow_nominal = 0.2
    "Discharge air volume flow rate into zone"
    annotation(Dialog(group="Core zone"));

  Fluid.Movers.SpeedControlled_y pum(redeclare package Medium = MediumW,
    redeclare Fluid.Movers.Data.Pumps.Wilo.chilledBeamPumpCurve per,
      addPowerToMedium=false)
    "Chilled water pump"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90,
        origin={100,-130})));

  Fluid.FixedResistances.Junction jun(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,-
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={100,-40})));

  Fluid.FixedResistances.Junction jun3(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = -90,
        origin={250,-40})));

  Fluid.Actuators.Valves.TwoWayLinear val(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatSou_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{120,-50},{140,-30}})));

  BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        ModelicaServices.ExternalReferences.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_AZ_Phoenix-Sky.Harbor.Intl.AP.722780_TMY3.mos"))
    annotation (Placement(transformation(extent={{-310,-20},{-290,0}})));

  BoundaryConditions.WeatherData.Bus weaBus "Weather Data Bus"
    annotation (Placement(transformation(extent={{-280,-20},{-260,0}}),
        iconTransformation(extent={{-360,170},{-340,190}})));

  Fluid.Sources.Outside           amb(redeclare package Medium = MediumA,
      nPorts=3)
    "Ambient conditions"
    annotation (Placement(transformation(extent={{-250,-20},{-228,2}})));

  Fluid.HeatExchangers.WetCoilCounterFlow           cooCoi(
    show_T=true,
    UA_nominal=3*mAirTot_flow_nominal*1000*15/
        Buildings.Fluid.HeatExchangers.BaseClasses.lmtd(
        T_a1=26.2,
        T_b1=12.8,
        T_a2=6,
        T_b2=16),
    redeclare package Medium1 = MediumW,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mAirTot_flow_nominal*1000*15/4200/10,
    m2_flow_nominal=mAirTot_flow_nominal,
    dp2_nominal=0,
    dp1_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    allowFlowReversal1=true,
    allowFlowReversal2=true)
    "Cooling coil"
    annotation (Placement(transformation(extent={{-110,4},{-130,-16}})));

  Fluid.Sources.Boundary_pT           sinHea(
    redeclare package Medium = MediumW,
    p=100000,
    T=318.15,
    nPorts=1)
    "Sink for heating coil" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-210,-70})));

  Fluid.Sources.Boundary_pT           sinCoo(
    redeclare package Medium = MediumW,
    p=100000,
    T=285.15,
    nPorts=1)
    "Sink for cooling coil" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-140,-70})));

  Fluid.Sources.MassFlowSource_T           souCoo(
    redeclare package Medium = MediumW,
    T=280.372,
    nPorts=1,
    use_m_flow_in=true)
    "Source for cooling coil" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-100,-70})));

  Fluid.Sources.MassFlowSource_T           souHea(
    redeclare package Medium = MediumW,
    T=318.15,
    use_m_flow_in=true,
    nPorts=1)           "Source for heating coil" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-170,-70})));

  Fluid.HeatExchangers.DryCoilEffectivenessNTU           heaCoi(
    show_T=true,
    redeclare package Medium1 = MediumW,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mAirTot_flow_nominal*1000*(10 - (-20))/4200/10,
    m2_flow_nominal=mAirTot_flow_nominal,
    configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    Q_flow_nominal=mAirTot_flow_nominal*1006*(16.7 - 8.5),
    dp1_nominal=0,
    dp2_nominal=200 + 200 + 100 + 40,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    T_a1_nominal=318.15,
    T_a2_nominal=281.65)
    "Heating coil"
    annotation (Placement(transformation(extent={{-180,4},{-200,-16}})));

  Examples.VAVReheat.ThermalZones.VAVBranch souCAVTer(
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    m_flow_nominal=mAirSou_flow_nominal,
    VRoo=VRooSou,
    allowFlowReversal=false)
    "Zone for core of buildings (azimuth will be neglected)"
    annotation (Placement(transformation(extent={{390,-130},{430,-90}})));

  Fluid.FixedResistances.Junction jun5(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-40,150},{-20,170}})));

  Fluid.Sources.Boundary_pT           sinCoo1(
    redeclare package Medium = MediumW,
    p=100000,
    T=297.04,
    nPorts=2)
    "Sink for chillede water"         annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={250,-176})));

  Fluid.Sources.Boundary_pT           souCoo1(
    redeclare package Medium = MediumW,
    p=100000,
    use_T_in=true,
    T=285.15,
    nPorts=1)
    "Source for chilled water to chilled beam manifolds"
                                      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={100,-174})));

  Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumA,
      m_flow_nominal=mAirTot_flow_nominal)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));

  Fluid.Movers.SpeedControlled_y fan(redeclare package Medium = MediumA,
    redeclare Fluid.Movers.Data.Pumps.Wilo.chilledBeamFanCurve per,
      addPowerToMedium=false)
    "DOAS fan" annotation (Placement(
        transformation(
        extent={{-80,-10},{-60,10}})));

  Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium = MediumW)
    annotation (Placement(transformation(extent={{150,-80},{170,-60}})));

  Fluid.Sensors.RelativePressure senRelPre1(redeclare package Medium = MediumA)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90,
        origin={-50,30})));

  Modelica.Blocks.Routing.DeMultiplex demux(n=5)
    annotation (Placement(transformation(extent={{40,-30},{60,-10}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput dPChiWat
    "Measured chilled water differential presure"
    annotation (Placement(transformation(extent={{580,-110},{620,-70}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yPumSta
    "Pump proven on"
    annotation (Placement(transformation(extent={{580,-140},{620,-100}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys(uLow=0.04, uHigh=0.05)
    annotation (Placement(transformation(extent={{140,-120},{160,-100}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPumSta
    "Pump enable signal"
    annotation (Placement(transformation(extent={{-380,-130},{-340,-90}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uPumSpe
    "Pump speed signal"
    annotation (Placement(transformation(extent={{-380,-160},{-340,-120}})));

  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
    annotation (Placement(transformation(extent={{-320,-120},{-300,-100}})));

  Buildings.Controls.OBC.CDL.Continuous.Product pro
    annotation (Placement(transformation(extent={{-280,-140},{-260,-120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSup
    "Chilled water supply temperature"
    annotation (Placement(transformation(extent={{-380,-200},{-340,-160}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uHeaCoi
    "Heating coil control signal"
    annotation (Placement(transformation(extent={{-380,-70},{-340,-30}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCooCoi
    "AHU cooling coil control signal"
    annotation (Placement(transformation(extent={{-380,-100},{-340,-60}})));

  Buildings.Controls.OBC.CDL.Continuous.Gain gai(k=mHotWatCoi_nominal)
    annotation (Placement(transformation(extent={{-320,-60},{-300,-40}})));

  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=mChiWatCoi_nominal)
    annotation (Placement(transformation(extent={{-320,-90},{-300,-70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCAVDam[5]
    "CAV damper signal"
    annotation (Placement(transformation(extent={{-380,120},{-340,160}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCAVReh[5]
    "CAV reheat signal"
    annotation (Placement(transformation(extent={{-380,80},{-340,120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uBypValPos
    "Bypass valve position signal"
    annotation (Placement(transformation(extent={{-380,200},{-340,240}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uChiWatVal[5]
    "Chilled water valve position signal"
    annotation (Placement(transformation(extent={{-380,160},{-340,200}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput dPDOASAir
    "Measured airloop differential presure"
    annotation (Placement(transformation(extent={{580,-40},{620,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TDOASDis(
    final unit = "K",
    displayUnit="K",
    final quantity = "ThermodynamicTemperature")
    "Measured DOAS discharge air temperature"
    annotation (Placement(transformation(extent={{580,-170},{620,-130}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yChiWatVal[5]
    "Measured chilled water valve position"
    annotation (Placement(transformation(extent={{580,60},{620,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yBypValPos
    "Measured bypass valve position"
    annotation (Placement(transformation(extent={{584,102},{624,142}}),
        iconTransformation(extent={{584,102},{624,142}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSta
    "Supply fan enable signal"
    annotation (Placement(transformation(extent={{-380,20},{-340,60}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uFanSpe
    "Fan speed signal"
    annotation (Placement(transformation(extent={{-380,-10},{-340,30}})));

  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1
    annotation (Placement(transformation(extent={{-320,30},{-300,50}})));

  Buildings.Controls.OBC.CDL.Continuous.Product pro1
    annotation (Placement(transformation(extent={{-280,10},{-260,30}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yFanSta
    "Supply fan proven on"
    annotation (Placement(transformation(extent={{580,-70},{620,-30}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys1(uLow=0.04, uHigh=0.05)
    annotation (Placement(transformation(extent={{-40,-210},{-20,-190}})));

  Modelica.Blocks.Routing.Multiplex mux1(n=5)
    annotation (Placement(transformation(extent={{160,70},{180,90}})));

  Modelica.Blocks.Routing.DeMultiplex demux1(n=5)
    annotation (Placement(transformation(extent={{300,10},{320,30}})));

  Modelica.Blocks.Routing.DeMultiplex demux2(n=5)
    annotation (Placement(transformation(extent={{300,-50},{320,-30}})));

  Modelica.Blocks.Routing.Multiplex mux3(n=5)
    annotation (Placement(transformation(extent={{460,-20},{480,0}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yDamPos[5] "Measured CAV damper position"
    annotation (Placement(transformation(extent={{580,-10},{620,30}})));
  Fluid.Sensors.RelativeHumidityTwoPort senRelHum(redeclare package Medium =
        MediumA, m_flow_nominal=mAirTot_flow_nominal)
    "Relative humidity sensor"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180,
        origin={-94,168})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput relHumDOASRet
    "Measured DOAS return air relative humidity"
    annotation (Placement(transformation(extent={{582,-196},{622,-156}})));
  ZoneModel_simplified_v2 nor(
    nConExt=0,
    nConExtWin=0,
    nConPar=0,
    nConBou=3,
    nSurBou=0,
    datConBou(
      layers={conIntWal,conIntWal,conIntWal},
      A={6.47,40.76,6.47}*hRoo,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    Q_flow_nominal=QNor_flow_nominal,
    TRadSup_nominal=TChiWatSup_nominal,
    TRadRet_nominal=TChiWatRet_nominal,
    mRad_flow_nominal=mChiWatNor_flow_nominal,
    V=VRooNor,
    TAir_nominal=297.04,
    mA_flow_nominal=mAirNor_flow_nominal,
    mAChiBea_flow_nominal=mAChiBeaNor_flow_nominal)
    annotation (Placement(transformation(extent={{180,300},{200,320}})));

  ZoneModel_simplified_v2 wes(
    nConExt=0,
    nConExtWin=0,
    nConPar=0,
    nConBou=3,
    nSurBou=0,
    datConBou(
      layers={conIntWal,conIntWal,conIntWal},
      A={6.47,40.76,6.47}*hRoo,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    Q_flow_nominal=QWes_flow_nominal,
    TRadSup_nominal=TChiWatSup_nominal,
    TRadRet_nominal=TChiWatRet_nominal,
    mRad_flow_nominal=mChiWatWes_flow_nominal,
    V=VRooWes,
    TAir_nominal=297.04,
    mA_flow_nominal=mAirWes_flow_nominal,
    mAChiBea_flow_nominal=mAChiBeaWes_flow_nominal)
    annotation (Placement(transformation(extent={{120,240},{140,260}})));

  ZoneModel_simplified_v2 cor(
    nConExt=0,
    nConExtWin=0,
    nConPar=0,
    nConBou=3,
    nSurBou=0,
    datConBou(
      layers={conIntWal,conIntWal,conIntWal},
      A={6.47,40.76,6.47}*hRoo,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    Q_flow_nominal=QCor_flow_nominal,
    TRadSup_nominal=TChiWatSup_nominal,
    TRadRet_nominal=TChiWatRet_nominal,
    mRad_flow_nominal=mChiWatCor_flow_nominal,
    V=VRooCor,
    TAir_nominal=297.04,
    mA_flow_nominal=mAirCor_flow_nominal,
    mAChiBea_flow_nominal=mAChiBeaCor_flow_nominal)
    annotation (Placement(transformation(extent={{180,240},{200,260}})));

  ZoneModel_simplified_v2 eas(
    nConExt=0,
    nConExtWin=0,
    nConPar=0,
    nConBou=3,
    nSurBou=0,
    datConBou(
      layers={conIntWal,conIntWal,conIntWal},
      A={6.47,40.76,6.47}*hRoo,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    Q_flow_nominal=QEas_flow_nominal,
    TRadSup_nominal=TChiWatSup_nominal,
    TRadRet_nominal=TChiWatRet_nominal,
    mRad_flow_nominal=mChiWatEas_flow_nominal,
    V=VRooEas,
    TAir_nominal=297.04,
    mA_flow_nominal=mAirEas_flow_nominal,
    mAChiBea_flow_nominal=mAChiBeaEas_flow_nominal)
    annotation (Placement(transformation(extent={{240,240},{260,260}})));

  ZoneModel_simplified_v2 sou(
    nConExt=0,
    nConExtWin=0,
    nConPar=0,
    nConBou=3,
    nSurBou=0,
    datConBou(
      layers={conIntWal,conIntWal,conIntWal},
      A={6.47,40.76,6.47}*hRoo,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    Q_flow_nominal=QSou_flow_nominal,
    TRadSup_nominal=TChiWatSup_nominal,
    TRadRet_nominal=TChiWatRet_nominal,
    mRad_flow_nominal=mChiWatSou_flow_nominal,
    V=VRooSou,
    TAir_nominal=297.04,
    mA_flow_nominal=mAirSou_flow_nominal,
    mAChiBea_flow_nominal=mAChiBeaSou_flow_nominal)
    annotation (Placement(transformation(extent={{180,180},{200,200}})));
  BoundaryConditions.WeatherData.Bus zonMeaBus "Zone measurements bus"
    annotation (Placement(transformation(extent={{204,262},{224,282}}),
        iconTransformation(extent={{38,218},{58,238}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput                        TZon[5]
    "Measured zone temperature"
    annotation (Placement(transformation(extent={{580,240},{620,280}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yRelHumZon[5] "Measured zone relative humidity"
    annotation (Placement(transformation(extent={{580,280},{620,320}}),
        iconTransformation(extent={{580,280},{620,320}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput VDisAir_flow[5]
    "Measured zone discharge air volume flow rate"
    annotation (Placement(transformation(extent={{576,188},{616,228}}),
        iconTransformation(extent={{576,188},{616,228}})));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_1
    annotation (Placement(transformation(extent={{300,250},{320,270}})));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_2
    annotation (Placement(transformation(extent={{300,290},{320,310}})));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_3
    annotation (Placement(transformation(extent={{300,210},{320,230}})));
  Fluid.FixedResistances.Junction jun6(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,150},{10,170}})));
  Fluid.FixedResistances.Junction jun7(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{20,150},{40,170}})));
  Fluid.FixedResistances.Junction jun8(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{50,150},{70,170}})));

  Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,-
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={100,0})));
  Fluid.FixedResistances.Junction jun9(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,-
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={100,40})));
  Fluid.FixedResistances.Junction jun10(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,-
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={100,80})));
  Fluid.FixedResistances.Junction jun11(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,-
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={100,120})));
  Fluid.Actuators.Valves.TwoWayLinear val1(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatEas_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  Fluid.Actuators.Valves.TwoWayLinear val3(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatNor_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{120,30},{140,50}})));
  Fluid.Actuators.Valves.TwoWayLinear val4(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatWes_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{120,70},{140,90}})));
  Fluid.Actuators.Valves.TwoWayLinear val5(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatCor_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{120,110},{140,130}})));
  Fluid.Actuators.Valves.TwoWayLinear val2(
    redeclare package Medium = MediumW,
    m_flow_nominal=mChiWatTot_flow_nominal,
    dpValve_nominal=dpValve_nominal,
    dpFixed_nominal=dpFixed_nominal)
    annotation (Placement(transformation(extent={{160,130},{180,150}})));
  Fluid.FixedResistances.Junction jun2(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = -90,
        origin={250,0})));
  Fluid.FixedResistances.Junction jun12(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = -90,
        origin={250,40})));
  Fluid.FixedResistances.Junction jun13(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = -90,
        origin={250,80})));
  Fluid.FixedResistances.Junction jun14(
    redeclare package Medium = MediumW,
    m_flow_nominal={mChiWatTot_flow_nominal,-mChiWatTot_flow_nominal,
        mChiWatTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = -90,
        origin={250,120})));
  Fluid.FixedResistances.Junction jun4(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,-
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={350,-130})));
  Examples.VAVReheat.ThermalZones.VAVBranch easCAVTer(
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    m_flow_nominal=mAirEas_flow_nominal,
    VRoo=VRooEas,
    allowFlowReversal=false)
    "Zone for core of buildings (azimuth will be neglected)"
    annotation (Placement(transformation(extent={{390,-70},{430,-30}})));
  Fluid.FixedResistances.Junction jun15(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,-
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={350,-70})));
  Examples.VAVReheat.ThermalZones.VAVBranch norCAVTer(
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    m_flow_nominal=mAirNor_flow_nominal,
    VRoo=VRooNor,
    allowFlowReversal=false)
    "Zone for core of buildings (azimuth will be neglected)"
    annotation (Placement(transformation(extent={{390,-10},{430,30}})));
  Fluid.FixedResistances.Junction jun16(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,-
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={350,-10})));
  Examples.VAVReheat.ThermalZones.VAVBranch wesCAVTer(
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    m_flow_nominal=mAirWes_flow_nominal,
    VRoo=VRooWes,
    allowFlowReversal=false)
    "Zone for core of buildings (azimuth will be neglected)"
    annotation (Placement(transformation(extent={{390,50},{430,90}})));
  Fluid.FixedResistances.Junction jun17(
    redeclare package Medium = MediumA,
    m_flow_nominal={mAirTot_flow_nominal,-mAirTot_flow_nominal,-
        mAirTot_flow_nominal},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation = 90,
        origin={350,50})));
  Examples.VAVReheat.ThermalZones.VAVBranch corCAVTer(
    redeclare package MediumA = MediumA,
    redeclare package MediumW = MediumW,
    m_flow_nominal=mAirCor_flow_nominal,
    VRoo=VRooCor,
    allowFlowReversal=false)
    "Zone for core of buildings (azimuth will be neglected)"
    annotation (Placement(transformation(extent={{390,110},{430,150}})));
  Modelica.Blocks.Routing.DeMultiplex demux3(n=5)
    annotation (Placement(transformation(extent={{80,280},{100,300}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput QFlo[5] "Heate flow rate into the zone"
    annotation (Placement(transformation(extent={{-380,270},{-340,310}})));

  CDL.Logical.Pre pre
    annotation (Placement(transformation(extent={{-320,60},{-300,80}})));
  parameter HeatTransfer.Data.OpaqueConstructions.Generic conExtWal(final nLay=3,
      material={matWoo,matIns,matGyp}) "Exterior construction"
    annotation (Placement(transformation(extent={{380,312},{400,332}})));
  parameter HeatTransfer.Data.OpaqueConstructions.Generic conIntWal(final nLay=1,
      material={matGyp2}) "Interior wall construction"
    annotation (Placement(transformation(extent={{420,312},{440,332}})));
  parameter HeatTransfer.Data.Solids.Plywood matWoo(
    x=0.01,
    k=0.11,
    d=544,
    nStaRef=1) "Wood for exterior construction"
    annotation (Placement(transformation(extent={{452,308},{472,328}})));
  parameter HeatTransfer.Data.Solids.Generic matIns(
    x=0.087,
    k=0.049,
    c=836.8,
    d=265,
    nStaRef=5) "Steelframe construction with insulation"
    annotation (Placement(transformation(extent={{492,308},{512,328}})));
  parameter HeatTransfer.Data.Solids.GypsumBoard matGyp(
    x=0.0127,
    k=0.16,
    c=830,
    d=784,
    nStaRef=2) "Gypsum board"
    annotation (Placement(transformation(extent={{450,280},{470,300}})));
  parameter HeatTransfer.Data.Solids.GypsumBoard matGyp2(
    x=0.025,
    k=0.16,
    c=830,
    d=784,
    nStaRef=2) "Gypsum board"
    annotation (Placement(transformation(extent={{490,280},{510,300}})));
  CDL.Interfaces.RealOutput                        outdoorairtemp
    "Measured DOAS return air relative humidity"
    annotation (Placement(transformation(extent={{596,-232},{636,-192}}),
        iconTransformation(extent={{596,-232},{636,-192}})));
  Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium = MediumA,
      m_flow_nominal=mAirTot_flow_nominal)
    annotation (Placement(transformation(extent={{-170,186},{-192,160}})));
  CDL.Interfaces.RealOutput returnairtemp "Measured bypass valve position"
    annotation (Placement(transformation(extent={{582,152},{622,192}}),
        iconTransformation(extent={{584,102},{624,142}})));
equation

  connect(jun.port_1,pum. port_b) annotation (Line(points={{100,-50},{100,-120}},
                               color={0,127,255}));

  connect(jun.port_3, val.port_a)
    annotation (Line(points={{110,-40},{120,-40}},   color={0,127,255}));

  connect(weaDat.weaBus, weaBus) annotation (Line(
      points={{-290,-10},{-270,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));

  connect(weaBus, amb.weaBus) annotation (Line(
      points={{-270,-10},{-260,-10},{-260,-8.78},{-250,-8.78}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));

  connect(cooCoi.port_b1,sinCoo. ports[1]) annotation (Line(
      points={{-130,-12},{-140,-12},{-140,-60}},
      color={28,108,200},
      thickness=0.5));

  connect(cooCoi.port_a1,souCoo. ports[1]) annotation (Line(
      points={{-110,-12},{-100,-12},{-100,-60}},
      color={28,108,200},
      thickness=0.5));

  connect(heaCoi.port_b2,cooCoi. port_a2) annotation (Line(
      points={{-180,0},{-130,0}},
      color={0,127,255},
      thickness=0.5));

  connect(souHea.ports[1],heaCoi. port_a1) annotation (Line(
      points={{-170,-60},{-170,-12},{-180,-12}},
      color={28,108,200},
      thickness=0.5));

  connect(heaCoi.port_b1,sinHea. ports[1]) annotation (Line(
      points={{-200,-12},{-210,-12},{-210,-60}},
      color={28,108,200},
      thickness=0.5));

  connect(amb.ports[1], heaCoi.port_a2) annotation (Line(points={{-228,-6.06667},
          {-206,-6.06667},{-206,0},{-200,0}},
                                 color={0,127,255}));

  connect(souCoo1.ports[1], pum.port_a)
    annotation (Line(points={{100,-164},{100,-140}}, color={0,127,255}));

  connect(jun3.port_2, sinCoo1.ports[1])
    annotation (Line(points={{250,-50},{250,-166},{248,-166}},
                                                    color={0,127,255}));

  connect(fan.port_b, senTem.port_a)
    annotation (Line(points={{-60,0},{-40,0}}, color={0,127,255}));

  connect(cooCoi.port_b2, fan.port_a)
    annotation (Line(points={{-110,0},{-80,0}}, color={0,127,255}));

  connect(senRelPre.port_a, pum.port_b) annotation (Line(points={{150,-70},{100,
          -70},{100,-120}},  color={0,127,255}));

  connect(senRelPre.port_b, sinCoo1.ports[2]) annotation (Line(points={{170,-70},
          {252,-70},{252,-166}},  color={0,127,255}));

  connect(fan.port_b, senRelPre1.port_a)
    annotation (Line(points={{-60,0},{-50,0},{-50,20}}, color={0,127,255}));

  connect(senRelPre1.port_b, amb.ports[2]) annotation (Line(points={{-50,40},{
          -50,60},{-212,60},{-212,-2},{-216,-2},{-216,-9},{-228,-9}},
        color={0,127,255}));

  connect(pum.y_actual, hys.u) annotation (Line(points={{93,-119},{93,-110},{138,
          -110}}, color={0,0,127}));

  connect(uPumSta, booToRea.u)
    annotation (Line(points={{-360,-110},{-322,-110}}, color={255,0,255}));

  connect(booToRea.y, pro.u1) annotation (Line(points={{-298,-110},{-290,-110},{
          -290,-124},{-282,-124}}, color={0,0,127}));

  connect(uPumSpe, pro.u2) annotation (Line(points={{-360,-140},{-290,-140},{-290,
          -136},{-282,-136}}, color={0,0,127}));

  connect(pro.y, pum.y)
    annotation (Line(points={{-258,-130},{88,-130}},  color={0,0,127}));

  connect(TChiWatSup, souCoo1.T_in) annotation (Line(points={{-360,-180},{80,-180},
          {80,-186},{96,-186}},                    color={0,0,127}));

  connect(uCooCoi, gai1.u)
    annotation (Line(points={{-360,-80},{-322,-80}}, color={0,0,127}));

  connect(gai1.y, souCoo.m_flow_in) annotation (Line(points={{-298,-80},{-270,
          -80},{-270,-110},{-108,-110},{-108,-82}}, color={0,0,127}));

  connect(uHeaCoi, gai.u)
    annotation (Line(points={{-360,-50},{-322,-50}}, color={0,0,127}));

  connect(gai.y, souHea.m_flow_in) annotation (Line(points={{-298,-50},{-260,
          -50},{-260,-92},{-178,-92},{-178,-82}}, color={0,0,127}));

  connect(senTem.T, TDOASDis) annotation (Line(points={{-30,11},{-30,20},{-10,20},
          {-10,-152},{348,-152},{348,-150},{600,-150}},
                                      color={0,0,127}));

  connect(uFanSta, booToRea1.u)
    annotation (Line(points={{-360,40},{-322,40}}, color={255,0,255}));

  connect(booToRea1.y, pro1.u1) annotation (Line(points={{-298,40},{-290,40},{
          -290,26},{-282,26}}, color={0,0,127}));

  connect(uFanSpe, pro1.u2) annotation (Line(points={{-360,10},{-300,10},{-300,
          14},{-282,14}}, color={0,0,127}));

  connect(pro1.y, fan.y)
    annotation (Line(points={{-258,20},{-70,20},{-70,12}}, color={0,0,127}));

  connect(fan.y_actual, hys1.u) annotation (Line(points={{-59,7},{-54,7},{-54,-200},
          {-42,-200}},     color={0,0,127}));

  connect(val.y_actual, mux1.u[1]) annotation (Line(points={{135,-33},{154,-33},
          {154,85.6},{160,85.6}},                     color={0,0,127}));

  connect(demux.y[1], val.y) annotation (Line(points={{60,-14.4},{130,-14.4},{130,
          -28}},                          color={0,0,127}));

  connect(senRelHum.phi, relHumDOASRet) annotation (Line(points={{-94.1,157},{
          -94.1,-190},{560,-190},{560,-176},{602,-176}}, color={0,0,127}));
  connect(wes.port_a, sou.port_a) annotation (Line(points={{130,260},{130,270},{
          170,270},{170,230},{190,230},{190,200}}, color={191,0,0}));
  connect(wes.port_a, nor.port_a) annotation (Line(points={{130,260},{130,270},{
          170,270},{170,330},{190,330},{190,320}}, color={191,0,0}));
  connect(nor.port_a, eas.port_a) annotation (Line(points={{190,320},{190,330},{
          250,330},{250,260}}, color={191,0,0}));
  connect(eas.port_a, sou.port_a) annotation (Line(points={{250,260},{250,264},{
          210,264},{210,230},{190,230},{190,200}}, color={191,0,0}));
  connect(wes.port_a, cor.port_a) annotation (Line(points={{130,260},{130,270},{
          190,270},{190,260}}, color={191,0,0}));
  connect(eas.port_a, cor.port_a) annotation (Line(points={{250,260},{250,264},{
          190,264},{190,260}}, color={191,0,0}));

  connect(sou.yRelHumZon, zonMeaBus.yRelHumSou)
    annotation (Line(points={{202,198},{214,198},{214,272}}, color={0,0,127}));
  connect(sou.TZon, zonMeaBus.TSou)
    annotation (Line(points={{202,194},{214,194},{214,272}}, color={0,0,127}));
  connect(sou.VDisAir_flow, zonMeaBus.VAirSou)
    annotation (Line(points={{202,190},{214,190},{214,272}}, color={0,0,127}));

  connect(wes.yRelHumZon, zonMeaBus.yRelHumWes)
    annotation (Line(points={{142,258},{172,258},{172,272},{214,272}},
                                                             color={0,0,127}));
  connect(wes.TZon, zonMeaBus.TWes)
    annotation (Line(points={{142,254},{172,254},{172,272},{214,272}},
                                                             color={0,0,127}));
  connect(wes.VDisAir_flow, zonMeaBus.VAirWes)
    annotation (Line(points={{142,250},{172,250},{172,272},{214,272}},
                                                             color={0,0,127}));

  connect(nor.yRelHumZon, zonMeaBus.yRelHumNor)
    annotation (Line(points={{202,318},{214,318},{214,272}}, color={0,0,127}));
  connect(nor.TZon, zonMeaBus.TNor)
    annotation (Line(points={{202,314},{214,314},{214,272}}, color={0,0,127}));
  connect(nor.VDisAir_flow, zonMeaBus.VAirNor)
    annotation (Line(points={{202,310},{214,310},{214,272}}, color={0,0,127}));

  connect(eas.yRelHumZon, zonMeaBus.yRelHumEas)
    annotation (Line(points={{262,258},{268,258},{268,272},{214,272}},
                                                             color={0,0,127}));
  connect(eas.TZon, zonMeaBus.TEas)
    annotation (Line(points={{262,254},{268,254},{268,272},{214,272}},
                                                             color={0,0,127}));
  connect(eas.VDisAir_flow, zonMeaBus.VAirEas)
    annotation (Line(points={{262,250},{268,250},{268,272},{214,272}},
                                                             color={0,0,127}));

  connect(cor.yRelHumZon, zonMeaBus.yRelHumCor)
    annotation (Line(points={{202,258},{214,258},{214,272}}, color={0,0,127}));
  connect(cor.TZon, zonMeaBus.TCor)
    annotation (Line(points={{202,254},{214,254},{214,272}}, color={0,0,127}));
  connect(cor.VDisAir_flow, zonMeaBus.VAirCor)
    annotation (Line(points={{202,250},{214,250},{214,272}}, color={0,0,127}));

  connect(multiplex5_1.y, TZon)
    annotation (Line(points={{321,260},{600,260}}, color={0,0,127}));

  connect(multiplex5_1.u1[1], zonMeaBus.TSou) annotation (Line(points={{298,270},
          {280,270},{280,272},{214,272}}, color={0,0,127}));
  connect(multiplex5_1.u2[1], zonMeaBus.TEas) annotation (Line(points={{298,265},
          {280,265},{280,272},{214,272}}, color={0,0,127}));
  connect(multiplex5_1.u3[1], zonMeaBus.TNor) annotation (Line(points={{298,260},
          {280,260},{280,272},{214,272}}, color={0,0,127}));
  connect(multiplex5_1.u4[1], zonMeaBus.TWes) annotation (Line(points={{298,255},
          {280,255},{280,272},{214,272}}, color={0,0,127}));
  connect(multiplex5_1.u5[1], zonMeaBus.TCor) annotation (Line(points={{298,250},
          {280,250},{280,272},{214,272}}, color={0,0,127}));

  connect(multiplex5_3.y, VDisAir_flow)
    annotation (Line(points={{321,220},{458,220},{458,208},{596,208}},
                                                   color={0,0,127}));
  connect(multiplex5_2.y, yRelHumZon)
    annotation (Line(points={{321,300},{600,300}}, color={0,0,127}));

  connect(multiplex5_2.u1[1], zonMeaBus.yRelHumSou)
    annotation (Line(points={{298,310},{280,310},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_2.u2[1], zonMeaBus.yRelHumEas)
    annotation (Line(points={{298,305},{280,305},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_2.u3[1], zonMeaBus.yRelHumNor)
    annotation (Line(points={{298,300},{280,300},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_2.u4[1], zonMeaBus.yRelHumWes)
    annotation (Line(points={{298,295},{280,295},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_2.u5[1], zonMeaBus.yRelHumCor)
    annotation (Line(points={{298,290},{280,290},{280,272},{214,272}},
    color={0,0,127}));

  connect(multiplex5_3.u1[1], zonMeaBus.VAirSou)
    annotation (Line(points={{298,230},{280,230},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_3.u2[1], zonMeaBus.VAirEas)
    annotation (Line(points={{298,225},{280,225},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_3.u3[1], zonMeaBus.VAirNor)
    annotation (Line(points={{298,220},{280,220},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_3.u4[1], zonMeaBus.VAirWes)
    annotation (Line(points={{298,215},{280,215},{280,272},{214,272}},
    color={0,0,127}));
  connect(multiplex5_3.u5[1], zonMeaBus.VAirCor)
    annotation (Line(points={{298,210},{280,210},{280,272},{214,272}},
    color={0,0,127}));

  connect(jun5.port_2, jun6.port_1)
    annotation (Line(points={{-20,160},{-10,160}}, color={0,127,255}));
  connect(jun6.port_2, jun7.port_1)
    annotation (Line(points={{10,160},{20,160}}, color={0,127,255}));
  connect(jun7.port_2, jun8.port_1)
    annotation (Line(points={{40,160},{50,160}}, color={0,127,255}));
  connect(sou.portAir_b, jun8.port_3) annotation (Line(points={{180,186},{160,186},
          {160,150},{60,150}}, color={0,127,255}));
  connect(eas.portAir_b, jun8.port_2) annotation (Line(points={{240,246},{230,246},
          {230,210},{140,210},{140,160},{70,160}}, color={0,127,255}));
  connect(nor.portAir_b, jun7.port_3) annotation (Line(points={{180,306},{160,306},
          {160,210},{140,210},{140,170},{46,170},{46,150},{30,150}}, color={0,127,
          255}));
  connect(wes.portAir_b, jun6.port_3) annotation (Line(points={{120,246},{112,246},
          {112,170},{14,170},{14,150},{0,150}}, color={0,127,255}));
  connect(cor.portAir_b, jun5.port_3) annotation (Line(points={{180,246},{160,246},
          {160,210},{140,210},{140,170},{-14,170},{-14,150},{-30,150}}, color={0,
          127,255}));
  connect(jun.port_2, jun1.port_1)
    annotation (Line(points={{100,-30},{100,-10}}, color={0,127,255}));
  connect(jun1.port_2, jun9.port_1)
    annotation (Line(points={{100,10},{100,30}}, color={0,127,255}));
  connect(jun9.port_2, jun10.port_1)
    annotation (Line(points={{100,50},{100,70}}, color={0,127,255}));
  connect(jun10.port_2, jun11.port_1)
    annotation (Line(points={{100,90},{100,110}}, color={0,127,255}));
  connect(jun1.port_3, val1.port_a)
    annotation (Line(points={{110,0},{120,0}}, color={0,127,255}));
  connect(jun9.port_3, val3.port_a)
    annotation (Line(points={{110,40},{120,40}}, color={0,127,255}));
  connect(jun10.port_3, val4.port_a)
    annotation (Line(points={{110,80},{120,80}}, color={0,127,255}));
  connect(jun11.port_3, val5.port_a)
    annotation (Line(points={{110,120},{120,120}}, color={0,127,255}));
  connect(jun11.port_2, val2.port_a) annotation (Line(points={{100,130},{100,140},
          {160,140}}, color={0,127,255}));
  connect(val2.port_b, jun14.port_1) annotation (Line(points={{180,140},{250,140},
          {250,130}}, color={0,127,255}));
  connect(jun14.port_2, jun13.port_1)
    annotation (Line(points={{250,110},{250,90}}, color={0,127,255}));
  connect(jun13.port_2, jun12.port_1)
    annotation (Line(points={{250,70},{250,50}}, color={0,127,255}));
  connect(jun12.port_2, jun2.port_1)
    annotation (Line(points={{250,30},{250,10}}, color={0,127,255}));
  connect(jun2.port_2, jun3.port_1)
    annotation (Line(points={{250,-10},{250,-30}}, color={0,127,255}));
  connect(val.port_b, sou.portChiWat_a) annotation (Line(points={{140,-40},{150,
          -40},{150,170},{186,170},{186,180}}, color={0,127,255}));
  connect(val1.port_b, eas.portChiWat_a) annotation (Line(points={{140,0},{150,0},
          {150,170},{246,170},{246,240}}, color={0,127,255}));
  connect(val3.port_b, nor.portChiWat_a) annotation (Line(points={{140,40},{150,
          40},{150,170},{224,170},{224,290},{186,290},{186,300}}, color={0,127,255}));
  connect(val4.port_b, wes.portChiWat_a) annotation (Line(points={{140,80},{150,
          80},{150,180},{126,180},{126,240}}, color={0,127,255}));
  connect(val5.port_b, cor.portChiWat_a) annotation (Line(points={{140,120},{150,
          120},{150,220},{186,220},{186,240}}, color={0,127,255}));
  connect(cor.portChiWat_b, jun14.port_3) annotation (Line(points={{194,240},{194,
          220},{228,220},{228,120},{240,120}}, color={0,127,255}));
  connect(wes.portChiWat_b, jun13.port_3) annotation (Line(points={{134,240},{134,
          164},{228,164},{228,80},{240,80}}, color={0,127,255}));
  connect(nor.portChiWat_b, jun12.port_3) annotation (Line(points={{194,300},{194,
          294},{228,294},{228,40},{240,40}}, color={0,127,255}));
  connect(eas.portChiWat_b, jun2.port_3) annotation (Line(points={{254,240},{254,
          216},{228,216},{228,0},{240,0}}, color={0,127,255}));
  connect(sou.portChiWat_b, jun3.port_3) annotation (Line(points={{194,180},{194,
          164},{228,164},{228,-40},{240,-40}}, color={0,127,255}));
  connect(uChiWatVal, demux.u) annotation (Line(points={{-360,180},{-240,180},{-240,
          140},{0,140},{0,-20},{38,-20}}, color={0,0,127}));
  connect(demux.y[2], val1.y) annotation (Line(points={{60,-17.2},{80,-17.2},{80,
          18},{130,18},{130,12}}, color={0,0,127}));
  connect(demux.y[3], val3.y) annotation (Line(points={{60,-20},{80,-20},{80,60},
          {130,60},{130,52}}, color={0,0,127}));
  connect(demux.y[4], val4.y) annotation (Line(points={{60,-22.8},{80,-22.8},{80,
          100},{130,100},{130,92}}, color={0,0,127}));
  connect(demux.y[5], val5.y) annotation (Line(points={{60,-25.6},{80,-25.6},{80,
          138},{130,138},{130,132}}, color={0,0,127}));
  connect(uBypValPos, val2.y) annotation (Line(points={{-360,220},{100,220},{100,
          158},{170,158},{170,152}}, color={0,0,127}));
  connect(val1.y_actual, mux1.u[2]) annotation (Line(points={{135,7},{154,7},{154,
          82.8},{160,82.8}}, color={0,0,127}));
  connect(val3.y_actual, mux1.u[3]) annotation (Line(points={{135,47},{154,47},{
          154,80},{160,80}}, color={0,0,127}));
  connect(val4.y_actual, mux1.u[4]) annotation (Line(points={{135,87},{154,87},{
          154,77.2},{160,77.2}}, color={0,0,127}));
  connect(val5.y_actual, mux1.u[5]) annotation (Line(points={{135,127},{154,127},
          {154,74.4},{160,74.4}}, color={0,0,127}));
  connect(mux1.y, yChiWatVal) annotation (Line(points={{181,80},{200,80},{200,-200},
          {520,-200},{520,80},{600,80}}, color={0,0,127}));
  connect(jun4.port_3, souCAVTer.port_a)
    annotation (Line(points={{360,-130},{400,-130}}, color={0,127,255}));
  connect(jun15.port_3, easCAVTer.port_a)
    annotation (Line(points={{360,-70},{400,-70}}, color={0,127,255}));
  connect(jun16.port_3, norCAVTer.port_a)
    annotation (Line(points={{360,-10},{400,-10}}, color={0,127,255}));
  connect(jun17.port_3, wesCAVTer.port_a)
    annotation (Line(points={{360,50},{400,50}}, color={0,127,255}));
  connect(senTem.port_b, jun4.port_1) annotation (Line(points={{-20,0},{10,0},{10,
          -214},{350,-214},{350,-140}}, color={0,127,255}));
  connect(jun4.port_2, jun15.port_1)
    annotation (Line(points={{350,-120},{350,-80}}, color={0,127,255}));
  connect(jun15.port_2, jun16.port_1)
    annotation (Line(points={{350,-60},{350,-20}}, color={0,127,255}));
  connect(jun16.port_2, jun17.port_1)
    annotation (Line(points={{350,0},{350,40}}, color={0,127,255}));
  connect(jun17.port_2, corCAVTer.port_a) annotation (Line(points={{350,60},{350,
          110},{400,110}}, color={0,127,255}));
  connect(demux1.y[1], souCAVTer.yVAV) annotation (Line(points={{320,25.6},{374,
          25.6},{374,-102},{386,-102}}, color={0,0,127}));
  connect(demux1.y[2], easCAVTer.yVAV) annotation (Line(points={{320,22.8},{374,
          22.8},{374,-42},{386,-42}}, color={0,0,127}));
  connect(demux1.y[3], norCAVTer.yVAV) annotation (Line(points={{320,20},{374,20},
          {374,18},{386,18}}, color={0,0,127}));
  connect(demux1.y[4], wesCAVTer.yVAV) annotation (Line(points={{320,17.2},{374,
          17.2},{374,78},{386,78}}, color={0,0,127}));
  connect(demux1.y[5], corCAVTer.yVAV) annotation (Line(points={{320,14.4},{374,
          14.4},{374,138},{386,138}}, color={0,0,127}));
  connect(demux2.y[1], souCAVTer.yVal) annotation (Line(points={{320,-34.4},{368,
          -34.4},{368,-118},{386,-118}}, color={0,0,127}));
  connect(demux2.y[2], easCAVTer.yVal) annotation (Line(points={{320,-37.2},{368,
          -37.2},{368,-58},{386,-58}}, color={0,0,127}));
  connect(demux2.y[3], norCAVTer.yVal) annotation (Line(points={{320,-40},{368,-40},
          {368,2},{386,2}}, color={0,0,127}));
  connect(demux2.y[4], wesCAVTer.yVal) annotation (Line(points={{320,-42.8},{368,
          -42.8},{368,62},{386,62}}, color={0,0,127}));
  connect(demux2.y[5], corCAVTer.yVal) annotation (Line(points={{320,-45.6},{368,
          -45.6},{368,122},{386,122}}, color={0,0,127}));
  connect(souCAVTer.y_actual, mux3.u[1]) annotation (Line(points={{432,-94},{450,
          -94},{450,-4.4},{460,-4.4}}, color={0,0,127}));
  connect(easCAVTer.y_actual, mux3.u[2]) annotation (Line(points={{432,-34},{450,
          -34},{450,-7.2},{460,-7.2}}, color={0,0,127}));
  connect(norCAVTer.y_actual, mux3.u[3]) annotation (Line(points={{432,26},{450,
          26},{450,-10},{460,-10}}, color={0,0,127}));
  connect(wesCAVTer.y_actual, mux3.u[4]) annotation (Line(points={{432,86},{450,
          86},{450,-12.8},{460,-12.8}}, color={0,0,127}));
  connect(corCAVTer.y_actual, mux3.u[5]) annotation (Line(points={{432,146},{450,
          146},{450,-15.6},{460,-15.6}}, color={0,0,127}));
  connect(mux3.y, yDamPos) annotation (Line(points={{481,-10},{560,-10},{560,10},
          {600,10}}, color={0,0,127}));
  connect(uCAVDam, demux1.u) annotation (Line(points={{-360,140},{-250,140},{-250,
          120},{16,120},{16,-90},{280,-90},{280,20},{298,20}}, color={0,0,127}));
  connect(uCAVReh, demux2.u) annotation (Line(points={{-360,100},{-4,100},{-4,-148},
          {290,-148},{290,-40},{298,-40}}, color={0,0,127}));
  connect(souCAVTer.port_b, sou.portAir_a) annotation (Line(points={{400,-90},{440,
          -90},{440,180},{220,180},{220,186},{200,186}}, color={0,127,255}));
  connect(easCAVTer.port_b, eas.portAir_a) annotation (Line(points={{400,-30},{440,
          -30},{440,180},{268,180},{268,246},{260,246}}, color={0,127,255}));
  connect(norCAVTer.port_b, nor.portAir_a) annotation (Line(points={{400,30},{440,
          30},{440,180},{220,180},{220,306},{200,306}}, color={0,127,255}));
  connect(wesCAVTer.port_b, wes.portAir_a) annotation (Line(points={{400,90},{440,
          90},{440,180},{220,180},{220,224},{146,224},{146,246},{140,246}},
        color={0,127,255}));
  connect(corCAVTer.port_b, cor.portAir_a) annotation (Line(points={{400,150},{440,
          150},{440,180},{220,180},{220,224},{206,224},{206,246},{200,246}},
        color={0,127,255}));
  connect(val2.y_actual, yBypValPos) annotation (Line(points={{175,147},{380,
          147},{380,170},{520,170},{520,122},{604,122}},
                                                    color={0,0,127}));
  connect(senRelPre1.p_rel, dPDOASAir) annotation (Line(points={{-41,30},{4,30},
          {4,-158},{540,-158},{540,-20},{600,-20}}, color={0,0,127}));
  connect(hys.y, yPumSta) annotation (Line(points={{162,-110},{196,-110},{196,-204},
          {546,-204},{546,-120},{600,-120}}, color={255,0,255}));
  connect(senRelPre.p_rel, dPChiWat) annotation (Line(points={{160,-79},{160,-84},
          {186,-84},{186,-212},{560,-212},{560,-90},{600,-90}}, color={0,0,127}));
  connect(QFlo, demux3.u)
    annotation (Line(points={{-360,290},{78,290}}, color={0,0,127}));
  connect(demux3.y[1], sou.QFlo) annotation (Line(points={{100,295.6},{106,295.6},
          {106,194},{178,194}}, color={0,0,127}));
  connect(demux3.y[2], eas.QFlo) annotation (Line(points={{100,292.8},{180,292.8},
          {180,280},{234,280},{234,254},{238,254}}, color={0,0,127}));
  connect(demux3.y[3], nor.QFlo) annotation (Line(points={{100,290},{106,290},{106,
          314},{178,314}}, color={0,0,127}));
  connect(demux3.y[4], wes.QFlo) annotation (Line(points={{100,287.2},{106,287.2},
          {106,254},{118,254}}, color={0,0,127}));
  connect(demux3.y[5], cor.QFlo) annotation (Line(points={{100,284.4},{106,284.4},
          {106,234},{176,234},{176,254},{178,254}}, color={0,0,127}));
  connect(val.y_actual, sou.uVal) annotation (Line(points={{135,-33},{154,-33},{
          154,190},{178,190}}, color={0,0,127}));
  connect(val1.y_actual, eas.uVal) annotation (Line(points={{135,7},{154,7},{154,
          204},{234,204},{234,250},{238,250}}, color={0,0,127}));
  connect(val3.y_actual, nor.uVal) annotation (Line(points={{135,47},{154,47},{154,
          310},{178,310}}, color={0,0,127}));
  connect(val4.y_actual, wes.uVal) annotation (Line(points={{135,87},{154,87},{154,
          220},{110,220},{110,250},{118,250}}, color={0,0,127}));
  connect(val5.y_actual, cor.uVal) annotation (Line(points={{135,127},{154,127},
          {154,248},{174,248},{174,250},{178,250}}, color={0,0,127}));
  connect(uFanSta, pre.u) annotation (Line(points={{-360,40},{-328,40},{-328,70},
          {-322,70}}, color={255,0,255}));
  connect(pre.y, yFanSta) annotation (Line(points={{-298,70},{-36,70},{-36,-174},
          {50,-174},{50,-208},{554,-208},{554,-50},{600,-50}}, color={255,0,255}));
  connect(weaBus.TDryBul, outdoorairtemp) annotation (Line(
      points={{-270,-10},{156,-10},{156,-212},{616,-212}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(jun5.port_1, senRelHum.port_a) annotation (Line(points={{-40,160},{
          -62,160},{-62,168},{-84,168}}, color={0,127,255}));
  connect(senRelHum.port_b, senTem1.port_a) annotation (Line(points={{-104,168},
          {-138,168},{-138,173},{-170,173}}, color={0,127,255}));
  connect(senTem1.port_b, amb.ports[3]) annotation (Line(points={{-192,173},{
          -220,173},{-220,-11.9333},{-228,-11.9333}}, color={0,127,255}));
  connect(senTem1.T, returnairtemp) annotation (Line(points={{-181,158.7},{-132,
          158.7},{-132,194},{554,194},{554,172},{602,172}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-340,-220},
            {580,340}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-340,-220},{580,340}})));
end TestBed_v2;
