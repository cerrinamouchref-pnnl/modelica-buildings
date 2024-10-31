within Buildings.Examples.ChilledBeamSystem.BaseClasses;
block TestBed
  "Testbed consisting of a 5-zone building model paired with DOAS and chilled water supply system"
  replaceable package MediumA = Buildings.Media.Air "Medium model";
  replaceable package MediumW = Buildings.Media.Water "Medium model";
  parameter Real hRoo = 2.74 "Height of room";
  parameter Modelica.Units.SI.PressureDifference dpValve_nominal = 6000 "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint";
  parameter Modelica.Units.SI.PressureDifference dpFixed_nominal = 0 "Pressure drop of pipe and other resistances that are in series";
  parameter Real TChiWatSup_nominal = 273.15 + 15.56 "Nominal chilled water supply temperature into zone";
  parameter Real TChiWatRet_nominal = 273.15 + 19 "Nominal chilled water return temperature from zone";
  parameter Real mChiWatTot_flow_nominal = mChiWatSou_flow_nominal + mChiWatEas_flow_nominal + mChiWatNor_flow_nominal + mChiWatWes_flow_nominal + mChiWatCor_flow_nominal "Total nominal chilled water flow rate through all five zones";
  parameter Real mAirTot_flow_nominal = mAirSou_flow_nominal + mAirEas_flow_nominal + mAirNor_flow_nominal + mAirWes_flow_nominal + mAirCor_flow_nominal "Total nominal air flow rate through all five zones";
  parameter Real mHotWatCoi_nominal = 96 "Hot water mass flow rate through AHU heating coil";
  parameter Real mChiWatCoi_nominal = 96 "Chilled water mass flow rate through AHU cooling coil";
  parameter Real mHotWatReh_nominal = mAirTot_flow_nominal*1000*15/4200/10 "Hot water mass flow rate through CAV terminal reheat coils";

  // Cerrina Added Parameters
   parameter Modelica.Units.SI.MassFlowRate mCooAir_flow_nominal "Nominal air mass flow rate from cooling sizing calculations";
  parameter Modelica.Units.SI.MassFlowRate mHeaAir_flow_nominal "Nominal air mass flow rate from heating sizing calculations";
  final parameter Modelica.Units.SI.MassFlowRate mHeaWat_flow_nominal= QHea_flow_nominal/(cpWatLiq*(THeaWatInl_nominal - THeaWatOut_nominal)) "Nominal mass flow rate of hot water to reheat coil";
  parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(start=55 + 273.15,
      displayUnit="degC") "Reheat coil nominal inlet water temperature";
  parameter Modelica.Units.SI.Temperature THeaWatOut_nominal(start=
        THeaWatInl_nominal - 10, displayUnit="degC")
    "Reheat coil nominal outlet water temperature";
  parameter Modelica.Units.SI.Temperature THeaAirInl_nominal(start=12 + 273.15,
      displayUnit="degC")
    "Inlet air nominal temperature into VAV box during heating";
  parameter Modelica.Units.SI.Temperature THeaAirDis_nominal(start=28 + 273.15,
      displayUnit="degC")
    "Discharge air temperature from VAV box during heating";
  parameter Modelica.Units.SI.HeatFlowRate QHea_flow_nominal=
      mHeaAir_flow_nominal * cpAir * (THeaAirDis_nominal-THeaAirInl_nominal)
    "Nominal heating heat flow rate";
  constant Modelica.Units.SI.SpecificHeatCapacity cpAir=Buildings.Utilities.Psychrometrics.Constants.cpAir
    "Air specific heat capacity";
  constant Modelica.Units.SI.SpecificHeatCapacity cpWatLiq=Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
    "Water specific heat capacity";

  // South zone
  parameter Real QSou_flow_nominal = -100000 "Nominal heat flow into south zone" annotation (
    Dialog(group = "South zone"));
  parameter Real VRooSou = 500 "Volume of zone air in south zone" annotation (
    Dialog(group = "South zone"));
  parameter Real mChiWatSou_flow_nominal = 0.2 "Chilled water volume flow rate into zone" annotation (
    Dialog(group = "South zone"));
  parameter Real mAirSou_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "South zone"));
  parameter Real mAChiBeaSou_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "South zone"));
  // East zone
  parameter Real QEas_flow_nominal = -100000 "Nominal heat flow into east zone" annotation (
    Dialog(group = "East zone"));
  parameter Real VRooEas = 500 "Volume of zone air in east zone" annotation (
    Dialog(group = "East zone"));
  parameter Real mChiWatEas_flow_nominal = 0.2 "Chilled water volume flow rate into zone" annotation (
    Dialog(group = "East zone"));
  parameter Real mAirEas_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "East zone"));
  parameter Real mAChiBeaEas_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "East zone"));
  // North zone
  parameter Real QNor_flow_nominal = -100000 "Nominal heat flow into north zone" annotation (
    Dialog(group = "North zone"));
  parameter Real VRooNor = 500 "Volume of zone air in north zone" annotation (
    Dialog(group = "North zone"));
  parameter Real mChiWatNor_flow_nominal = 0.2 "Chilled water volume flow rate into zone" annotation (
    Dialog(group = "North zone"));
  parameter Real mAirNor_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "North zone"));
  parameter Real mAChiBeaNor_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "North zone"));
  // West zone
  parameter Real QWes_flow_nominal = -100000 "Nominal heat flow into west zone" annotation (
    Dialog(group = "West zone"));
  parameter Real VRooWes = 500 "Volume of zone air in west zone" annotation (
    Dialog(group = "West zone"));
  parameter Real mChiWatWes_flow_nominal = 0.2 "Chilled water volume flow rate into zone" annotation (
    Dialog(group = "West zone"));
  parameter Real mAirWes_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "West zone"));
  parameter Real mAChiBeaWes_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "West zone"));
  // Core zone
  parameter Real QCor_flow_nominal = -100000 "Nominal heat flow into core zone" annotation (
    Dialog(group = "Core zone"));
  parameter Real VRooCor = 500 "Volume of zone air in core zone" annotation (
    Dialog(group = "Core zone"));
  parameter Real mChiWatCor_flow_nominal = 0.2 "Chilled water volume flow rate into zone" annotation (
    Dialog(group = "Core zone"));
  parameter Real mAirCor_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "Core zone"));
  parameter Real mAChiBeaCor_flow_nominal = 0.2 "Discharge air volume flow rate into zone" annotation (
    Dialog(group = "Core zone"));
  parameter HeatTransfer.Data.OpaqueConstructions.Generic conExtWal(final nLay = 3, final material = {matWoo, matIns, matGyp}) "Exterior construction" annotation (
    Placement(transformation(extent = {{380, 312}, {400, 332}})));
  parameter HeatTransfer.Data.OpaqueConstructions.Generic conIntWal(final nLay = 1, final material = {matGyp2}) "Interior wall construction" annotation (
    Placement(transformation(extent = {{420, 312}, {440, 332}})));
  parameter HeatTransfer.Data.Solids.Plywood matWoo(final x = 0.01, final k = 0.11, final d = 544, final nStaRef = 1) "Wood for exterior construction" annotation (
    Placement(transformation(extent = {{452, 308}, {472, 328}})));
  parameter HeatTransfer.Data.Solids.Generic matIns(final x = 0.087, final k = 0.049, final c = 836.8, final d = 265, final nStaRef = 5) "Steelframe construction with insulation" annotation (
    Placement(transformation(extent = {{492, 308}, {512, 328}})));
  parameter HeatTransfer.Data.Solids.GypsumBoard matGyp(final x = 0.0127, final k = 0.16, final c = 830, final d = 784, final nStaRef = 2) "Gypsum board" annotation (
    Placement(transformation(extent = {{450, 280}, {470, 300}})));
  parameter HeatTransfer.Data.Solids.GypsumBoard matGyp2(final x = 0.025, final k = 0.16, final c = 830, final d = 784, final nStaRef = 2) "Gypsum board" annotation (
    Placement(transformation(extent = {{490, 280}, {510, 300}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPumSta "Pump enable signal" annotation (
    Placement(transformation(extent = {{-380, -130}, {-340, -90}}), iconTransformation(extent={{-142,
            -160},{-102,-120}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSta "Supply fan enable signal" annotation (
    Placement(transformation(extent={{-382,48},{-342,88}}),      iconTransformation(extent={{-142,2},
            {-102,42}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uFanSpe
    "Fan speed signal"                                                       annotation (
    Placement(transformation(extent={{-380,18},{-340,58}}),       iconTransformation(extent={{-142,
            -42},{-102,-2}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uPumSpe "Pump speed signal" annotation (
    Placement(transformation(extent = {{-380, -160}, {-340, -120}}), iconTransformation(extent={{-142,
            -200},{-102,-160}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSup "Chilled water supply temperature" annotation (
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-380, -200}, {-340, -160}}, rotation = 0), iconTransformation(origin={-123,
            -219},                                                                                                                                            extent = {{-21, -21}, {21, 21}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uHeaCoi "Heating coil control signal" annotation (
    Placement(transformation(extent = {{-380, -70}, {-340, -30}}), iconTransformation(extent={{-142,
            -80},{-102,-40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCooCoi "AHU cooling coil control signal" annotation (
    Placement(transformation(extent = {{-380, -100}, {-340, -60}}), iconTransformation(extent={{-142,
            -122},{-102,-82}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCAVDam[5] "CAV damper signal" annotation (
    Placement(transformation(extent = {{-380, 120}, {-340, 160}}), iconTransformation(extent={{-142,80},
            {-102,120}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uCAVReh[5] "CAV reheat signal" annotation (
    Placement(transformation(extent = {{-380, 80}, {-340, 120}}), iconTransformation(extent={{-142,40},
            {-102,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uBypValPos "Bypass valve position signal" annotation (
    Placement(transformation(extent = {{-380, 200}, {-340, 240}}), iconTransformation(extent={{-142,
            160},{-102,200}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput uChiWatVal[5] "Chilled water valve position signal" annotation (
    Placement(transformation(extent = {{-380, 160}, {-340, 200}}), iconTransformation(extent={{-142,
            120},{-102,160}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yPumSta "Pump proven on" annotation (
    Placement(visible = true, transformation(origin = {2, 30}, extent = {{580, -140}, {620, -100}}, rotation = 0), iconTransformation(origin={0,80},    extent = {{100, -160}, {140, -120}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yFanSta "Supply fan proven on" annotation (
    Placement(visible = true, transformation(origin = {-2, 32}, extent = {{580, -70}, {620, -30}}, rotation = 0), iconTransformation(origin={0,60},   extent = {{100, -80}, {140, -40}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput dPChiWat "Measured chilled water differential presure" annotation (
    Placement(visible = true, transformation(origin = {-2, 20}, extent = {{580, -110}, {620, -70}}, rotation = 0), iconTransformation(origin={0,80},    extent = {{100, -120}, {140, -80}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput dPDOASAir "Measured airloop differential presure" annotation (
    Placement(visible = true, transformation(origin = {0, 36}, extent = {{580, -40}, {620, 0}}, rotation = 0), iconTransformation(origin={0,72},   extent = {{100, -40}, {140, 0}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TDOASDis(final unit = "K", displayUnit = "K", final quantity = "ThermodynamicTemperature") "Measured DOAS discharge air temperature" annotation (
    Placement(visible = true, transformation(origin = {0, 28}, extent = {{580, -170}, {620, -130}}, rotation = 0), iconTransformation(origin={0,100},   extent = {{100, -200}, {140, -160}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yChiWatVal[5] "Measured chilled water valve position" annotation (
    Placement(transformation(extent = {{580, 60}, {620, 100}}), iconTransformation(extent={{100,100},
            {140,140}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yBypValPos "Measured bypass valve position" annotation (
    Placement(transformation(extent = {{580, 100}, {620, 140}}), iconTransformation(extent={{100,140},
            {140,180}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yDamPos[5] "Measured CAV damper position" annotation (
    Placement(visible = true, transformation(origin = {2, 46}, extent = {{580, -10}, {620, 30}}, rotation = 0), iconTransformation(origin={0,60},   extent = {{100, 0}, {140, 40}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput relHumDOASRet "Measured DOAS return air relative humidity" annotation (
    Placement(visible = true, transformation(origin = {2, 28}, extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={0,100},   extent = {{100, -240}, {140, -200}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput TZon[5] "Measured zone temperature" annotation (
    Placement(transformation(extent = {{580, 240}, {620, 280}}), iconTransformation(extent={{100,180},
            {140,220}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yRelHumZon[5] "Measured zone relative humidity" annotation (
    Placement(transformation(extent = {{580, 280}, {620, 320}}), iconTransformation(extent={{100,200},
            {140,240}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput VDisAir_flow[5] "Measured zone discharge air volume flow rate" annotation (
    Placement(transformation(extent = {{580, 200}, {620, 240}}), iconTransformation(extent={{100,160},
            {140,200}})));
  Buildings.Fluid.FixedResistances.Junction jun(redeclare package Medium = MediumW, final m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal}, final dp_nominal = {0, 0, 0}) "Chilled water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, -40})));
  Buildings.Fluid.FixedResistances.Junction jun3(redeclare package Medium = MediumW, final m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, final dp_nominal = {0, 0, 0}) "Chilled water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {250, -40})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val(redeclare package Medium = MediumW, final m_flow_nominal = mChiWatSou_flow_nominal, final dpValve_nominal = dpValve_nominal, final dpFixed_nominal = dpFixed_nominal) "Chilled beam control valve for south zone" annotation (
    Placement(transformation(extent = {{120, -50}, {140, -30}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(final filNam=
        ModelicaServices.ExternalReferences.loadResource(
        "./Buildings/Resources/weatherdata/USA_AZ_Phoenix-Sky.Harbor.Intl.AP.722780_TMY3.mos"))                                                                                                             "Weather data" annotation (
    Placement(visible = true, transformation(origin={-40,-12},    extent = {{-310, -20}, {-290, 0}}, rotation = 0)));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather Data Bus" annotation (
    Placement(visible = true, transformation(origin = {-16, -6}, extent = {{-280, -20}, {-260, 0}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{0, 0}, {0, 0}}, rotation = 0)));
  Buildings.Fluid.Sources.Outside amb(redeclare package Medium = MediumA, final nPorts=6)   "Ambient conditions" annotation (
    Placement(visible = true, transformation(origin={-62,-22},    extent = {{-250, -20}, {-228, 2}}, rotation = 0)));
  Buildings.Fluid.HeatExchangers.WetCoilCounterFlow cooCoi(final show_T = true, final UA_nominal = 3*mAirTot_flow_nominal*1000*15/Buildings.Fluid.HeatExchangers.BaseClasses.lmtd(T_a1 = 26.2, T_b1 = 12.8, T_a2 = 6, T_b2 = 16), redeclare
      package Medium1 =                                                                                                                                                                                                         MediumW, redeclare
      package Medium2 =                                                                                                                                                                                                         MediumA, final m1_flow_nominal = mAirTot_flow_nominal*1000*15/4200/10, final m2_flow_nominal = mAirTot_flow_nominal, final dp2_nominal = 0, final dp1_nominal = 0, final energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, final allowFlowReversal1 = true, final allowFlowReversal2 = true) "Cooling coil" annotation (
    Placement(transformation(extent = {{-110, 4}, {-130, -16}})));
  Buildings.Fluid.Sources.Boundary_pT sinHea(redeclare package Medium = MediumW, p = 100000, T = 318.15, nPorts = 1) "Sink for heating coil" annotation (
    Placement(visible = true, transformation(origin = {-212, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Buildings.Fluid.Sources.Boundary_pT sinCoo(redeclare package Medium = MediumW, final p = 100000, final T = 285.15, final nPorts = 1) "Sink for cooling coil" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-140, -70})));
  Buildings.Fluid.Sources.MassFlowSource_T souCoo(redeclare package Medium = MediumW, final T = 280.372, final nPorts = 1, final use_m_flow_in = true) "Source for cooling coil" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-100, -70})));
  Buildings.Fluid.Sources.MassFlowSource_T souHea(redeclare package Medium = MediumW, final T = 318.15, final use_m_flow_in = true, final nPorts = 1) "Source for heating coil" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-170, -70})));
  Buildings.Fluid.HeatExchangers.DryCoilEffectivenessNTU heaCoi(final show_T = true, redeclare
      package Medium1 =                                                                                          MediumW, redeclare
      package Medium2 =                                                                                                                               MediumA, final m1_flow_nominal = mAirTot_flow_nominal*1000*(10 - (-20))/4200/10, final m2_flow_nominal = mAirTot_flow_nominal, final configuration = Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow, final Q_flow_nominal = mAirTot_flow_nominal*1006*(16.7 - 8.5), final dp1_nominal = 0, final dp2_nominal = 200 + 200 + 100 + 40, final allowFlowReversal1 = false, final allowFlowReversal2 = false, final T_a1_nominal = 318.15, final T_a2_nominal = 281.65) "Heating coil" annotation (
    Placement(visible = true, transformation(origin = {26, -10}, extent = {{-180, 4}, {-200, -16}}, rotation = 0)));
  Buildings.Examples.VAVReheat.BaseClasses.VAVReheatBox souCAVTer(redeclare
      package MediumA =                                                                       MediumA, redeclare
      package MediumW =                                                                                                            MediumW,
    mCooAir_flow_nominal=mCooAir_flow_nominal,
    mHeaAir_flow_nominal=mHeaAir_flow_nominal,                                                                                              final VRoo = VRooSou, final allowFlowReversal = false,
    THeaWatInl_nominal=THeaWatInl_nominal,
    THeaWatOut_nominal=THeaWatOut_nominal,
    THeaAirInl_nominal=THeaAirInl_nominal,
    THeaAirDis_nominal=THeaAirDis_nominal)                                                                                                                                                         "CAV terminal for south zone" annotation (
    Placement(transformation(extent = {{390, -130}, {430, -90}})));
  Buildings.Fluid.FixedResistances.Junction jun5(redeclare package Medium = MediumA, final m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, mAirTot_flow_nominal}, final dp_nominal = {0, 0, 0}) "Return air mixer" annotation (
    Placement(transformation(extent = {{-40, 150}, {-20, 170}})));
  Buildings.Fluid.Sources.Boundary_pT sinCoo1(redeclare package Medium = MediumW, final p = 100000, final T = 297.04, final nPorts = 2) "Sink for chillede water" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {250, -176})));
  Buildings.Fluid.Sources.Boundary_pT souCoo1(redeclare package Medium = MediumW, final p = 100000, final use_T_in = true, final T = 285.15, final nPorts = 1) "Source for chilled water to chilled beam manifolds" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, -174})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumA, final m_flow_nominal = mAirTot_flow_nominal) "AHU discharge air temperature sensor" annotation (
    Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
  Buildings.Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium
      =                                                                         MediumW) "Differential pressure sensor between chilled water supply and return" annotation (
    Placement(transformation(extent = {{150, -80}, {170, -60}})));
  Buildings.Fluid.Sensors.RelativePressure senRelPre1(redeclare package Medium
      =                                                                          MediumA) "Differential pressure sensor between AHU discharge and outdoor air" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin={-50,32})));
  Modelica.Blocks.Routing.DeMultiplex demux(final n = 5) "Demultiplexer for chilled water valve signals" annotation (
    Placement(transformation(extent={{42,-30},{62,-10}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(final uLow = 0.04, final uHigh = 0.05) "Block for generating pump proven on signal" annotation (
    Placement(transformation(extent = {{140, -120}, {160, -100}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea "Convert pump enable signal to Real signal" annotation (
    Placement(transformation(extent = {{-320, -120}, {-300, -100}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply pro "Find pump flow signal by multiplying enable signal with speed signal" annotation (
    Placement(transformation(extent = {{-280, -140}, {-260, -120}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(final k=10*
        mHotWatCoi_nominal)                                                              "Multiply control signal by nominal flowrate" annotation (
    Placement(transformation(extent = {{-320, -60}, {-300, -40}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai1(final k = mChiWatCoi_nominal) "Multiply control signal by nominal flowrate" annotation (
    Placement(transformation(extent = {{-320, -90}, {-300, -70}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1 "Convert fan enable signal to Real signal" annotation (
    Placement(visible = true, transformation(origin={-8,10},    extent = {{-320, 30}, {-300, 50}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Multiply pro1 "Find fan flow signal by multiplying enable signal with speed signal" annotation (
    Placement(visible = true, transformation(origin={-18,10},   extent = {{-280, 10}, {-260, 30}}, rotation = 0)));
  Modelica.Blocks.Routing.Multiplex mux1(final n = 5) "Multiplexer for chilled water valve position measurements" annotation (
    Placement(transformation(extent = {{160, 70}, {180, 90}})));
  Modelica.Blocks.Routing.DeMultiplex demux1(final n = 5) "Demultiplexer for CAV terminal damper signals" annotation (
    Placement(transformation(extent = {{300, -260}, {320, -240}})));
  Modelica.Blocks.Routing.DeMultiplex demux2(n = 5) "Demultiplexer for CAV terminal reheat signals" annotation (
    Placement(transformation(extent = {{300, -300}, {320, -280}})));
  Modelica.Blocks.Routing.Multiplex mux3(n = 5) "Multiplexer for CAV terminal damper position measurements" annotation (
    Placement(transformation(extent = {{490, -20}, {510, 0}})));
  Buildings.Fluid.Sensors.RelativeHumidityTwoPort senRelHum(redeclare package
      Medium =                                                                         MediumA, m_flow_nominal = mAirTot_flow_nominal) "Relative humidity sensor" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-90, 160})));
  Buildings.Examples.ChilledBeamSystem.BaseClasses.ZoneModel_simplified nor(
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
    mAChiBea_flow_nominal=mAChiBeaNor_flow_nominal) "North zone"
    annotation (Placement(transformation(extent={{180,300},{200,320}})));

  Buildings.Examples.ChilledBeamSystem.BaseClasses.ZoneModel_simplified wes(
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
    mAChiBea_flow_nominal=mAChiBeaWes_flow_nominal) "West zone"
    annotation (Placement(transformation(extent={{120,240},{140,260}})));

  Buildings.Examples.ChilledBeamSystem.BaseClasses.ZoneModel_simplified cor(
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
    mAChiBea_flow_nominal=mAChiBeaCor_flow_nominal) "Core zone"
    annotation (Placement(transformation(extent={{180,240},{200,260}})));

  Buildings.Examples.ChilledBeamSystem.BaseClasses.ZoneModel_simplified eas(
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
    mAChiBea_flow_nominal=mAChiBeaEas_flow_nominal) "East zone"
    annotation (Placement(transformation(extent={{240,240},{260,260}})));

  Buildings.Examples.ChilledBeamSystem.BaseClasses.ZoneModel_simplified sou(
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
    mAChiBea_flow_nominal=mAChiBeaSou_flow_nominal) "South zone"
    annotation (Placement(transformation(extent={{180,180},{200,200}})));

  Buildings.BoundaryConditions.WeatherData.Bus zonMeaBus "Zone measurements bus" annotation (
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{204, 262}, {224, 282}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{0, 0}, {0, 0}}, rotation = 0)));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_1 "Multiplexer for zone temperature measurements" annotation (
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{300, 250}, {320, 270}}, rotation = 0)));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_2 "Multiplexer for zone relative humidity measurements" annotation (
    Placement(transformation(extent = {{300, 290}, {320, 310}})));
  Modelica.Blocks.Routing.Multiplex5 multiplex5_3 "Multiplexer for zone discharge air volume flowrate measurements" annotation (
    Placement(transformation(extent = {{300, 210}, {320, 230}})));
  Buildings.Fluid.FixedResistances.Junction jun6(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Return air mixer" annotation (
    Placement(transformation(extent = {{-10, 150}, {10, 170}})));
  Buildings.Fluid.FixedResistances.Junction jun7(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Return air mixer" annotation (
    Placement(transformation(extent = {{20, 150}, {40, 170}})));
  Buildings.Fluid.FixedResistances.Junction jun8(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Return air mixer" annotation (
    Placement(transformation(extent = {{50, 150}, {70, 170}})));
  Fluid.FixedResistances.Junction jun1(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, 0})));
  Fluid.FixedResistances.Junction jun9(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, 40})));
  Fluid.FixedResistances.Junction jun10(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, 80})));
  Fluid.FixedResistances.Junction jun11(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {100, 120})));
  Fluid.Actuators.Valves.TwoWayLinear val1(redeclare package Medium = MediumW, m_flow_nominal = mChiWatEas_flow_nominal, dpValve_nominal = dpValve_nominal, dpFixed_nominal = dpFixed_nominal) "Chilled beam control valve for east zone" annotation (
    Placement(transformation(extent = {{120, -10}, {140, 10}})));
  Fluid.Actuators.Valves.TwoWayLinear val3(redeclare package Medium = MediumW, m_flow_nominal = mChiWatNor_flow_nominal, dpValve_nominal = dpValve_nominal, dpFixed_nominal = dpFixed_nominal) "Chilled beam control valve for north zone" annotation (
    Placement(transformation(extent = {{120, 30}, {140, 50}})));
  Fluid.Actuators.Valves.TwoWayLinear val4(redeclare package Medium = MediumW, m_flow_nominal = mChiWatWes_flow_nominal, dpValve_nominal = dpValve_nominal, dpFixed_nominal = dpFixed_nominal) "Chilled beam control valve for west zone" annotation (
    Placement(transformation(extent = {{120, 70}, {140, 90}})));
  Fluid.Actuators.Valves.TwoWayLinear val5(redeclare package Medium = MediumW, m_flow_nominal = mChiWatCor_flow_nominal, dpValve_nominal = dpValve_nominal, dpFixed_nominal = dpFixed_nominal) "Chilled beam control valve for core zone" annotation (
    Placement(transformation(extent = {{120, 110}, {140, 130}})));
  Fluid.Actuators.Valves.TwoWayLinear val2(redeclare package Medium = MediumW, m_flow_nominal = mChiWatTot_flow_nominal, dpValve_nominal = dpValve_nominal, dpFixed_nominal = dpFixed_nominal) "Chilled water bypass valve" annotation (
    Placement(transformation(extent = {{160, 130}, {180, 150}})));
  Fluid.FixedResistances.Junction jun2(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {250, 0})));
  Fluid.FixedResistances.Junction jun12(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {250, 40})));
  Fluid.FixedResistances.Junction jun13(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {250, 80})));
  Fluid.FixedResistances.Junction jun14(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Chilled water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {250, 120})));
  Fluid.FixedResistances.Junction jun4(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Supply air splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {350, -130})));
  Buildings.Examples.VAVReheat.BaseClasses.VAVReheatBox easCAVTer(redeclare
      package MediumA =                                                                       MediumA, redeclare
      package MediumW =                                                                                                            MediumW,
    mCooAir_flow_nominal=mCooAir_flow_nominal,
    mHeaAir_flow_nominal=mHeaAir_flow_nominal,                                                                                              VRoo = VRooEas, allowFlowReversal = false,
    THeaWatInl_nominal=THeaWatInl_nominal,
    THeaWatOut_nominal=THeaWatOut_nominal,
    THeaAirInl_nominal=THeaAirInl_nominal,
    THeaAirDis_nominal=THeaAirDis_nominal)                                                                                                                                             "CAV terminal for east zone" annotation (
    Placement(transformation(extent = {{390, -70}, {430, -30}})));
  Fluid.FixedResistances.Junction jun15(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Supply air splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {350, -70})));
  Buildings.Examples.VAVReheat.BaseClasses.VAVReheatBox norCAVTer(redeclare
      package MediumA =                                                                       MediumA, redeclare
      package MediumW =                                                                                                            MediumW,
    mCooAir_flow_nominal=mCooAir_flow_nominal,
    mHeaAir_flow_nominal=mHeaAir_flow_nominal,                                                                                              VRoo = VRooNor, allowFlowReversal = false,
    THeaWatInl_nominal=THeaWatInl_nominal,
    THeaWatOut_nominal=THeaWatOut_nominal,
    THeaAirInl_nominal=THeaAirInl_nominal,
    THeaAirDis_nominal=THeaAirDis_nominal)                                                                                                                                             "CAV terminal for north zone" annotation (
    Placement(transformation(extent = {{390, -10}, {430, 30}})));
  Fluid.FixedResistances.Junction jun16(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Supply air splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {350, -10})));
  Buildings.Examples.VAVReheat.BaseClasses.VAVReheatBox wesCAVTer(redeclare
      package MediumA =                                                                       MediumA, redeclare
      package MediumW =                                                                                                            MediumW,
    mCooAir_flow_nominal=mCooAir_flow_nominal,
    mHeaAir_flow_nominal=mHeaAir_flow_nominal,                                                                                              VRoo = VRooWes, allowFlowReversal = false,
    THeaWatInl_nominal=THeaWatInl_nominal,
    THeaWatOut_nominal=THeaWatOut_nominal,
    THeaAirInl_nominal=THeaAirInl_nominal,
    THeaAirDis_nominal=THeaAirDis_nominal)                                                                                                                                             "CAV terminal for west zone" annotation (
    Placement(transformation(extent = {{390, 50}, {430, 90}})));
  Fluid.FixedResistances.Junction jun17(redeclare package Medium = MediumA, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Supply air splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {350, 50})));
  Buildings.Examples.VAVReheat.BaseClasses.VAVReheatBox corCAVTer(redeclare
      package MediumA =                                                                       MediumA, redeclare
      package MediumW =                                                                                                            MediumW,
    mCooAir_flow_nominal=mCooAir_flow_nominal,
    mHeaAir_flow_nominal=mHeaAir_flow_nominal,                                                                                                                                                                                        VRoo = VRooCor, allowFlowReversal = false,
    THeaWatInl_nominal=THeaWatInl_nominal,
    THeaWatOut_nominal=THeaWatOut_nominal,
    THeaAirInl_nominal=THeaAirInl_nominal,
    THeaAirDis_nominal=THeaAirDis_nominal)                                                                                                                                                                                                         "CAV terminal for core zone" annotation (
    Placement(visible = true, transformation(origin = {0, -2}, extent = {{390, 110}, {430, 150}}, rotation = 0)));
  Modelica.Blocks.Routing.DeMultiplex demux3(n = 5) "Demultiplexer for zone heat gain signal" annotation (
    Placement(transformation(extent = {{80, 280}, {100, 300}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput QFlo[5] "Heat flow rate into the zone" annotation (
    Placement(transformation(extent = {{-380, 270}, {-340, 310}}), iconTransformation(extent={{-142,
            200},{-102,240}})));
  Buildings.Controls.OBC.CDL.Logical.Pre pre1 "Logical pre block" annotation (
    Placement(visible = true, transformation(origin = {-14, 8}, extent = {{-320, 60}, {-300, 80}}, rotation = 0)));
  Fluid.FixedResistances.Junction jun18(redeclare package Medium = MediumW, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {320, -80})));
  Fluid.FixedResistances.Junction jun22(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {480, 70})));
  Fluid.FixedResistances.Junction jun23(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {480, 20})));
  Buildings.Fluid.FixedResistances.Junction jun24(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water mixer" annotation (
    Placement(visible = true, transformation(origin = {480, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Fluid.FixedResistances.Junction jun25(redeclare package Medium = MediumW, m_flow_nominal = {mChiWatTot_flow_nominal, -mChiWatTot_flow_nominal, mChiWatTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water mixer" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {480, -90})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gaiM_flow(final k = mAirTot_flow_nominal*1000*15/4200/10) "Calculate hot water mass flowrate based on reheat signal" annotation (
    Placement(transformation(extent = {{140, -290}, {160, -270}})));
  Fluid.Sources.MassFlowSource_T souTer(redeclare package Medium = MediumW, nPorts = 1, use_m_flow_in = true, T = 323.15) "Hot water source for terminal boxes " annotation (
    Placement(transformation(extent = {{180, -290}, {200, -270}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax mulMax(nin = 5) "Find maximum reheat signal for generating hot water" annotation (
    Placement(transformation(extent = {{100, -290}, {120, -270}})));
  Fluid.Sources.Boundary_pT sinTer(redeclare package Medium = MediumW, p(displayUnit = "Pa") = 3E5, nPorts = 1) "Hot water sink for terminal boxes" annotation (
    Placement(transformation(extent = {{450, -260}, {470, -240}})));
  Fluid.FixedResistances.Junction jun19(redeclare package Medium = MediumW, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {320, -30})));
  Fluid.FixedResistances.Junction jun20(redeclare package Medium = MediumW, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {320, 20})));
  Fluid.FixedResistances.Junction jun21(redeclare package Medium = MediumW, m_flow_nominal = {mAirTot_flow_nominal, -mAirTot_flow_nominal, -mAirTot_flow_nominal}, dp_nominal = {0, 0, 0}) "Hot water splitter" annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {320, 70})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput OutdoorAirTemp "Outdoor Air Dry Bulb Temperature" annotation (
    Placement(visible = true, transformation(origin={8,-52},    extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={0,60},   extent = {{100, -240}, {140, -200}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel(delayTime = 10) annotation (
    Placement(visible = true, transformation(origin = {-288, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput exhFanSta "Exhaust Fan Proven on" annotation (
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{580, -70}, {620, -30}}, rotation = 0), iconTransformation(origin={0,80},    extent = {{100, -80}, {140, -40}}, rotation = 0)));
  Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium = MediumA, m_flow_nominal = mAirTot_flow_nominal) annotation (
    Placement(visible = true, transformation(origin = {-158, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Fluid.Sensors.RelativePressure relativePressure(redeclare package
      Medium =                                                                         MediumA) annotation (
    Placement(visible = true, transformation(origin = {-130, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput bldgSP "Building Static Pressure SetPoint" annotation (
    Placement(visible = true, transformation(origin={4,-106},    extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={-480,0},      extent = {{580, -200}, {620, -160}}, rotation = 0)));
  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumA) annotation (
    Placement(visible = true, transformation(origin={-296,6},     extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput rAT annotation (
    Placement(visible = true, transformation(origin={2,-76},    extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={0,80},   extent = {{100, -240}, {140, -200}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput erwsuphum "ERW supply relative humidity sensor" annotation (
    Placement(visible = true, transformation(origin = {0, -14}, extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={0,120},   extent = {{100, -240}, {140, -200}}, rotation = 0)));
  Buildings.Fluid.Sensors.RelativeHumidityTwoPort senRelHum1(redeclare package
      Medium =                                                                          MediumA,
      m_flow_nominal=mAirTot_flow_nominal)                                                       annotation (
    Placement(visible = true, transformation(origin={-186,16},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y mov(redeclare package
      Medium =                                                                          MediumA, dp_nominal(displayUnit = "Pa") = 2000, m_flow_nominal = mAirTot_flow_nominal)  annotation (
    Placement(visible = true, transformation(origin = {-64, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y mov1(redeclare package
      Medium =                                                                           MediumW, dp_nominal(displayUnit = "Pa") = 60000, m_flow_nominal = mChiWatTot_flow_nominal)  annotation (
    Placement(visible = true, transformation(origin = {74, -116}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Controls.OBC.CDL.Interfaces.RealInput uEneRecWheSpe
    "Energy Recovery speed signal" annotation (Placement(transformation(
          extent={{-380,-10},{-340,30}}), iconTransformation(extent={{-142,-20},
            {-102,20}})));
  Fluid.HeatExchangers.ThermalWheels.Sensible.SpeedControlled whe(
    redeclare package Medium1 = MediumA,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mAirTot_flow_nominal,
    m2_flow_nominal=mAirTot_flow_nominal,
    P_nominal=100)
    annotation (Placement(transformation(extent={{-260,6},{-240,26}})));
  Fluid.Sensors.TemperatureTwoPort senTemEneWhe(redeclare package Medium =
        MediumA, m_flow_nominal=mAirTot_flow_nominal) annotation (
      Placement(visible=true, transformation(
        origin={-220,32},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Controls.OBC.CDL.Interfaces.RealOutput           TEneWhe
    "Temperature of air leaving the energy recovery wheel"                                    annotation (
    Placement(visible = true, transformation(origin={2,-130},    extent = {{580, -200}, {620, -160}}, rotation = 0), iconTransformation(origin={-480,-40},    extent = {{580, -200}, {620, -160}}, rotation = 0)));
equation
  connect(jun.port_3, val.port_a) annotation (
    Line(points = {{110, -40}, {120, -40}}, color = {0, 127, 255}));
  connect(weaDat.weaBus, weaBus) annotation (
    Line(points={{-330,-22},{-330,-16},{-286,-16}},        color = {255, 204, 51}, thickness = 0.5));
  connect(weaBus, amb.weaBus) annotation (
    Line(points={{-286,-16},{-300,-16},{-300,-30.78},{-312,-30.78}},    color = {255, 204, 51}, thickness = 0.5));
  connect(cooCoi.port_b1, sinCoo.ports[1]) annotation (
    Line(points={{-130,-12},{-140,-12},{-140,-60}},        color = {28, 108, 200}, thickness = 0.5));
  connect(heaCoi.port_b2, cooCoi.port_a2) annotation (
    Line(points = {{-154, -10}, {-143, -10}, {-143, 0}, {-130, 0}}, color = {0, 127, 255}, thickness = 0.5));
  connect(souHea.ports[1], heaCoi.port_a1) annotation (
    Line(points = {{-170, -60}, {-170, -36}, {-154, -36}, {-154, -22}}, color = {28, 108, 200}, thickness = 0.5));
  connect(heaCoi.port_b1, sinHea.ports[1]) annotation (
    Line(points = {{-174, -22}, {-212, -22}, {-212, -64}}, color = {28, 108, 200}, thickness = 0.5));
  connect(jun3.port_2, sinCoo1.ports[1]) annotation (
    Line(points={{250,-50},{250,-166},{251,-166}},        color = {0, 127, 255}));
  connect(senRelPre.port_b, sinCoo1.ports[2]) annotation (
    Line(points={{170,-70},{249,-70},{249,-166}},        color = {0, 127, 255}));
  connect(uPumSta, booToRea.u) annotation (
    Line(points = {{-360, -110}, {-322, -110}}, color = {255, 0, 255}));
  connect(booToRea.y, pro.u1) annotation (
    Line(points = {{-298, -110}, {-290, -110}, {-290, -124}, {-282, -124}}, color = {0, 0, 127}));
  connect(uPumSpe, pro.u2) annotation (
    Line(points = {{-360, -140}, {-290, -140}, {-290, -136}, {-282, -136}}, color = {0, 0, 127}));
  connect(TChiWatSup, souCoo1.T_in) annotation (
    Line(points = {{-360, -180}, {80, -180}, {80, -194}, {96, -194}, {96, -186}}, color = {0, 0, 127}));
  connect(uCooCoi, gai1.u) annotation (
    Line(points = {{-360, -80}, {-322, -80}}, color = {0, 0, 127}));
  connect(gai1.y, souCoo.m_flow_in) annotation (
    Line(points = {{-298, -80}, {-270, -80}, {-270, -110}, {-108, -110}, {-108, -82}}, color = {0, 0, 127}));
  connect(uHeaCoi, gai.u) annotation (
    Line(points = {{-360, -50}, {-322, -50}}, color = {0, 0, 127}));
  connect(gai.y, souHea.m_flow_in) annotation (
    Line(points = {{-298, -50}, {-260, -50}, {-260, -92}, {-178, -92}, {-178, -82}}, color = {0, 0, 127}));
  connect(senTem.T, TDOASDis) annotation (
    Line(points = {{-30, 11}, {-30, 20}, {-10, 20}, {-10, -152}, {348, -152}, {348, -122}, {600, -122}}, color = {0, 0, 127}));
  connect(uFanSta, booToRea1.u) annotation (
    Line(points={{-362,68},{-341,68},{-341,50},{-330,50}},          color = {255, 0, 255}));
  connect(booToRea1.y, pro1.u1) annotation (
    Line(points={{-306,50},{-306,36},{-300,36}},                    color = {0, 0, 127}));
  connect(uFanSpe, pro1.u2) annotation (
    Line(points={{-360,38},{-330,38},{-330,24},{-300,24}},          color = {0, 0, 127}));
  connect(val.y_actual, mux1.u[1]) annotation (
    Line(points={{135,-33},{154,-33},{154,77.2},{160,77.2}},          color = {0, 0, 127}));
  connect(demux.y[1], val.y) annotation (
    Line(points={{62,-22.8},{130,-22.8},{130,-28}},        color = {0, 0, 127}));
  connect(senRelHum.port_a, jun5.port_1) annotation (
    Line(points = {{-80, 160}, {-40, 160}}, color = {0, 127, 255}));
  connect(senRelHum.phi, relHumDOASRet) annotation (
    Line(points = {{-90.1, 149}, {-90.1, -152}, {602, -152}}, color = {0, 0, 127}));
  connect(wes.port_a, sou.port_a) annotation (
    Line(points = {{130, 260}, {130, 270}, {170, 270}, {170, 230}, {190, 230}, {190, 200}}, color = {191, 0, 0}));
  connect(wes.port_a, nor.port_a) annotation (
    Line(points = {{130, 260}, {130, 270}, {170, 270}, {170, 330}, {190, 330}, {190, 320}}, color = {191, 0, 0}));
  connect(nor.port_a, eas.port_a) annotation (
    Line(points = {{190, 320}, {190, 330}, {250, 330}, {250, 260}}, color = {191, 0, 0}));
  connect(eas.port_a, sou.port_a) annotation (
    Line(points = {{250, 260}, {250, 264}, {210, 264}, {210, 230}, {190, 230}, {190, 200}}, color = {191, 0, 0}));
  connect(wes.port_a, cor.port_a) annotation (
    Line(points = {{130, 260}, {130, 270}, {190, 270}, {190, 260}}, color = {191, 0, 0}));
  connect(eas.port_a, cor.port_a) annotation (
    Line(points = {{250, 260}, {250, 264}, {190, 264}, {190, 260}}, color = {191, 0, 0}));
  connect(sou.yRelHumZon, zonMeaBus.yRelHumSou) annotation (
    Line(points = {{202, 198}, {214, 198}, {214, 272}}, color = {0, 0, 127}));
  connect(sou.TZon, zonMeaBus.TSou) annotation (
    Line(points = {{202, 194}, {214, 194}, {214, 272}}, color = {0, 0, 127}));
  connect(sou.VDisAir_flow, zonMeaBus.VAirSou) annotation (
    Line(points = {{202, 190}, {214, 190}, {214, 272}}, color = {0, 0, 127}));
  connect(wes.yRelHumZon, zonMeaBus.yRelHumWes) annotation (
    Line(points = {{142, 258}, {172, 258}, {172, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(wes.TZon, zonMeaBus.TWes) annotation (
    Line(points = {{142, 254}, {172, 254}, {172, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(wes.VDisAir_flow, zonMeaBus.VAirWes) annotation (
    Line(points = {{142, 250}, {172, 250}, {172, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(nor.yRelHumZon, zonMeaBus.yRelHumNor) annotation (
    Line(points = {{202, 318}, {214, 318}, {214, 272}}, color = {0, 0, 127}));
  connect(nor.TZon, zonMeaBus.TNor) annotation (
    Line(points = {{202, 314}, {214, 314}, {214, 272}}, color = {0, 0, 127}));
  connect(nor.VDisAir_flow, zonMeaBus.VAirNor) annotation (
    Line(points = {{202, 310}, {214, 310}, {214, 272}}, color = {0, 0, 127}));
  connect(eas.yRelHumZon, zonMeaBus.yRelHumEas) annotation (
    Line(points = {{262, 258}, {268, 258}, {268, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(eas.TZon, zonMeaBus.TEas) annotation (
    Line(points = {{262, 254}, {268, 254}, {268, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(eas.VDisAir_flow, zonMeaBus.VAirEas) annotation (
    Line(points = {{262, 250}, {268, 250}, {268, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(cor.yRelHumZon, zonMeaBus.yRelHumCor) annotation (
    Line(points = {{202, 258}, {214, 258}, {214, 272}}, color = {0, 0, 127}));
  connect(cor.TZon, zonMeaBus.TCor) annotation (
    Line(points = {{202, 254}, {214, 254}, {214, 272}}, color = {0, 0, 127}));
  connect(cor.VDisAir_flow, zonMeaBus.VAirCor) annotation (
    Line(points = {{202, 250}, {214, 250}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_1.y, TZon) annotation (
    Line(points = {{321, 260}, {600, 260}}, color = {0, 0, 127}));
  connect(multiplex5_1.u1[1], zonMeaBus.TSou) annotation (
    Line(points = {{298, 270}, {280, 270}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_1.u2[1], zonMeaBus.TEas) annotation (
    Line(points = {{298, 265}, {280, 265}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_1.u3[1], zonMeaBus.TNor) annotation (
    Line(points = {{298, 260}, {280, 260}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_1.u4[1], zonMeaBus.TWes) annotation (
    Line(points = {{298, 255}, {280, 255}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_1.u5[1], zonMeaBus.TCor) annotation (
    Line(points = {{298, 250}, {280, 250}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.y, VDisAir_flow) annotation (
    Line(points = {{321, 220}, {600, 220}}, color = {0, 0, 127}));
  connect(multiplex5_2.y, yRelHumZon) annotation (
    Line(points = {{321, 300}, {600, 300}}, color = {0, 0, 127}));
  connect(multiplex5_2.u1[1], zonMeaBus.yRelHumSou) annotation (
    Line(points = {{298, 310}, {280, 310}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_2.u2[1], zonMeaBus.yRelHumEas) annotation (
    Line(points = {{298, 305}, {280, 305}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_2.u3[1], zonMeaBus.yRelHumNor) annotation (
    Line(points = {{298, 300}, {280, 300}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_2.u4[1], zonMeaBus.yRelHumWes) annotation (
    Line(points = {{298, 295}, {280, 295}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_2.u5[1], zonMeaBus.yRelHumCor) annotation (
    Line(points = {{298, 290}, {280, 290}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.u1[1], zonMeaBus.VAirSou) annotation (
    Line(points = {{298, 230}, {280, 230}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.u2[1], zonMeaBus.VAirEas) annotation (
    Line(points = {{298, 225}, {280, 225}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.u3[1], zonMeaBus.VAirNor) annotation (
    Line(points = {{298, 220}, {280, 220}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.u4[1], zonMeaBus.VAirWes) annotation (
    Line(points = {{298, 215}, {280, 215}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(multiplex5_3.u5[1], zonMeaBus.VAirCor) annotation (
    Line(points = {{298, 210}, {280, 210}, {280, 272}, {214, 272}}, color = {0, 0, 127}));
  connect(jun5.port_2, jun6.port_1) annotation (
    Line(points = {{-20, 160}, {-10, 160}}, color = {0, 127, 255}));
  connect(jun6.port_2, jun7.port_1) annotation (
    Line(points = {{10, 160}, {20, 160}}, color = {0, 127, 255}));
  connect(jun7.port_2, jun8.port_1) annotation (
    Line(points = {{40, 160}, {50, 160}}, color = {0, 127, 255}));
  connect(sou.portAir_b, jun8.port_3) annotation (
    Line(points = {{180, 186}, {160, 186}, {160, 150}, {60, 150}}, color = {0, 127, 255}));
  connect(eas.portAir_b, jun8.port_2) annotation (
    Line(points = {{240, 246}, {230, 246}, {230, 210}, {140, 210}, {140, 160}, {70, 160}}, color = {0, 127, 255}));
  connect(nor.portAir_b, jun7.port_3) annotation (
    Line(points = {{180, 306}, {160, 306}, {160, 210}, {140, 210}, {140, 170}, {46, 170}, {46, 150}, {30, 150}}, color = {0, 127, 255}));
  connect(wes.portAir_b, jun6.port_3) annotation (
    Line(points = {{120, 246}, {112, 246}, {112, 170}, {14, 170}, {14, 150}, {0, 150}}, color = {0, 127, 255}));
  connect(cor.portAir_b, jun5.port_3) annotation (
    Line(points = {{180, 246}, {160, 246}, {160, 210}, {140, 210}, {140, 170}, {-14, 170}, {-14, 150}, {-30, 150}}, color = {0, 127, 255}));
  connect(jun.port_2, jun1.port_1) annotation (
    Line(points = {{100, -30}, {100, -10}}, color = {0, 127, 255}));
  connect(jun1.port_2, jun9.port_1) annotation (
    Line(points = {{100, 10}, {100, 30}}, color = {0, 127, 255}));
  connect(jun9.port_2, jun10.port_1) annotation (
    Line(points = {{100, 50}, {100, 70}}, color = {0, 127, 255}));
  connect(jun10.port_2, jun11.port_1) annotation (
    Line(points = {{100, 90}, {100, 110}}, color = {0, 127, 255}));
  connect(jun1.port_3, val1.port_a) annotation (
    Line(points = {{110, 0}, {120, 0}}, color = {0, 127, 255}));
  connect(jun9.port_3, val3.port_a) annotation (
    Line(points = {{110, 40}, {120, 40}}, color = {0, 127, 255}));
  connect(jun10.port_3, val4.port_a) annotation (
    Line(points = {{110, 80}, {120, 80}}, color = {0, 127, 255}));
  connect(jun11.port_3, val5.port_a) annotation (
    Line(points = {{110, 120}, {120, 120}}, color = {0, 127, 255}));
  connect(jun11.port_2, val2.port_a) annotation (
    Line(points = {{100, 130}, {100, 140}, {160, 140}}, color = {0, 127, 255}));
  connect(val2.port_b, jun14.port_1) annotation (
    Line(points = {{180, 140}, {250, 140}, {250, 130}}, color = {0, 127, 255}));
  connect(jun14.port_2, jun13.port_1) annotation (
    Line(points = {{250, 110}, {250, 90}}, color = {0, 127, 255}));
  connect(jun13.port_2, jun12.port_1) annotation (
    Line(points = {{250, 70}, {250, 50}}, color = {0, 127, 255}));
  connect(jun12.port_2, jun2.port_1) annotation (
    Line(points = {{250, 30}, {250, 10}}, color = {0, 127, 255}));
  connect(jun2.port_2, jun3.port_1) annotation (
    Line(points = {{250, -10}, {250, -30}}, color = {0, 127, 255}));
  connect(val.port_b, sou.portChiWat_a) annotation (
    Line(points = {{140, -40}, {150, -40}, {150, 170}, {186, 170}, {186, 180}}, color = {0, 127, 255}));
  connect(val1.port_b, eas.portChiWat_a) annotation (
    Line(points = {{140, 0}, {150, 0}, {150, 170}, {246, 170}, {246, 240}}, color = {0, 127, 255}));
  connect(val3.port_b, nor.portChiWat_a) annotation (
    Line(points = {{140, 40}, {150, 40}, {150, 170}, {224, 170}, {224, 290}, {186, 290}, {186, 300}}, color = {0, 127, 255}));
  connect(val4.port_b, wes.portChiWat_a) annotation (
    Line(points = {{140, 80}, {150, 80}, {150, 180}, {126, 180}, {126, 240}}, color = {0, 127, 255}));
  connect(val5.port_b, cor.portChiWat_a) annotation (
    Line(points = {{140, 120}, {150, 120}, {150, 220}, {186, 220}, {186, 240}}, color = {0, 127, 255}));
  connect(cor.portChiWat_b, jun14.port_3) annotation (
    Line(points = {{194, 240}, {194, 220}, {228, 220}, {228, 120}, {240, 120}}, color = {0, 127, 255}));
  connect(wes.portChiWat_b, jun13.port_3) annotation (
    Line(points = {{134, 240}, {134, 164}, {228, 164}, {228, 80}, {240, 80}}, color = {0, 127, 255}));
  connect(nor.portChiWat_b, jun12.port_3) annotation (
    Line(points = {{194, 300}, {194, 294}, {228, 294}, {228, 40}, {240, 40}}, color = {0, 127, 255}));
  connect(eas.portChiWat_b, jun2.port_3) annotation (
    Line(points = {{254, 240}, {254, 216}, {228, 216}, {228, 0}, {240, 0}}, color = {0, 127, 255}));
  connect(sou.portChiWat_b, jun3.port_3) annotation (
    Line(points = {{194, 180}, {194, 164}, {228, 164}, {228, -40}, {240, -40}}, color = {0, 127, 255}));
  connect(uChiWatVal, demux.u) annotation (
    Line(points={{-360,180},{-240,180},{-240,140},{0,140},{0,-20},{40,-20}},              color = {0, 0, 127}));
  connect(demux.y[2], val1.y) annotation (
    Line(points={{62,-21.4},{80,-21.4},{80,18},{130,18},{130,12}},            color = {0, 0, 127}));
  connect(demux.y[3], val3.y) annotation (
    Line(points={{62,-20},{80,-20},{80,60},{130,60},{130,52}},            color = {0, 0, 127}));
  connect(demux.y[4], val4.y) annotation (
    Line(points={{62,-18.6},{80,-18.6},{80,100},{130,100},{130,92}},            color = {0, 0, 127}));
  connect(demux.y[5], val5.y) annotation (
    Line(points={{62,-17.2},{80,-17.2},{80,138},{130,138},{130,132}},            color = {0, 0, 127}));
  connect(uBypValPos, val2.y) annotation (
    Line(points = {{-360, 220}, {100, 220}, {100, 158}, {170, 158}, {170, 152}}, color = {0, 0, 127}));
  connect(val1.y_actual, mux1.u[2]) annotation (
    Line(points={{135,7},{154,7},{154,78.6},{160,78.6}},          color = {0, 0, 127}));
  connect(val3.y_actual, mux1.u[3]) annotation (
    Line(points = {{135, 47}, {154, 47}, {154, 80}, {160, 80}}, color = {0, 0, 127}));
  connect(val4.y_actual, mux1.u[4]) annotation (
    Line(points={{135,87},{154,87},{154,81.4},{160,81.4}},          color = {0, 0, 127}));
  connect(val5.y_actual, mux1.u[5]) annotation (
    Line(points={{135,127},{154,127},{154,82.8},{160,82.8}},          color = {0, 0, 127}));
  connect(mux1.y, yChiWatVal) annotation (
    Line(points = {{181, 80}, {200, 80}, {200, -200}, {520, -200}, {520, 80}, {600, 80}}, color = {0, 0, 127}));
  connect(jun4.port_3, souCAVTer.port_aAir) annotation (
    Line(points = {{360, -130}, {410, -130}}, color = {0, 127, 255}));
  connect(jun15.port_3, easCAVTer.port_aAir) annotation (
    Line(points = {{360, -70}, {410, -70}}, color = {0, 127, 255}));
  connect(jun16.port_3, norCAVTer.port_aAir) annotation (
    Line(points = {{360, -10}, {410, -10}}, color = {0, 127, 255}));
  connect(jun17.port_3, wesCAVTer.port_aAir) annotation (
    Line(points = {{360, 50}, {410, 50}}, color = {0, 127, 255}));
  connect(senTem.port_b, jun4.port_1) annotation (
    Line(points = {{-20, 0}, {10, 0}, {10, -214}, {350, -214}, {350, -140}}, color = {0, 127, 255}));
  connect(jun4.port_2, jun15.port_1) annotation (
    Line(points = {{350, -120}, {350, -80}}, color = {0, 127, 255}));
  connect(jun15.port_2, jun16.port_1) annotation (
    Line(points = {{350, -60}, {350, -20}}, color = {0, 127, 255}));
  connect(jun16.port_2, jun17.port_1) annotation (
    Line(points = {{350, 0}, {350, 40}}, color = {0, 127, 255}));
  connect(jun17.port_2, corCAVTer.port_aAir) annotation (
    Line(points = {{350, 60}, {350, 108}, {410, 108}}, color = {0, 127, 255}));
  connect(demux1.y[1], souCAVTer.yVAV) annotation (
    Line(points={{320,-252.8},{374,-252.8},{374,-94},{386,-94}},          color = {0, 0, 127}));
  connect(demux1.y[2], easCAVTer.yVAV) annotation (
    Line(points={{320,-251.4},{374,-251.4},{374,-34},{386,-34}},          color = {0, 0, 127}));
  connect(demux1.y[3], norCAVTer.yVAV) annotation (
    Line(points = {{320, -250}, {374, -250}, {374, 26}, {386, 26}}, color = {0, 0, 127}));
  connect(demux1.y[4], wesCAVTer.yVAV) annotation (
    Line(points={{320,-248.6},{374,-248.6},{374,86},{386,86}},          color = {0, 0, 127}));
  connect(demux1.y[5], corCAVTer.yVAV) annotation (
    Line(points={{320,-247.2},{374,-247.2},{374,144},{386,144}},          color = {0, 0, 127}));
  connect(demux2.y[1], souCAVTer.yHea) annotation (
    Line(points={{320,-292.8},{368,-292.8},{368,-104},{386,-104}},          color = {0, 0, 127}));
  connect(demux2.y[2], easCAVTer.yHea) annotation (
    Line(points={{320,-291.4},{368,-291.4},{368,-44},{386,-44}},          color = {0, 0, 127}));
  connect(demux2.y[3], norCAVTer.yHea) annotation (
    Line(points = {{320, -290}, {368, -290}, {368, 16}, {386, 16}}, color = {0, 0, 127}));
  connect(demux2.y[4], wesCAVTer.yHea) annotation (
    Line(points={{320,-288.6},{368,-288.6},{368,76},{386,76}},          color = {0, 0, 127}));
  connect(demux2.y[5], corCAVTer.yHea) annotation (
    Line(points={{320,-287.2},{368,-287.2},{368,134},{386,134}},          color = {0, 0, 127}));
  connect(souCAVTer.y_actual, mux3.u[1]) annotation (
    Line(points={{432,-110},{450,-110},{450,-12.8},{490,-12.8}},        color = {0, 0, 127}));
  connect(easCAVTer.y_actual, mux3.u[2]) annotation (
    Line(points={{432,-50},{450,-50},{450,-11.4},{490,-11.4}},        color = {0, 0, 127}));
  connect(norCAVTer.y_actual, mux3.u[3]) annotation (
    Line(points = {{432, 10}, {450, 10}, {450, -10}, {490, -10}}, color = {0, 0, 127}));
  connect(wesCAVTer.y_actual, mux3.u[4]) annotation (
    Line(points={{432,70},{450,70},{450,-8.6},{490,-8.6}},            color = {0, 0, 127}));
  connect(corCAVTer.y_actual, mux3.u[5]) annotation (
    Line(points={{432,128},{450,128},{450,-7.2},{490,-7.2}},            color = {0, 0, 127}));
  connect(mux3.y, yDamPos) annotation (
    Line(points = {{511, -10}, {560, -10}, {560, 56}, {602, 56}}, color = {0, 0, 127}));
  connect(uCAVDam, demux1.u) annotation (
    Line(points = {{-360, 140}, {-250, 140}, {-250, 120}, {16, 120}, {16, -90}, {280, -90}, {280, -250}, {298, -250}}, color = {0, 0, 127}));
  connect(uCAVReh, demux2.u) annotation (
    Line(points = {{-360, 100}, {-4, 100}, {-4, -148}, {290, -148}, {290, -290}, {298, -290}}, color = {0, 0, 127}));
  connect(souCAVTer.port_bAir, sou.portAir_a) annotation (
    Line(points = {{410, -90}, {440, -90}, {440, 180}, {220, 180}, {220, 186}, {200, 186}}, color = {0, 127, 255}));
  connect(easCAVTer.port_bAir, eas.portAir_a) annotation (
    Line(points = {{410, -30}, {440, -30}, {440, 180}, {268, 180}, {268, 246}, {260, 246}}, color = {0, 127, 255}));
  connect(norCAVTer.port_bAir, nor.portAir_a) annotation (
    Line(points = {{410, 30}, {440, 30}, {440, 180}, {220, 180}, {220, 306}, {200, 306}}, color = {0, 127, 255}));
  connect(wesCAVTer.port_bAir, wes.portAir_a) annotation (
    Line(points = {{410, 90}, {440, 90}, {440, 180}, {220, 180}, {220, 224}, {146, 224}, {146, 246}, {140, 246}}, color = {0, 127, 255}));
  connect(corCAVTer.port_bAir, cor.portAir_a) annotation (
    Line(points = {{410, 148}, {440, 148}, {440, 180}, {220, 180}, {220, 224}, {206, 224}, {206, 246}, {200, 246}}, color = {0, 127, 255}));
  connect(val2.y_actual, yBypValPos) annotation (
    Line(points = {{175, 147}, {360, 147}, {360, 170}, {520, 170}, {520, 120}, {600, 120}}, color = {0, 0, 127}));
  connect(senRelPre1.p_rel, dPDOASAir) annotation (
    Line(points={{-41,32},{4,32},{4,-158},{540,-158},{540,16},{600,16}},              color = {0, 0, 127}));
  connect(hys.y, yPumSta) annotation (
    Line(points = {{162, -110}, {196, -110}, {196, -204}, {546, -204}, {546, -90}, {602, -90}}, color = {255, 0, 255}));
  connect(senRelPre.p_rel, dPChiWat) annotation (
    Line(points = {{160, -79}, {160, -84}, {186, -84}, {186, -212}, {560, -212}, {560, -70}, {598, -70}}, color = {0, 0, 127}));
  connect(QFlo, demux3.u) annotation (
    Line(points = {{-360, 290}, {78, 290}}, color = {0, 0, 127}));
  connect(demux3.y[1], sou.QFlo) annotation (
    Line(points={{100,287.2},{106,287.2},{106,194},{178,194}},          color = {0, 0, 127}));
  connect(demux3.y[2], eas.QFlo) annotation (
    Line(points={{100,288.6},{180,288.6},{180,280},{234,280},{234,254},{238,254}},              color = {0, 0, 127}));
  connect(demux3.y[3], nor.QFlo) annotation (
    Line(points = {{100, 290}, {106, 290}, {106, 314}, {178, 314}}, color = {0, 0, 127}));
  connect(demux3.y[4], wes.QFlo) annotation (
    Line(points={{100,291.4},{106,291.4},{106,254},{118,254}},          color = {0, 0, 127}));
  connect(demux3.y[5], cor.QFlo) annotation (
    Line(points={{100,292.8},{106,292.8},{106,234},{176,234},{176,254},{178,254}},              color = {0, 0, 127}));
  connect(val.y_actual, sou.uVal) annotation (
    Line(points = {{135, -33}, {154, -33}, {154, 190}, {178, 190}}, color = {0, 0, 127}));
  connect(val1.y_actual, eas.uVal) annotation (
    Line(points = {{135, 7}, {154, 7}, {154, 204}, {234, 204}, {234, 250}, {238, 250}}, color = {0, 0, 127}));
  connect(val3.y_actual, nor.uVal) annotation (
    Line(points = {{135, 47}, {154, 47}, {154, 310}, {178, 310}}, color = {0, 0, 127}));
  connect(val4.y_actual, wes.uVal) annotation (
    Line(points = {{135, 87}, {154, 87}, {154, 220}, {110, 220}, {110, 250}, {118, 250}}, color = {0, 0, 127}));
  connect(val5.y_actual, cor.uVal) annotation (
    Line(points = {{135, 127}, {154, 127}, {154, 248}, {174, 248}, {174, 250}, {178, 250}}, color = {0, 0, 127}));
  connect(uFanSta, pre1.u) annotation (
    Line(points={{-362,68},{-336,68},{-336,78}},        color = {255, 0, 255}));
  connect(pre1.y, yFanSta) annotation (
    Line(points={{-312,78},{-306,78},{-306,66},{-300,66},{-300,64},{-254,64},{
          -254,68},{-38,68},{-38,-174},{50,-174},{50,-208},{554,-208},{554,-18},
          {598,-18}},                                                                                                color = {255, 0, 255}));
  connect(souTer.m_flow_in, gaiM_flow.y) annotation (
    Line(points = {{178, -272}, {170, -272}, {170, -280}, {162, -280}}, color = {0, 0, 127}));
  connect(mulMax.y, gaiM_flow.u) annotation (
    Line(points = {{122, -280}, {138, -280}}, color = {0, 0, 127}));
  connect(uCAVReh, mulMax.u[1:5]) annotation (
    Line(points={{-360,100},{-4,100},{-4,-279.2},{98,-279.2}},          color = {0, 0, 127}));
  connect(souTer.ports[1], jun18.port_1) annotation (
    Line(points = {{200, -280}, {260, -280}, {260, -230}, {320, -230}, {320, -90}}, color = {0, 127, 255}));
  connect(jun22.port_2, jun23.port_1) annotation (
    Line(points = {{480, 60}, {480, 30}}, color = {0, 127, 255}));
  connect(jun23.port_2, jun24.port_1) annotation (
    Line(points = {{480, 10}, {480, -40}}, color = {0, 127, 255}));
  connect(jun24.port_2, jun25.port_1) annotation (
    Line(points = {{480, -60}, {480, -80}}, color = {0, 127, 255}));
  connect(jun25.port_2, sinTer.ports[1]) annotation (
    Line(points = {{480, -100}, {480, -250}, {470, -250}, {470, -250}}, color = {0, 127, 255}));
  connect(jun18.port_2, jun19.port_1) annotation (
    Line(points = {{320, -70}, {320, -40}}, color = {0, 127, 255}));
  connect(jun19.port_2, jun20.port_1) annotation (
    Line(points = {{320, -20}, {320, 10}}, color = {0, 127, 255}));
  connect(jun20.port_2, jun21.port_1) annotation (
    Line(points = {{320, 30}, {320, 60}}, color = {0, 127, 255}));
  connect(pre1.y, truDel.u) annotation (
    Line(points = {{-312, 78}, {-300, 78}, {-300, 84}}, color = {255, 0, 255}));
  connect(senTem1.port_b, senRelHum.port_b) annotation (
    Line(points = {{-148, 160}, {-100, 160}}, color = {0, 127, 255}));
  connect(cooCoi.port_a1, souCoo.ports[1]) annotation (
    Line(points = {{-110, -12}, {-100, -12}, {-100, -60}}, color = {0, 127, 255}));
  connect(eas.portAir_b, relativePressure.port_a) annotation (
    Line(points = {{240, 246}, {240, 44}, {-130, 44}}, color = {0, 127, 255}));
  connect(amb.ports[1], senMasFlo.port_a) annotation (
    Line(points={{-290,-32.8333},{-290,6},{-306,6}},     color = {0, 127, 255}));
  connect(jun21.port_2, corCAVTer.port_aHeaWat) annotation (
    Line(points = {{320, 80}, {320, 128}, {390, 128}}, color = {0, 127, 255}));
  connect(corCAVTer.port_bHeaWat, jun22.port_3) annotation (
    Line(points = {{390, 116}, {470, 116}, {470, 70}}, color = {0, 127, 255}));
  connect(jun20.port_3, wesCAVTer.port_aHeaWat) annotation (
    Line(points = {{330, 20}, {378, 20}, {378, 70}, {390, 70}}, color = {0, 127, 255}));
  connect(wesCAVTer.port_bHeaWat, jun23.port_3) annotation (
    Line(points = {{390, 58}, {470, 58}, {470, 20}}, color = {0, 127, 255}));
  connect(jun19.port_3, norCAVTer.port_aHeaWat) annotation (
    Line(points = {{330, -30}, {382, -30}, {382, 10}, {390, 10}}, color = {0, 127, 255}));
  connect(jun21.port_3, easCAVTer.port_aHeaWat) annotation (
    Line(points = {{330, 70}, {338, 70}, {338, -50}, {390, -50}}, color = {0, 127, 255}));
  connect(easCAVTer.port_bHeaWat, jun24.port_3) annotation (
    Line(points = {{390, -62}, {470, -62}, {470, -50}}, color = {0, 127, 255}));
  connect(jun18.port_3, souCAVTer.port_aHeaWat) annotation (
    Line(points = {{330, -80}, {342, -80}, {342, -110}, {390, -110}}, color = {0, 127, 255}));
  connect(souCAVTer.port_bHeaWat, jun25.port_3) annotation (
    Line(points = {{390, -122}, {378, -122}, {378, -144}, {460, -144}, {460, -90}, {470, -90}}, color = {0, 127, 255}));
  connect(norCAVTer.port_bHeaWat, jun22.port_1) annotation (
    Line(points = {{390, -2}, {458, -2}, {458, 80}, {480, 80}}, color = {0, 127, 255}));
  connect(senTem1.T, rAT) annotation (
    Line(points={{-158,171},{-114,171},{-114,172},{-68,172},{-68,-312},{
          470,-312},{470,-256},{602,-256}},                                                      color = {0, 0, 127}));
  connect(senRelHum1.port_b, heaCoi.port_a2) annotation (
    Line(points={{-176,16},{-174,16},{-174,-10}},      color = {0, 127, 255}));
  connect(cooCoi.port_b2, mov.port_a) annotation (
    Line(points = {{-110, 0}, {-74, 0}, {-74, -18}}, color = {0, 127, 255}));
  connect(mov.port_b, senTem.port_a) annotation (
    Line(points = {{-54, -18}, {-54, 0}, {-40, 0}}, color = {0, 127, 255}));
  connect(mov1.y_actual, hys.u) annotation (
    Line(points = {{67, -105}, {108, -105}, {108, -110}, {138, -110}}, color = {0, 0, 127}));
  connect(mov1.port_b, jun.port_1) annotation (
    Line(points = {{74, -106}, {100, -106}, {100, -50}}, color = {0, 127, 255}));
  connect(mov1.port_b, senRelPre.port_a) annotation (
    Line(points = {{74, -106}, {124, -106}, {124, -70}, {150, -70}}, color = {0, 127, 255}));
  connect(mov1.port_a, souCoo1.ports[1]) annotation (
    Line(points = {{74, -126}, {100, -126}, {100, -164}}, color = {0, 127, 255}));
  connect(pro.y, mov1.y) annotation (
    Line(points = {{-258, -130}, {36, -130}, {36, -116}, {62, -116}}, color = {0, 0, 127}));
  connect(weaBus.TDryBul, OutdoorAirTemp) annotation (Line(
      points={{-285.95,-15.95},{-258,-15.95},{-258,-60},{-244,-60},{-244,-224},
          {574,-224},{574,-232},{608,-232}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(senTem.port_b, senRelPre1.port_a) annotation (Line(points={{-20,0},{
          -36,0},{-36,16},{-50,16},{-50,22}}, color={0,127,255}));
  connect(pro1.y, mov.y)
    annotation (Line(points={{-276,30},{-64,30},{-64,-6}}, color={0,0,127}));
  connect(relativePressure.p_rel, bldgSP) annotation (Line(points={{-121,54},
          {-100,54},{-100,50},{-82,50},{-82,-340},{530,-340},{530,-286},{
          604,-286}},
                  color={0,0,127}));
  connect(senRelPre1.port_b, amb.ports[2]) annotation (Line(points={{-50,42},{24,
          42},{24,88},{-102,88},{-102,38},{-170,38},{-170,-32.1},{-290,-32.1}},
                         color={0,127,255}));
  connect(senRelHum1.phi, erwsuphum) annotation (Line(points={{-185.9,27},
          {-198,27},{-198,-254},{272,-254},{272,-194},{600,-194}},
                                                             color={0,0,127}));
  connect(relativePressure.port_b, amb.ports[3]) annotation (Line(points={{-130,64},
          {-142,64},{-142,62},{-210,62},{-210,-31.3667},{-290,-31.3667}},
                                                                        color={
          0,127,255}));
  connect(truDel.y, exhFanSta) annotation (Line(points={{-276,84},{-122,84},{
          -122,80},{28,80},{28,-130},{306,-130},{306,-180},{510,-180},{510,-50},
          {600,-50}}, color={255,0,255}));
  connect(uEneRecWheSpe, whe.uSpe) annotation (Line(points={{-360,10},{
          -326,10},{-326,16},{-262,16}}, color={0,0,127}));
  connect(senMasFlo.port_b, whe.port_a1) annotation (Line(points={{-286,6},
          {-246,6},{-246,22},{-260,22}},      color={0,127,255}));
  connect(senTem1.port_a, whe.port_a2) annotation (Line(points={{-168,160},
          {-192,160},{-192,10},{-240,10}}, color={0,127,255}));
  connect(whe.port_b2, amb.ports[4]) annotation (Line(points={{-260,10},{-246,
          10},{-246,0},{-264,0},{-264,-30.6333},{-290,-30.6333}},color={0,
          127,255}));
  connect(whe.port_b1, senTemEneWhe.port_a) annotation (Line(points={{
          -240,22},{-236,22},{-236,32},{-230,32}}, color={0,127,255}));
  connect(senTemEneWhe.port_b, senRelHum1.port_a) annotation (Line(points=
         {{-210,32},{-204,32},{-204,16},{-196,16}}, color={0,127,255}));
  connect(bldgSP, bldgSP)
    annotation (Line(points={{604,-286},{604,-286}}, color={0,0,127}));
  connect(senTemEneWhe.T, TEneWhe) annotation (Line(points={{-220,43},{
          -184,43},{-184,-394},{556,-394},{556,-310},{602,-310}}, color={
          0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -240}, {100, 240}}), graphics={                                                                                           Rectangle(origin = {-1, -5}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-101, -247}, {101, 247}}),
                                                                                                         Text(textColor = {0, 0, 255}, extent = {{-140, 280}, {140, 240}}, textString = "%name")}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-340, -320}, {580, 340}})));
end TestBed;
