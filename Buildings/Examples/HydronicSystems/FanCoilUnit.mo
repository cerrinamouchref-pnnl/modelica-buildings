within Buildings.Examples.HydronicSystems;
model FanCoilUnit
  extends Modelica.Icons.Example;
  replaceable package MediumA = Buildings.Media.Air(T_default=293.15)
    "Medium model for air";
 replaceable package MediumW = Buildings.Media.Water "Medium model";
   //replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
    //"Medium model of air";
  replaceable package MediumHW = Modelica.Media.Interfaces.PartialMedium
    "Medium model of hot water";
  replaceable package MediumCHW = Modelica.Media.Interfaces.PartialMedium
    "Medium model of chilled water";



  ThermalZones.EnergyPlus_24_2_0.Examples.SmallOffice.BaseClasses.Floor floor1(
      redeclare package Medium = MediumA)
    annotation (Placement(transformation(extent={{8,38},{86,82}})));

  Fluid.ZoneEquipment.FanCoilUnit.FourPipe fanCoiUni(
    redeclare package MediumA = MediumA,
    redeclare package MediumHW = MediumW,
    redeclare package MediumCHW = MediumW,
    mHotWat_flow_nominal=0.21805,
    dpAir_nominal=100,
    UAHeaCoi_nominal=2.25*146.06,
    mChiWat_flow_nominal=0.23106,
    UACooCoi_nominal=2.25*146.06,
    mAirOut_flow_nominal=0.000000001,
    mAir_flow_nominal=0.09,
    QHeaCoi_flow_nominal=6036.5,
    each fanPer=fanPer,
    fan(dpMax=100000))
    annotation (Placement(transformation(extent={{-10,-10},{8,8}})));

  Controls.OBC.ASHRAE.G36.FanCoilUnit.Controller conFCU(each TSupSet_max=308.15,
      each TSupSet_min=285.85)
    annotation (Placement(transformation(extent={{-62,-26},{-42,14}})));
  BoundaryConditions.WeatherData.ReaderTMY3 weaDat(each filNam=
        Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"),
      computeWetBulbTemperature=false) "Weather data reader"
    annotation (Placement(transformation(extent={{-66,56},{-46,76}})));
  Fluid.Sources.Boundary_pT souHea(
    redeclare package Medium = MediumW,
    p(displayUnit="Pa") = 300000 + 6000,
    T=333.15,
    nPorts=1) "Source for heating coil" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-3,-51})));
  Fluid.Sources.Boundary_pT sinHea(
    redeclare package Medium = MediumW,
    p(displayUnit="bar") = 300000,
    T=328.15,
    nPorts=1) "Sink for heating coil" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-17,-51})));
  Fluid.Sources.Boundary_pT sinCoo(
    redeclare package Medium = MediumW,
    p=300000,
    T=288.15,
    nPorts=1) "Sink for cooling coil" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={9,-51})));
  Fluid.Sources.Boundary_pT souCoo(
    redeclare package Medium = MediumW,
    p(displayUnit="Pa") = 300000 + 6000,
    T=279.15,
    nPorts=1) "Source for cooling coil loop" annotation (Placement(
        transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={21,-51})));
  Controls.SetPoints.OccupancySchedule           occSch(occupancy=3600*{6,19})
    "Occupancy schedule"
    annotation (Placement(transformation(extent={{-116,-32},{-102,-18}})));
  Modelica.Blocks.Sources.Constant SetAdj(k=0)
    annotation (Placement(transformation(extent={{-110,8},{-100,18}})));
  Controls.OBC.CDL.Reals.Sources.Constant TOccSetPoi(k=72 + 273.15)
    annotation (Placement(transformation(extent={{-116,-54},{-102,-40}})));
  Controls.OBC.CDL.Reals.Sources.Constant TUnOccCooSet(k=78 + 273.15)
    annotation (Placement(transformation(extent={{-114,-78},{-102,-66}})));
  Controls.OBC.CDL.Reals.Sources.Constant TUnOccHeaSet(k=65 + 273.15)
    annotation (Placement(transformation(extent={{-114,-100},{-100,-86}})));
  Modelica.Blocks.Sources.Constant EcoMod(k=1e-6)
                                               "Econ Mode control signal"
    annotation (Placement(transformation(extent={{-38,8},{-32,14}})));
  Controls.OBC.CDL.Conversions.RealToInteger reaToInt
    annotation (Placement(transformation(extent={{-114,-6},{-106,2}})));
  Modelica.Blocks.Sources.Constant LimLev(k=0)
    annotation (Placement(transformation(extent={{-130,-6},{-120,4}})));
  Controls.OBC.CDL.Reals.GreaterThreshold greThr
    annotation (Placement(transformation(extent={{46,-2},{58,10}})));
  Controls.OBC.CDL.Logical.Timer tim(t=120)
    annotation (Placement(transformation(extent={{64,0},{74,10}})));
  replaceable parameter Fluid.Movers.Data.Generic           fanPer
    constrainedby Fluid.Movers.Data.Generic
    "Record with performance data for supply fan"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{-102,56},{-92,66}})),
      Dialog(group="Fan parameters"));
  parameter Fluid.ZoneEquipment.FanCoilUnit.Validation.Data.FanData per
    annotation (Placement(transformation(extent={{-96,40},{-86,50}})));
protected
  Controls.OBC.CDL.Reals.Sources.Constant           cooWarTim(final k=0)
    "Cooldown and warm-up time"
    annotation (Placement(transformation(extent={{-114,24},{-106,32}})));
equation
  connect(conFCU.yFan, fanCoiUni.uFan) annotation (Line(points={{-41,2},{-14,2},
          {-14,0.8},{-10.9,0.8}},
                              color={0,0,127}));
  connect(conFCU.yCooCoi, fanCoiUni.uCoo) annotation (Line(points={{-41,-6},{
          -41,-4},{-12,-4},{-12,-2.8},{-10.9,-2.8}},
                                         color={0,0,127}));
  connect(conFCU.yHeaCoi, fanCoiUni.uHea) annotation (Line(points={{-41,-4},{
          -41,-6.4},{-10.9,-6.4}},      color={0,0,127}));
  connect(LimLev.y, reaToInt.u) annotation (Line(points={{-119.5,-1},{-119.5,-2},
          {-114.8,-2}}, color={0,0,127}));
  connect(greThr.y, tim.u) annotation (Line(points={{59.2,4},{62,4},{62,5},{63,
          5}}, color={255,0,255}));
  connect(fanCoiUni.TAirSup, conFCU.TSup) annotation (Line(points={{8.9,-5.32},
          {36,-5.32},{36,-58},{-76,-58},{-76,-7},{-63,-7}}, color={0,0,127}));
  connect(weaDat.weaBus, fanCoiUni.weaBus) annotation (Line(
      points={{-46,66},{-34,66},{-34,52},{-14,52},{-14,6.92},{-8.74,6.92}},
      color={255,204,51},
      thickness=0.5));

  connect(fanCoiUni.yFan_actual, greThr.u) annotation (Line(points={{8.45,6.2},{
          44.8,6.2},{44.8,4}}, color={0,0,127}));
  connect(fanCoiUni.port_Air_a, floor1.portsCor[2]) annotation (Line(points={{8,0.8},
          {22,0.8},{22,2},{39.8783,2},{39.8783,61.0154}},      color={0,127,255}));
  connect(fanCoiUni.port_Air_b, floor1.portsCor[1]) annotation (Line(points={{8,-2.8},
          {28,-2.8},{28,-4},{36.487,-4},{36.487,61.0154}},         color={0,127,
          255}));
  connect(weaDat.weaBus, floor1.weaBus) annotation (Line(
      points={{-46,66},{-32,66},{-32,64},{-18,64},{-18,88.7692},{57.1739,
          88.7692}},
      color={255,204,51},
      thickness=0.5));
  connect(sinHea.ports[1], fanCoiUni.port_HW_b) annotation (Line(points={{-17,
          -46},{-18,-46},{-18,-10},{-6.4,-10}}, color={0,127,255}));
  connect(fanCoiUni.port_HW_a, souHea.ports[1]) annotation (Line(points={{-3.7,
          -10},{-3.7,-42},{-3,-42},{-3,-46}}, color={0,127,255}));
  connect(sinCoo.ports[1], fanCoiUni.port_CHW_b) annotation (Line(points={{9,
          -46},{8,-46},{8,-22},{1.7,-22},{1.7,-10}}, color={0,127,255}));
  connect(fanCoiUni.port_CHW_a, souCoo.ports[1]) annotation (Line(points={{4.4,
          -10},{6,-10},{6,-22},{21,-22},{21,-46}}, color={0,127,255}));
  connect(cooWarTim.y, conFCU.warUpTim) annotation (Line(points={{-105.2,28},{
          -92,28},{-92,22},{-66,22},{-66,12},{-63,12}}, color={0,0,127}));
  connect(cooWarTim.y, conFCU.cooDowTim) annotation (Line(points={{-105.2,28},{
          -92,28},{-92,22},{-66,22},{-66,10},{-63,10}}, color={0,0,127}));
  connect(SetAdj.y, conFCU.setAdj) annotation (Line(points={{-99.5,13},{-70,13},
          {-70,5},{-63.1,5}}, color={0,0,127}));
  connect(occSch.tNexOcc, conFCU.tNexOcc) annotation (Line(points={{-101.3,
          -20.8},{-82,-20.8},{-82,8},{-63,8}}, color={0,0,127}));
  connect(occSch.occupied, conFCU.u1Occ) annotation (Line(points={{-101.3,-29.2},
          {-78,-29.2},{-78,-0.9},{-63,-0.9}}, color={255,0,255}));
  connect(reaToInt.y, conFCU.uCooDemLimLev) annotation (Line(points={{-105.2,-2},
          {-70,-2},{-70,-3},{-63,-3}}, color={255,127,0}));
  connect(reaToInt.y, conFCU.uHeaDemLimLev) annotation (Line(points={{-105.2,-2},
          {-70,-2},{-70,-4},{-66,-4},{-66,-5},{-63,-5}}, color={255,127,0}));
  connect(TOccSetPoi.y, conFCU.TOccHeaSet) annotation (Line(points={{-100.6,-47},
          {-72,-47},{-72,-11},{-63,-11}}, color={0,0,127}));
  connect(TOccSetPoi.y, conFCU.TOccCooSet) annotation (Line(points={{-100.6,-47},
          {-100.6,-48},{-72,-48},{-72,-12},{-68,-12},{-68,-13},{-63,-13}},
        color={0,0,127}));
  connect(TUnOccCooSet.y, conFCU.TUnoCooSet) annotation (Line(points={{-100.8,
          -72},{-68,-72},{-68,-17},{-63,-17}}, color={0,0,127}));
  connect(TUnOccHeaSet.y, conFCU.TUnoHeaSet) annotation (Line(points={{-98.6,
          -93},{-63,-93},{-63,-15}}, color={0,0,127}));
  connect(EcoMod.y, fanCoiUni.uEco) annotation (Line(points={{-31.7,11},{-10.9,
          11},{-10.9,4.4}}, color={0,0,127}));
  connect(tim.passed, conFCU.u1Fan) annotation (Line(points={{75,1},{78,1},{78,
          -10},{12,-10},{12,-16},{-36,-16},{-36,-30},{-66,-30},{-66,-21},{-63,
          -21}}, color={255,0,255}));
  connect(conFCU.TZon, floor1.TRooAir[5]) annotation (Line(points={{-63,-9},{
          -70,-9},{-70,-32},{94,-32},{94,61.3538},{87.6957,61.3538}}, color={0,
          0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=16500000,
      StopTime=16501000,
      Interval=10,
      __Dymola_Algorithm="Dassl"));
end FanCoilUnit;
