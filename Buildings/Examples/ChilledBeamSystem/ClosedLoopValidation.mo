within Buildings.Examples.ChilledBeamSystem;
model ClosedLoopValidation
  extends Modelica.Icons.Example;
  parameter Real schTab[5, 2] = [0, 0; 8, 1; 18, 1; 21, 0; 24, 0] "Table defining schedule for enabling plant";
  Buildings.Examples.ChilledBeamSystem.BaseClasses.TestBed chiBeaTesBed(
    TChiWatRet_nominal=273.15 + 25,
    mChiWatTot_flow_nominal=2.114,
    mAirTot_flow_nominal=1*0.676*1.225,
    mHotWatCoi_nominal=0.078,
    mChiWatCoi_nominal=0.645,
    mCooAir_flow_nominal=1*0.676*1.225,
    mHeaAir_flow_nominal=1*0.676*1.225,
    THeaWatInl_nominal=313.15,
    THeaWatOut_nominal=298.15,
    THeaAirInl_nominal=285.85,
    THeaAirDis_nominal=308.15,
    VRooSou=239.25,
    mChiWatSou_flow_nominal=0.387,
    mAirSou_flow_nominal=1*0.143*1.225,
    mAChiBeaSou_flow_nominal=0.143*1.225,
    VRooEas=103.31,
    mChiWatEas_flow_nominal=0.9,
    mAirEas_flow_nominal=1*0.065*1.225,
    mAChiBeaEas_flow_nominal=1*0.065*1.225,
    VRooNor=239.25,
    mChiWatNor_flow_nominal=0.253,
    mAirNor_flow_nominal=1*0.143*1.225,
    mAChiBeaNor_flow_nominal=0.143*1.225,
    VRooWes=103.31,
    mChiWatWes_flow_nominal=0.262,
    mAirWes_flow_nominal=1*0.065*1.225,
    mAChiBeaWes_flow_nominal=0.065*1.225,
    VRooCor=447.68,
    mChiWatCor_flow_nominal=0.27,
    mAirCor_flow_nominal=1*0.26*1.225,
    mAChiBeaCor_flow_nominal=0.26*1.225) "Chilled beam system test-bed"
    annotation (Placement(visible=true, transformation(
        origin={-76,-36},
        extent={{88,-16},{108,32}},
        rotation=0)));
  Buildings.Controls.OBC.ChilledBeams.Terminal.Controller terCon[5](TdCoo = {0.1, 100, 0.1, 0.1, 0.1}, TiCoo = fill(50, 5), VDes_occ = {0.143, 0.065, 0.143, 0.065, 0.26}, VDes_unoccSch = {0.028, 0.012, 0.028, 0.012, 0.052}, VDes_unoccUnsch = {0.056, 0.024, 0.056, 0.024, 0.104}, controllerTypeCoo = fill(Buildings.Controls.OBC.CDL.Types.SimpleController.PID, 5)) "Terminal controllers"
  annotation (
    Placement(visible = true, transformation(origin={32,14},   extent = {{10, 40}, {30, 72}}, rotation = 0)));
  Buildings.Controls.OBC.ChilledBeams.System.Controller sysCon(nPum = 1, nVal = 5, minPumSpe = 0.7,
    maxPumSpe=1,                                                                                    TiPumSpe = 50,
    dPChiWatMax=1,                                                                                                 kBypVal = 10e-3, TiBypVal = 900,
    chiWatStaPreMax=1,
    chiWatStaPreMin=1,
    triAmoVal=0,
    resAmoVal=1,
    maxResVal=1,
    samPerVal=1,
    delTimVal=60)                                                                                                                                   annotation (
    Placement(visible = true, transformation(origin={112,-20},   extent = {{10, -70}, {30, -50}}, rotation = 0)));
  Buildings.Controls.OBC.FDE.DOAS.Controller DOAScon(
    dehumSet=0.6,
    dehumOff=0.05,
    timThrDehDis=1200,
    timDelDehEna=180,
    timThrDehEna=600,
    controllerTypeExhFan=Buildings.Controls.OBC.CDL.Types.SimpleController.P,
    kCoiHea=0.0000001,
    TiCoiHea=0.001,
    TdCoiHea=0.001,
    is_vav=true,
    kFanSpe=0.005,
    TdFanSpe=0,
    TiFanSpe=10,
    controllerTypeFanSpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI)
    annotation (Placement(visible = true,
      transformation(extent={{-39,-70},{-19,-6}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax TZonMax(nin=5)   annotation (
    Placement(transformation(extent={{120,-4},{140,16}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax yDamPosMax(nin=5)   annotation (
    Placement(transformation(extent={{-52,16},{-32,36}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable enaSch(final table = schTab, final smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, final timeScale = 3600) "Table defining when occupancy is expected" annotation (
    Placement(transformation(extent={{-144,70},{-124,90}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow = 0.45, uHigh = 0.5) annotation (
    Placement(transformation(extent = {{-120, 70}, {-100, 90}})));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booRep(nout = 5) annotation (
    Placement(transformation(extent = {{-90, 70}, {-70, 90}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant uConSig[5](k = fill(false, 5)) "Constant Boolean source" annotation (
    Placement(visible = true, transformation(origin={-2,14},   extent = {{-90, 30}, {-70, 50}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant chiWatSupTem(k = 273.15 + 7.22) "Chilled water supply temperature" annotation (
    Placement(transformation(extent={{-150,-24},{-130,-4}})));
  Modelica.Blocks.Sources.CombiTimeTable loads(
    tableOnFile = true,
    tableName = "tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "./Buildings/Resources/Data/Examples/ChilledBeamSystem/zoneLoads.txt"),
    columns = {2, 3, 4, 5, 6},
    timeScale = 60)
    "Table defining thermal loads for zone"
    annotation (Placement(transformation(extent={{-154,-80},{-134,-60}})));
  Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy = 3600*{8, 18}) annotation (
    Placement(visible = true, transformation(origin={120,132}, extent = {{-152, -44}, {-132, -24}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooHea(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 15 + 273.15; 8*3600, 20 + 273.15; 18*3600, 15 + 273.15; 24*3600, 15 + 273.15]) annotation (
    Placement(visible = true, transformation(origin={-8,-28},   extent = {{-152, 40}, {-132, 60}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooCoo(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 30 + 273.15; 8*3600, 25 + 273.15; 18*3600, 30 + 273.15; 24*3600, 30 + 273.15]) annotation (
    Placement(visible = true, transformation(origin={-4,32},     extent = {{-152, 10}, {-132, 30}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booleanScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin={98,14},      extent = {{-90, 70}, {-70, 90}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout = 5)  annotation (
    Placement(visible = true, transformation(origin={-48,64},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator realScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin={-110,34},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Controls.OBC.CDL.Reals.Sources.Constant ERW_drybulb_temp(k=295)
    annotation (Placement(transformation(extent={{-152,-52},{-132,-32}})));
equation
  connect(enaSch.y[1], hys.u) annotation (
    Line(points={{-122,80},{-122,80}},      color = {0, 0, 127}));
  connect(hys.y, booRep.u) annotation (
    Line(points = {{-98, 80}, {-92, 80}}, color = {255, 0, 255}));
//connect(booRep.y, terCon.uDetOcc) annotation(
//Line(points = {{-68, 80}, {-4, 80}, {-4, 58}, {8, 58}}, color = {255, 0, 255}));
  connect(uConSig.y, terCon.uConSen) annotation (
    Line(points={{-70,54},{-70,48},{-4,48},{-4,68},{40,68}},            color = {255, 0, 255}));
  connect(chiBeaTesBed.TZon, terCon.TZon) annotation (
    Line(points={{34,-8},{34,-2},{78,-2},{78,34},{34,34},{34,64},{40,64}},
                                                            color = {0, 0, 127}, thickness = 0.5));
  connect(booRep.y, terCon.uOccExp) annotation (
    Line(points={{-68,80},{-8,80},{-8,80},{40,80}},          color = {255, 0, 255}, thickness = 0.5));
  connect(TSetRooHea.y[1], reaScaRep.u) annotation (
    Line(points={{-138,22},{-72,22},{-72,40},{-64,40},{-64,42},{-66,42},{-66,62},
          {-60,62},{-60,64}},                                     color = {0, 0, 127}));
  connect(TSetRooCoo.y[1], realScalarReplicator.u) annotation (
    Line(points={{-134,52},{-128,52},{-128,42},{-126,42},{-126,34},{-122,34}},
                                            color = {0, 0, 127}));
  connect(realScalarReplicator.y, terCon.TZonCooSet) annotation (
    Line(points={{-98,34},{-70,34},{-70,38},{-62,38},{-62,44},{-46,44},{-46,46},
          {-8,46},{-8,72},{40,72}},                          color = {0, 0, 127}, thickness = 0.5));
  connect(chiBeaTesBed.VDisAir_flow, terCon.VDis_flow) annotation (
    Line(points={{34,-10},{34,-4},{40,-4},{40,42},{38,42},{38,60},{40,60}},
                                                          color = {0, 0, 127}, thickness = 0.5));
  connect(booleanScalarReplicator.y, terCon.uOccDet) annotation (Line(points={{30,94},
          {40,94},{40,84}},                                          color={255,
          0,255}));
  connect(sysCon.yBypValPos, chiBeaTesBed.uBypValPos) annotation (Line(points={{144,-86},
          {150,-86},{150,-10},{60,-10},{60,16},{-8,16},{-8,-10},{9.8,-10}},
                                                                  color={0,0,
          127}));
  connect(chiBeaTesBed.uPumSpe, sysCon.yPumSpe) annotation (Line(points={{9.8,-46},
          {-8,-46},{-8,-68},{104,-68},{104,-96},{154,-96},{154,-80},{144,-80}},
                                                                   color={0,0,
          127}));
  connect(chiBeaTesBed.yChiWatVal, sysCon.uValPos) annotation (Line(points={{34,-16},
          {82,-16},{82,-66},{108,-66},{108,-86},{120,-86}},
                                             color={0,0,127}));
  connect(chiBeaTesBed.dPChiWat, sysCon.dPChiWatLoo) annotation (Line(points={{34,-30},
          {112,-30},{112,-80},{120,-80}},                           color={0,0,
          127}));
  connect(yDamPosMax.y, DOAScon.uDamMaxOpe) annotation (Line(points={{-30,26},{
          -30,14},{-74,14},{-74,8},{-76,8},{-76,-20},{-41,-20}},
                                                              color={0,0,127}));
  connect(TZonMax.y, DOAScon.TAirHig) annotation (Line(points={{142,6},{142,36},
          {-22,36},{-22,10},{-72,10},{-72,-36},{-41,-36}},                color=
         {0,0,127}));
  connect(chiWatSupTem.y, chiBeaTesBed.TChiWatSup) annotation (Line(points={{-128,
          -14},{-68,-14},{-68,4},{-12,4},{-12,-49.9},{9.7,-49.9}},
                                                          color={0,0,127}));
  connect(chiBeaTesBed.erwsuphum, DOAScon.phiAirEneRecWhe) annotation (Line(
        points={{34,-38},{50,-38},{50,-42},{66,-42},{66,-50},{46,-50},{46,-62},
          {-8,-62},{-8,-76},{-68,-76},{-68,-54},{-54,-54},{-54,-58},{-48,-58},{
          -48,-56},{-41,-56}},
        color={0,0,127}));
  connect(DOAScon.TAirSup, DOAScon.TAirDisCoiCoo) annotation (Line(points={{-41,-40},
          {-41,-38},{-57,-38},{-57,-52},{-41,-52}},      color={0,0,127}));
  connect(chiBeaTesBed.rAT, DOAScon.TAirRet) annotation (Line(points={{34,-42},
          {38,-42},{38,-84},{-2,-84},{-2,-90},{-44,-90},{-44,-84},{-46,-84},{
          -46,-44},{-41,-44}},       color={0,0,127}));
  connect(DOAScon.TAirOut, chiBeaTesBed.OutdoorAirTemp) annotation (Line(points={{-41,-48},
          {-64,-48},{-64,-92},{-50,-92},{-50,-84},{24,-84},{24,-76},{74,-76},{
          74,-44},{34,-44}},                    color={0,0,127}));
  connect(chiBeaTesBed.bldgSP, DOAScon.dPAirStaBui) annotation (Line(points={{34,-46},
          {42,-46},{42,-54},{50,-54},{50,-72},{-41,-72},{-41,-68}},
        color={0,0,127}));
  connect(DOAScon.uFanSupPro, chiBeaTesBed.yFanSta) annotation (Line(points={{-41,-24},
          {-84,-24},{-84,12},{58,12},{58,-26},{46,-26},{46,-28},{34,-28}},
        color={255,0,255}));
  connect(chiBeaTesBed.exhFanSta, DOAScon.uFanExhPro) annotation (Line(points={{34,-26},
          {44,-26},{44,-22},{54,-22},{54,-32},{70,-32},{70,-58},{-4,-58},{-4,
          -98},{-68,-98},{-68,-64},{-41,-64}},
                                       color={255,0,255}));
  connect(TZonMax.u[1:5], chiBeaTesBed.TZon) annotation (Line(points={{118,6.8},
          {104,6.8},{104,6},{88,6},{88,-8},{34,-8}},      color={0,0,127}));
  connect(yDamPosMax.u[1:5], chiBeaTesBed.yDamPos) annotation (Line(points={{-54,
          26.8},{-66,26.8},{-66,8},{66,8},{66,-18},{34,-18},{34,-20}},
        color={0,0,127}));
  connect(chiBeaTesBed.yChiWatVal, terCon.uChiVal) annotation (Line(points={{34,-16},
          {50,-16},{50,52},{40,52},{40,56}},
        color={0,0,127}));
  connect(chiBeaTesBed.uPumSta, sysCon.yChiWatPum[1]) annotation (Line(points={{9.8,-42},
          {-4,-42},{-4,-68},{118,-68},{118,-60},{144,-60},{144,-74}},
        color={255,0,255}));
  connect(chiBeaTesBed.uCAVReh, terCon.yReh) annotation (Line(points={{9.8,-22},
          {-6,-22},{-6,-14},{-8,-14},{-8,-6},{-6,-6},{-6,42},{112,42},{112,78},
          {64,78}},                                    color={0,0,127}));
  connect(terCon.yChiVal, chiBeaTesBed.uChiWatVal) annotation (Line(points={{64,74},
          {102,74},{102,22},{-8,22},{-8,0},{-10,0},{-10,-14},{9.8,-14}},
        color={0,0,127}));
  connect(terCon.yDam, chiBeaTesBed.uCAVDam) annotation (Line(points={{64,70},{
          96,70},{96,26},{-10,26},{-10,-18},{9.8,-18}},               color={0,
          0,127}));
  connect(loads.y, chiBeaTesBed.QFlo) annotation (Line(points={{-133,-70},{-76,
          -70},{-76,-24},{-64,-24},{-64,2},{-18,2},{-18,-4},{-4,-4},{-4,-6},{
          9.8,-6}},              color={0,0,127}));
  connect(DOAScon.yFanSup, chiBeaTesBed.uFanSta) annotation (Line(points={{-17,-22},
          {-12,-22},{-12,-25.8},{9.8,-25.8}},          color={255,0,255}));
  connect(reaScaRep.y, terCon.TZonHeaSet) annotation (Line(points={{-36,64},{
          -10,64},{-10,76},{40,76}},             color={0,0,127}));
  connect(chiBeaTesBed.relHumDOASRet, DOAScon.phiAirRet) annotation (Line(
        points={{34,-40},{68,-40},{68,6},{-66,6},{-66,-20},{-64,-20},{-64,-32},
          {-41,-32}},
        color={0,0,127}));
  connect(ERW_drybulb_temp.y, DOAScon.TAirSupEneWhe) annotation (Line(points={{-130,
          -42},{-70,-42},{-70,-60},{-41,-60}},                color={0,0,127}));
  connect(DOAScon.yEneRecWheSpe, chiBeaTesBed.uEneRecWheSpe) annotation (Line(
        points={{-17,-46},{-10,-46},{-10,-28},{9.8,-28}},              color={0,
          0,127}));
  connect(chiBeaTesBed.uFanSpe, DOAScon.yFanSupSpe) annotation (Line(points={{9.8,
          -30.2},{9.8,-26},{-17,-26}},
        color={0,0,127}));
  connect(DOAScon.yCoiHea, chiBeaTesBed.uHeaCoi) annotation (Line(points={{-17,-34},
          {9.8,-34}},                                 color={0,0,127}));
  connect(DOAScon.yCoiCoo, chiBeaTesBed.uCooCoi) annotation (Line(points={{-17,-30},
          {-10,-30},{-10,-38.2},{9.8,-38.2}},         color={0,0,127}));
  connect(TSetRooHea.y[1], DOAScon.TZonHeaSet) annotation (Line(points={{-138,22},
          {-98,22},{-98,-16},{-74,-16},{-74,-12},{-41,-12}},       color={0,0,
          127}));
  connect(TSetRooCoo.y[1], DOAScon.TZonCooSet) annotation (Line(points={{-134,52},
          {-94,52},{-94,18},{-70,18},{-70,-8},{-41,-8}},
                                                      color={0,0,127}));
  connect(occSch.occupied, booleanScalarReplicator.u) annotation (Line(points={
          {-11,92},{-2,92},{-2,94},{6,94}}, color={255,0,255}));
  connect(hys.y, DOAScon.Occ) annotation (Line(points={{-98,80},{-96,80},{-96,
          -16},{-41,-16}}, color={255,0,255}));
  connect(chiBeaTesBed.yPumSta, sysCon.uPumSta[1]) annotation (Line(points={{34,
          -34},{50,-34},{50,-36},{106,-36},{106,-60},{116,-60},{116,-74},{120,
          -74}}, color={255,0,255}));
  connect(chiBeaTesBed.TDOASDis, DOAScon.TAirDisCoiCoo) annotation (Line(points
        ={{34,-36},{44,-36},{44,-44},{54,-44},{54,-78},{-58,-78},{-58,-56},{-44,
          -56},{-44,-52},{-41,-52}}, color={0,0,127}));
  connect(chiBeaTesBed.dPDOASAir, DOAScon.dPAirDucSta) annotation (Line(points=
          {{34,-22.8},{38,-22.8},{38,-20},{74,-20},{74,14},{-42,14},{-42,-2},{
          -54,-2},{-54,-28},{-41,-28}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
    experiment(
      StartTime=19180800,
      StopTime=19440000,
      Interval=600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"),
    Documentation(info="<html>
<p>Simulates Buildings.Examples.ChilledBeamSystem.BaseClasses.TestBed.</p>
<p>It consists of a closed loop system for the chilled beam system block, <span style=\"font-family: Courier New;\">chiBeaTesBed. </span>Uses
<code>sysCon</code> to operate the main chilled beam system and <code>terCon</code> to operate the zone CAV terminal box and the zone chilled beam manifold control valve.
 </p>
</html>"));
end ClosedLoopValidation;
