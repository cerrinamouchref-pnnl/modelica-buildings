within Buildings.Examples.ChilledBeamSystem;
model ClosedLoopValidation
  extends Modelica.Icons.Example;
  parameter Real schTab[5, 2] = [0, 0; 8, 1; 18, 1; 21, 0; 24, 0] "Table defining schedule for enabling plant";
  Buildings.Examples.ChilledBeamSystem.BaseClasses.TestBed chiBeaTesBed(TChiWatRet_nominal = 273.15 + 25, mChiWatTot_flow_nominal = 2.114, mAirTot_flow_nominal = 1*0.676*1.225, mHotWatCoi_nominal = 0.078, mChiWatCoi_nominal = 0.645,
    mCooAir_flow_nominal=1*0.676*1.225,
    mHeaAir_flow_nominal=1*0.676*1.225,
    THeaWatInl_nominal=313.15,
    THeaWatOut_nominal=298.15,
    THeaAirInl_nominal=285.85,
    THeaAirDis_nominal=308.15,                                                                                                                                                                                                     VRooSou = 239.25, mChiWatSou_flow_nominal = 0.387, mAirSou_flow_nominal = 1*0.143*1.225, mAChiBeaSou_flow_nominal = 0.143*1.225, VRooEas = 103.31, mChiWatEas_flow_nominal = 0.9, mAirEas_flow_nominal = 1*0.065*1.225, mAChiBeaEas_flow_nominal = 1*0.065*1.225, VRooNor = 239.25, mChiWatNor_flow_nominal = 0.253, mAirNor_flow_nominal = 1*0.143*1.225, mAChiBeaNor_flow_nominal = 0.143*1.225, VRooWes = 103.31, mChiWatWes_flow_nominal = 0.262, mAirWes_flow_nominal = 1*0.065*1.225, mAChiBeaWes_flow_nominal = 0.065*1.225, VRooCor = 447.68, mChiWatCor_flow_nominal = 0.27, mAirCor_flow_nominal = 1*0.26*1.225, mAChiBeaCor_flow_nominal = 0.26*1.225) "Chilled beam system test-bed" annotation (
    Placement(visible = true, transformation(origin={-98,-24},    extent = {{88, -16}, {108, 32}}, rotation = 0)));
  Buildings.Controls.OBC.ChilledBeams.Terminal.Controller terCon[5](TdCoo = {0.1, 100, 0.1, 0.1, 0.1}, TiCoo = fill(50, 5), VDes_occ = {0.143, 0.065, 0.143, 0.065, 0.26}, VDes_unoccSch = {0.028, 0.012, 0.028, 0.012, 0.052}, VDes_unoccUnsch = {0.056, 0.024, 0.056, 0.024, 0.104}, controllerTypeCoo = fill(Buildings.Controls.OBC.CDL.Types.SimpleController.PID, 5)) "Terminal controllers" annotation (
    Placement(visible = true, transformation(origin={32,14},   extent = {{10, 40}, {30, 60}}, rotation = 0)));
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
  Buildings.Controls.OBC.FDE.DOAS.DOAScontroller DOAScon   annotation (
    Placement(visible = true, transformation(origin={-39,-10},  extent = {{-13, -24}, {13, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.MultiMax TZonMax(nin=5)   annotation (
    Placement(transformation(extent={{-40,52},{-20,72}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax yDamPosMax(nin=5)   annotation (
    Placement(transformation(extent={{-58,20},{-38,40}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable enaSch(final table = schTab, final smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, final timeScale = 3600) "Table defining when occupancy is expected" annotation (
    Placement(transformation(extent = {{-150, 70}, {-130, 90}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow = 0.45, uHigh = 0.5) annotation (
    Placement(transformation(extent = {{-120, 70}, {-100, 90}})));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booRep(nout = 5) annotation (
    Placement(transformation(extent = {{-90, 70}, {-70, 90}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant uConSig[5](k = fill(false, 5)) "Constant Boolean source" annotation (
    Placement(visible = true, transformation(origin={2,14},    extent = {{-90, 30}, {-70, 50}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant chiWatSupTem(k = 273.15 + 7.22) "Chilled water supply temperature" annotation (
    Placement(transformation(extent={{-148,-26},{-128,-6}})));
  Modelica.Blocks.Sources.CombiTimeTable loads(
    tableOnFile = true,
    tableName = "tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "./Buildings/Resources/Data/Examples/ChilledBeamSystem/zoneLoads.txt"),
    columns = {2, 3, 4, 5, 6},
    timeScale = 60)
    "Table defining thermal loads for zone"
    annotation (Placement(transformation(extent={{-160,-60},{-140,-40}})));
  Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy = 3600*{8, 18}) annotation (
    Placement(visible = true, transformation(origin={-6,74},   extent = {{-152, -44}, {-132, -24}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooHea(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 15 + 273.15; 8*3600, 20 + 273.15; 18*3600, 15 + 273.15; 24*3600, 15 + 273.15]) annotation (
    Placement(visible = true, transformation(origin = {72, 64}, extent = {{-152, 40}, {-132, 60}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooCoo(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 30 + 273.15; 8*3600, 25 + 273.15; 18*3600, 30 + 273.15; 24*3600, 30 + 273.15]) annotation (
    Placement(visible = true, transformation(origin = {74, 122}, extent = {{-152, 10}, {-132, 30}}, rotation = 0)));
  Buildings.Controls.OBC.FDE.DOAS.Validation.Baseclasses.erwTsim eRWtemp
    annotation (Placement(visible=true, transformation(
        origin={-82,-78},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Buildings.Controls.OBC.CDL.Reals.Subtract sub annotation (
    Placement(visible = true, transformation(origin={6,-82},      extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booleanScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin={-36,-42},    extent = {{-90, 70}, {-70, 90}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout = 5)  annotation (
    Placement(visible = true, transformation(origin = {-26, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator realScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin = {-30, 142}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(DOAScon.yFanSupSpe, chiBeaTesBed.uFanSpe) annotation (Line(points={{-23.14,
          -7.8},{-15.5,-7.8},{-15.5,-18},{-12,-18}}, color={0,0,127}));
  connect(enaSch.y[1], hys.u) annotation (
    Line(points = {{-128, 80}, {-122, 80}}, color = {0, 0, 127}));
  connect(hys.y, booRep.u) annotation (
    Line(points = {{-98, 80}, {-92, 80}}, color = {255, 0, 255}));
//connect(booRep.y, terCon.uDetOcc) annotation(
//Line(points = {{-68, 80}, {-4, 80}, {-4, 58}, {8, 58}}, color = {255, 0, 255}));
  connect(uConSig.y, terCon.uConSen) annotation (
    Line(points={{-66,54},{-66,48},{-4,48},{-4,62.75},{40,62.75}},      color = {255, 0, 255}));
  connect(hys.y,DOAScon.Occ)  annotation (
    Line(points={{-98,80},{-96,80},{-96,-0.4},{-54.6,-0.4}},color = {255, 0, 255}));
  connect(chiBeaTesBed.TZon, terCon.TZon) annotation (
    Line(points={{12,4.8},{20,4.8},{20,60.25},{40,60.25}},  color = {0, 0, 127}, thickness = 0.5));
  connect(eRWtemp.yTSimEneRecWhe, sub.u2) annotation (Line(points={{-69.6,-78},
          {-30,-78},{-30,-88},{-6,-88}}, color={0,0,127}));
  connect(booRep.y, terCon.uOccExp) annotation (
    Line(points={{-68,80},{-8,80},{-8,70.25},{40,70.25}},    color = {255, 0, 255}, thickness = 0.5));
  connect(TSetRooHea.y[1], reaScaRep.u) annotation (
    Line(points = {{-58, 114}, {-50, 114}, {-50, 98}, {-38, 98}}, color = {0, 0, 127}));
  connect(TSetRooCoo.y[1], realScalarReplicator.u) annotation (
    Line(points = {{-56, 142}, {-42, 142}}, color = {0, 0, 127}));
  connect(realScalarReplicator.y, terCon.TZonCooSet) annotation (
    Line(points={{-18,142},{8,142},{8,65.25},{40,65.25}},    color = {0, 0, 127}, thickness = 0.5));
  connect(chiBeaTesBed.VDisAir_flow, terCon.VDis_flow) annotation (
    Line(points={{12,2.2},{22,2.2},{22,57.75},{40,57.75}},color = {0, 0, 127}, thickness = 0.5));
  connect(occSch.occupied, booleanScalarReplicator.u) annotation (Line(points={
          {-137,34},{-134,34},{-134,38},{-128,38}}, color={255,0,255}));
  connect(booleanScalarReplicator.y, terCon.uOccDet) annotation (Line(points={{
          -104,38},{-102,38},{-102,12},{6,12},{6,72.75},{40,72.75}}, color={255,
          0,255}));
  connect(sysCon.yBypValPos, chiBeaTesBed.uBypValPos) annotation (Line(points={
          {144,-86},{148,-86},{148,16},{-20,16},{-20,2},{-12,2}}, color={0,0,
          127}));
  connect(chiBeaTesBed.uPumSpe, sysCon.yPumSpe) annotation (Line(points={{-12,
          -34},{-16,-34},{-16,-60},{156,-60},{156,-80},{144,-80}}, color={0,0,
          127}));
  connect(chiBeaTesBed.yChiWatVal, sysCon.uValPos) annotation (Line(points={{12,
          -4},{106,-4},{106,-86},{120,-86}}, color={0,0,127}));
  connect(chiBeaTesBed.dPChiWat, sysCon.dPChiWatLoo) annotation (Line(points={{
          12,-20},{58,-20},{58,-24},{102,-24},{102,-80},{120,-80}}, color={0,0,
          127}));
  connect(chiBeaTesBed.yPumSta, sysCon.uPumSta[1]) annotation (Line(points={{12,
          -23.2},{50,-23.2},{50,-30},{98,-30},{98,-74},{120,-74}}, color={255,0,
          255}));
  connect(DOAScon.yBypDam, eRWtemp.uBypDam) annotation (Line(points={{-23.14,-17},
          {-20,-17},{-20,-42},{-102,-42},{-102,-72},{-94.4,-72}}, color={255,0,
          255}));
  connect(chiBeaTesBed.OutdoorAirTemp, eRWtemp.TAirOut) annotation (Line(points=
         {{12,-37.6},{20,-37.6},{20,-40},{26,-40},{26,-96},{-98,-96},{-98,-84},
          {-94.4,-84}}, color={0,0,127}));
  connect(chiBeaTesBed.OutdoorAirTemp, sub.u1) annotation (Line(points={{12,
          -37.6},{14,-37.6},{14,-64},{-10,-64},{-10,-76},{-6,-76}}, color={0,0,
          127}));
  connect(yDamPosMax.y, DOAScon.uDamMaxOpe) annotation (Line(points={{-36,30},{
          -30,30},{-30,16},{-72,16},{-72,-2.8},{-54.6,-2.8}}, color={0,0,127}));
  connect(TZonMax.y, DOAScon.TAirHig) annotation (Line(points={{-18,62},{-16,62},
          {-16,34},{-26,34},{-26,18},{-80,18},{-80,-12.6},{-54.6,-12.6}}, color=
         {0,0,127}));
  connect(chiWatSupTem.y, chiBeaTesBed.TChiWatSup) annotation (Line(points={{
          -126,-16},{-86,-16},{-86,-37.1},{-12.1,-37.1}}, color={0,0,127}));
  connect(chiBeaTesBed.TDOASDis, DOAScon.TAirDisCoiCoo) annotation (Line(points=
         {{12,-26},{22,-26},{22,-50},{-24,-50},{-24,-34},{-74,-34},{-74,-22.6},
          {-54.6,-22.6}}, color={0,0,127}));
  connect(chiBeaTesBed.dPDOASAir, DOAScon.dPAirDucSta) annotation (Line(points=
          {{12,-10.8},{30,-10.8},{30,-10},{50,-10},{50,24},{-28,24},{-28,6},{-62,
          6},{-62,-7.6},{-54.6,-7.6}}, color={0,0,127}));
  connect(chiBeaTesBed.erwsuphum, DOAScon.phiAirEneRecWhe) annotation (Line(
        points={{12,-28.6},{16,-28.6},{16,-66},{-70,-66},{-70,-25.2},{-54.6,-25.2}},
        color={0,0,127}));
  connect(eRWtemp.yTSimEneRecWhe, DOAScon.TAirSupEneWhe) annotation (Line(
        points={{-69.6,-78},{-64,-78},{-64,-27.6},{-54.6,-27.6}}, color={0,0,
          127}));
  connect(DOAScon.TAirSup, DOAScon.TAirDisCoiCoo) annotation (Line(points={{-54.6,
          -15.2},{-72,-15.2},{-72,-22.6},{-54.6,-22.6}}, color={0,0,127}));
  connect(chiBeaTesBed.rAT, DOAScon.TAirRet) annotation (Line(points={{12,-34.4},
          {36,-34.4},{36,-34},{46,-34},{46,-94},{-106,-94},{-106,-17.6},{-54.6,
          -17.6}},                   color={0,0,127}));
  connect(DOAScon.TAirOut, chiBeaTesBed.OutdoorAirTemp) annotation (Line(points=
         {{-54.6,-20.2},{-92,-20.2},{-92,-56},{-52,-56},{-52,-72},{-12,-72},{-12,
          -48},{18,-48},{18,-37.6},{12,-37.6}}, color={0,0,127}));
  connect(chiBeaTesBed.bldgSP, DOAScon.dPAirStaBui) annotation (Line(points={{
          12,-40},{12,-46},{-14,-46},{-14,-40},{-58,-40},{-58,-32.4},{-54.6,-32.4}},
        color={0,0,127}));
  connect(DOAScon.uFanSupPro, chiBeaTesBed.yFanSta) annotation (Line(points={{-54.6,
          -5.2},{-58,-5.2},{-58,8},{-16,8},{-16,20},{18,20},{18,-16.8},{12,-16.8}},
        color={255,0,255}));
  connect(chiBeaTesBed.exhFanSta, DOAScon.uFanExhPro) annotation (Line(points={
          {12,-14},{20,-14},{20,-16},{28,-16},{28,-54},{-22,-54},{-22,-44},{-62,
          -44},{-62,-30},{-54.6,-30}}, color={255,0,255}));
  connect(TZonMax.u[1:5], chiBeaTesBed.TZon) annotation (Line(points={{-42,60.4},
          {-44,60.4},{-44,46},{16,46},{16,4.8},{12,4.8}}, color={0,0,127}));
  connect(yDamPosMax.u[1:5], chiBeaTesBed.yDamPos) annotation (Line(points={{
          -60,28.4},{-60,50},{-50,50},{-50,48},{26,48},{26,-7.8},{12,-7.8}},
        color={0,0,127}));
  connect(chiBeaTesBed.yChiWatVal, terCon.uChiVal) annotation (Line(points={{12,
          -4},{22,-4},{22,0},{36,0},{36,48},{38,48},{38,55.25},{40,55.25}},
        color={0,0,127}));
  connect(DOAScon.yCoiCoo, chiBeaTesBed.uCooCoi) annotation (Line(points={{-23.14,
          -11},{-18,-11},{-18,-26},{-12,-26}}, color={0,0,127}));
  connect(DOAScon.yEneRecWheEna, eRWtemp.uEneRecWheStart) annotation (Line(
        points={{-23.14,-20},{-22,-20},{-22,-76},{-62,-76},{-62,-50},{-112,-50},
          {-112,-78},{-94.4,-78},{-94.4,-76}}, color={255,0,255}));
  connect(DOAScon.yCoiHea, chiBeaTesBed.uHeaCoi) annotation (Line(points={{-23.14,
          -14},{-18,-14},{-18,-22},{-12,-22}}, color={0,0,127}));
  connect(chiBeaTesBed.uPumSta, sysCon.yChiWatPum[1]) annotation (Line(points={
          {-12,-30},{-18,-30},{-18,-62},{76,-62},{76,-40},{144,-40},{144,-74}},
        color={255,0,255}));
  connect(chiBeaTesBed.uCAVReh, terCon.yReh) annotation (Line(points={{-12,-10},
          {-22,-10},{-22,42},{92,42},{92,69},{64,69}}, color={0,0,127}));
  connect(terCon.yChiVal, chiBeaTesBed.uChiWatVal) annotation (Line(points={{64,
          66.5},{76,66.5},{76,66},{84,66},{84,32},{-16,32},{-16,-2},{-12,-2}},
        color={0,0,127}));
  connect(terCon.yDam, chiBeaTesBed.uCAVDam) annotation (Line(points={{64,64},{
          72,64},{72,62},{78,62},{78,36},{-20,36},{-20,-6},{-12,-6}}, color={0,
          0,127}));
  connect(loads.y, chiBeaTesBed.QFlo) annotation (Line(points={{-139,-50},{-14,-50},
          {-14,6},{-12,6}},      color={0,0,127}));
  connect(sub.y, chiBeaTesBed.deltaT) annotation (Line(points={{18,-82},{20,-82},
          {20,-98},{-136,-98},{-136,-38},{-104,-38},{-104,6},{-68,6},{-68,10},{
          -24,10},{-24,-2},{-18,-2},{-18,-4},{-12,-4}}, color={0,0,127}));
  connect(DOAScon.yFanSup, chiBeaTesBed.uFanSta) annotation (Line(points={{-23.14,
          -4.6},{-17.57,-4.6},{-17.57,-14},{-12,-14}}, color={255,0,255}));
  connect(reaScaRep.y, terCon.TZonHeaSet) annotation (Line(points={{-14,98},{0,98},
          {0,68},{20,68},{20,67.75},{40,67.75}}, color={0,0,127}));
  connect(chiBeaTesBed.relHumDOASRet, DOAScon.phiAirRet) annotation (Line(
        points={{12,-31},{38,-31},{38,-32},{56,-32},{56,-58},{-94,-58},{-94,-10},
          {-54.6,-10}}, color={0,0,127}));
  connect(chiBeaTesBed.rAT, eRWtemp.TAirRet) annotation (Line(points={{12,-34.4},
          {60,-34.4},{60,-92},{-108,-92},{-108,-80},{-94.4,-80}}, color={0,0,
          127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
    experiment(StartTime = 16848000, StopTime = 16934400, Interval = 600, Tolerance = 1e-06, __Dymola_Algorithm = "Cvode"));
end ClosedLoopValidation;
