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
        origin={-12,-20},
        extent={{88,-16},{108,32}},
        rotation=0)));
  Buildings.Controls.OBC.ChilledBeams.Terminal.Controller terCon[5](TdCoo = {0.1, 100, 0.1, 0.1, 0.1}, TiCoo = fill(50, 5), VDes_occ = {0.143, 0.065, 0.143, 0.065, 0.26}, VDes_unoccSch = {0.028, 0.012, 0.028, 0.012, 0.052}, VDes_unoccUnsch = {0.056, 0.024, 0.056, 0.024, 0.104}, controllerTypeCoo = fill(Buildings.Controls.OBC.CDL.Types.SimpleController.PID, 5)) "Terminal controllers"
  annotation (
    Placement(visible = true, transformation(origin={-52,-62}, extent = {{10, 40}, {30, 72}}, rotation = 0)));
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
    Placement(visible = true, transformation(origin={-46,12},    extent = {{10, -70}, {30, -50}}, rotation = 0)));
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
      transformation(extent={{15,-50},{35,14}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax TZonMax(nin=5)   annotation (
    Placement(transformation(extent={{128,-4},{148,16}})));
  Buildings.Controls.OBC.CDL.Reals.MultiMax yDamPosMax(nin=5)   annotation (
    Placement(transformation(extent={{128,32},{148,52}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable enaSch(final table = schTab, final smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, final timeScale = 3600) "Table defining when occupancy is expected" annotation (
    Placement(transformation(extent={{-142,48},{-122,68}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow = 0.45, uHigh = 0.5) annotation (
    Placement(transformation(extent={{-108,44},{-88,64}})));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booRep(nout = 5) annotation (
    Placement(transformation(extent={{-78,48},{-58,68}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant uConSig[5](k = fill(false, 5)) "Constant Boolean source" annotation (
    Placement(visible = true, transformation(origin={-14,-96}, extent = {{-90, 30}, {-70, 50}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant chiWatSupTem(k = 273.15 + 7.22) "Chilled water supply temperature" annotation (
    Placement(transformation(extent={{-142,-34},{-122,-14}})));
  Modelica.Blocks.Sources.CombiTimeTable loads(
    tableOnFile = true,
    tableName = "tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "./Buildings/Resources/Data/Examples/ChilledBeamSystem/zoneLoads.txt"),
    columns = {2, 3, 4, 5, 6},
    timeScale = 60)
    "Table defining thermal loads for zone"
    annotation (Placement(transformation(extent={{-142,-98},{-122,-78}})));
  Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy = 3600*{8, 18}) annotation (
    Placement(visible = true, transformation(origin={10,120},  extent = {{-152, -44}, {-132, -24}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooHea(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 15 + 273.15; 8*3600, 20 + 273.15; 18*3600, 15 + 273.15; 24*3600, 15 + 273.15]) annotation (
    Placement(visible = true, transformation(origin={10,-46},   extent = {{-152, 40}, {-132, 60}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable TSetRooCoo(extrapolation = Buildings.Controls.OBC.CDL.Types.Extrapolation.Periodic, smoothness = Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments, table = [0, 30 + 273.15; 8*3600, 25 + 273.15; 18*3600, 30 + 273.15; 24*3600, 30 + 273.15]) annotation (
    Placement(visible = true, transformation(origin={10,12},     extent = {{-152, 10}, {-132, 30}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.BooleanScalarReplicator booleanScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin={-22,6},      extent = {{-90, 70}, {-70, 90}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout = 5)  annotation (
    Placement(visible = true, transformation(origin={-94,-4},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator realScalarReplicator(nout = 5) annotation (
    Placement(visible = true, transformation(origin={-94,28},     extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Controls.OBC.CDL.Reals.Sources.Constant ERW_drybulb_temp(k=295)
    annotation (Placement(transformation(extent={{-142,-68},{-122,-48}})));
equation
  connect(enaSch.y[1], hys.u) annotation (
    Line(points={{-120,58},{-120,66},{-110,66},{-110,54}},
                                            color = {0, 0, 127}));
  connect(hys.y, booRep.u) annotation (
    Line(points={{-86,54},{-84,54},{-84,58},{-80,58}},
                                          color = {255, 0, 255}));
//connect(booRep.y, terCon.uDetOcc) annotation(
//Line(points = {{-68, 80}, {-4, 80}, {-4, 58}, {8, 58}}, color = {255, 0, 255}));
  connect(TSetRooHea.y[1], reaScaRep.u) annotation (
    Line(points={{-120,4},{-116,4},{-116,-4},{-106,-4}},          color = {0, 0, 127}));
  connect(realScalarReplicator.y, terCon.TZonCooSet) annotation (
    Line(points={{-82,28},{-68,28},{-68,2},{-46,2},{-46,-4},{-44,-4}},
                                                             color = {0, 0, 127}, thickness = 0.5));
  connect(yDamPosMax.y, DOAScon.uDamMaxOpe) annotation (Line(points={{150,42},{
          152,42},{152,76},{2,76},{2,0},{13,0}},              color={0,0,127}));
  connect(TZonMax.y, DOAScon.TAirHig) annotation (Line(points={{150,6},{150,10},
          {156,10},{156,68},{48,68},{48,70},{4,70},{4,-16},{13,-16}},     color=
         {0,0,127}));
  connect(DOAScon.TAirOut, chiBeaTesBed.OutdoorAirTemp) annotation (Line(points={{13,-28},
          {8,-28},{8,-58},{106,-58},{106,-30},{98,-30},{98,-29.3333}},
                                                color={0,0,127}));
  connect(DOAScon.uFanSupPro, chiBeaTesBed.yFanSta) annotation (Line(points={{13,-4},
          {12,-4},{12,-10},{8,-10},{8,38},{110,38},{110,-10.9333},{98,-10.9333}},
        color={255,0,255}));
  connect(chiBeaTesBed.exhFanSta, DOAScon.uFanExhPro) annotation (Line(points={{98,
          -8.13333},{124,-8.13333},{124,-10},{148,-10},{148,-68},{4,-68},{4,-44},
          {13,-44}},                   color={255,0,255}));
  connect(yDamPosMax.u[1:5], chiBeaTesBed.yDamPos) annotation (Line(points={{126,
          42.8},{116,42.8},{116,14},{112,14},{112,-2.66667},{98,-2.66667}},
        color={0,0,127}));
  connect(DOAScon.yFanSup, chiBeaTesBed.uFanSta) annotation (Line(points={{37,-2},
          {46,-2},{46,-4},{54,-4},{54,-8},{68,-8},{68,-9.2},{74,-9.2}},
                                                       color={255,0,255}));
  connect(reaScaRep.y, terCon.TZonHeaSet) annotation (Line(points={{-82,-4},{
          -72,-4},{-72,0},{-44,0}},              color={0,0,127}));
  connect(chiBeaTesBed.relHumDOASRet, DOAScon.phiAirRet) annotation (Line(
        points={{98,-24},{152,-24},{152,30},{14,30},{14,24},{6,24},{6,-12},{13,
          -12}},
        color={0,0,127}));
  connect(ERW_drybulb_temp.y, DOAScon.TAirSupEneWhe) annotation (Line(points={{-120,
          -58},{-114,-58},{-114,-32},{-106,-32},{-106,-30},{2,-30},{2,-40},{13,
          -40}},                                              color={0,0,127}));
  connect(occSch.occupied, booleanScalarReplicator.u) annotation (Line(points={{-121,80},
          {-114,80},{-114,86}},             color={255,0,255}));
  connect(chiBeaTesBed.yPumSta, sysCon.uPumSta[1]) annotation (Line(points={{98,
          -16.1333},{98,-16},{138,-16},{138,-78},{-62,-78},{-62,-42},{-38,-42}},
                 color={255,0,255}));
  connect(chiBeaTesBed.TDOASDis, DOAScon.TAirDisCoiCoo) annotation (Line(points={{98,
          -18.5333},{98,-18},{130,-18},{130,-60},{-4,-60},{-4,-20},{12,-20},{12,
          -32},{13,-32}},            color={0,0,127}));
  connect(booleanScalarReplicator.y, terCon.uOccDet) annotation (Line(points={{
          -90,86},{-74,86},{-74,84},{-52,84},{-52,8},{-44,8}}, color={255,0,255}));
  connect(booRep.y, terCon.uOccExp)
    annotation (Line(points={{-56,58},{-56,4},{-44,4}}, color={255,0,255}));
  connect(hys.y, DOAScon.Occ) annotation (Line(points={{-86,54},{-82,54},{-82,
          44},{-4,44},{-4,4},{13,4}}, color={255,0,255}));
  connect(TSetRooCoo.y[1], realScalarReplicator.u) annotation (Line(points={{
          -120,32},{-116,32},{-116,30},{-112,30},{-112,28},{-106,28}}, color={0,
          0,127}));
  connect(TSetRooCoo.y[1], DOAScon.TZonCooSet) annotation (Line(points={{-120,
          32},{-118,32},{-118,16},{13,16},{13,12}}, color={0,0,127}));
  connect(TSetRooHea.y[1], DOAScon.TZonHeaSet) annotation (Line(points={{-120,4},
          {-114,4},{-114,12},{-62,12},{-62,38},{-10,38},{-10,8},{13,8}}, color=
          {0,0,127}));
  connect(chiWatSupTem.y, chiBeaTesBed.TChiWatSup) annotation (Line(points={{-120,
          -24},{-68,-24},{-68,-64},{74,-64},{74,-34},{73.9,-34},{73.9,-28.0667}},
                      color={0,0,127}));
  connect(chiBeaTesBed.yChiWatVal, sysCon.uValPos) annotation (Line(points={{98,
          -0.133333},{118,-0.133333},{118,-92},{-48,-92},{-48,-54},{-38,-54}},
        color={0,0,127}));
  connect(chiBeaTesBed.dPChiWat, sysCon.dPChiWatLoo) annotation (Line(points={{98,
          -13.3333},{128,-13.3333},{128,-14},{156,-14},{156,-88},{-54,-88},{-54,
          -44},{-38,-44},{-38,-48}},     color={0,0,127}));
  connect(sysCon.yPumSpe, chiBeaTesBed.uPumSpe) annotation (Line(points={{-14,-48},
          {-4,-48},{-4,-52},{58,-52},{58,-25.3333},{74,-25.3333}},      color={
          0,0,127}));
  connect(sysCon.yChiWatPum[1], chiBeaTesBed.uPumSta) annotation (Line(points={
          {-14,-42},{-6,-42},{-6,-54},{66,-54},{66,-22.8},{74,-22.8}}, color={
          255,0,255}));
  connect(sysCon.yBypValPos, chiBeaTesBed.uBypValPos) annotation (Line(points={
          {-14,-54},{-8,-54},{-8,-84},{70,-84},{70,1.33333},{74,1.33333}},
        color={0,0,127}));
  connect(uConSig.y, terCon.uConSen) annotation (Line(points={{-82,-56},{-56,
          -56},{-56,-8},{-44,-8}}, color={255,0,255}));
  connect(terCon.yDam, chiBeaTesBed.uCAVDam) annotation (Line(points={{-20,-6},
          {-6,-6},{-6,-4},{-2,-4},{-2,26},{64,26},{64,-3.73333},{74,-3.73333}},
        color={0,0,127}));
  connect(loads.y, chiBeaTesBed.QFlo) annotation (Line(points={{-121,-88},{-78,
          -88},{-78,-98},{-30,-98},{-30,-96},{56,-96},{56,3.86667},{74,3.86667}},
        color={0,0,127}));
  connect(terCon.yChiVal, chiBeaTesBed.uChiWatVal) annotation (Line(points={{
          -20,-2},{-16,-2},{-16,34},{50,34},{50,-1.33333},{74,-1.33333}}, color
        ={0,0,127}));
  connect(chiBeaTesBed.yChiWatVal, terCon.uChiVal) annotation (Line(points={{98,
          -0.133333},{102,-0.133333},{102,52},{-48,52},{-48,-20},{-44,-20}},
        color={0,0,127}));
  connect(terCon.yReh, chiBeaTesBed.uCAVReh) annotation (Line(points={{-20,2},{
          -20,62},{60,62},{60,-6.53333},{73.8,-6.53333}}, color={0,0,127}));
  connect(chiBeaTesBed.TZon, terCon.TZon) annotation (Line(points={{98,8},{120,
          8},{120,82},{-48,82},{-48,-12},{-44,-12}}, color={0,0,127}));
  connect(chiBeaTesBed.bldgSP, DOAScon.dPAirStaBui) annotation (Line(points={{
          98,-32},{100,-32},{100,-40},{42,-40},{42,-56},{10,-56},{10,-48},{13,
          -48}}, color={0,0,127}));
  connect(chiBeaTesBed.erwsuphum, DOAScon.phiAirEneRecWhe) annotation (Line(
        points={{98,-21.3333},{114,-21.3333},{114,-22},{128,-22},{128,-82},{-2,
          -82},{-2,-36},{13,-36}}, color={0,0,127}));
  connect(chiBeaTesBed.TZon, TZonMax.u[1:5])
    annotation (Line(points={{98,8},{126,8},{126,6.8}}, color={0,0,127}));
  connect(chiBeaTesBed.VDisAir_flow, terCon.VDis_flow) annotation (Line(points=
          {{98,5.33333},{100,5.33333},{100,22},{-62,22},{-62,-16},{-44,-16}},
        color={0,0,127}));
  connect(DOAScon.yEneRecWheSpe, chiBeaTesBed.uEneRecWheSpe) annotation (Line(
        points={{37,-26},{52,-26},{52,-10},{64,-10},{64,-12},{74,-12}}, color={
          0,0,127}));
  connect(DOAScon.yCoiCoo, chiBeaTesBed.uCooCoi) annotation (Line(points={{37,
          -10},{44,-10},{44,-20},{74,-20}}, color={0,0,127}));
  connect(DOAScon.yFanSupSpe, chiBeaTesBed.uFanSpe) annotation (Line(points={{37,-6},
          {50,-6},{50,-12},{62,-12},{62,-16},{72,-16},{72,-14.6667},{74,
          -14.6667}}, color={0,0,127}));
  connect(chiBeaTesBed.uHeaCoi, DOAScon.yCoiHea) annotation (Line(points={{74,
          -17.0667},{58,-17.0667},{58,-16},{37,-16},{37,-14}}, color={0,0,127}));
  connect(DOAScon.TAirSup, DOAScon.TAirDisCoiCoo) annotation (Line(points={{13,
          -20},{12,-20},{12,-32},{13,-32}}, color={0,0,127}));
  connect(chiBeaTesBed.rAT, DOAScon.TAirRet) annotation (Line(points={{98,-26.8},
          {106,-26.8},{106,-28},{108,-28},{108,-74},{-8,-74},{-8,-24},{13,-24}},
        color={0,0,127}));
  connect(chiBeaTesBed.dPDOASAir, DOAScon.dPAirDucSta) annotation (Line(points=
          {{98,-5.33333},{104,-5.33333},{104,26},{-6,26},{-6,-8},{13,-8}},
        color={0,0,127}));
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
<p>This model simulates Buildings.Examples.ChilledBeamSystem.BaseClasses.TestBed, a chilled beam system with five thermal zones. Key components include:</p>
<ol>
<li><b>DOAS Controller <code>(DOAScon)</code> :</b> Operates the supply fan, energy recovery wheel, cooling coil, and heating coil to ensure optimal outdoor air condtions are delivered</li>
<li><b>System Controller <code>(sysCon)</code>:</b> Regulates the single chilled water loop that serves the chilled beam system, supply chilled water from an ideal source</li>
<li><b>Terminal Controller <code>(terCon)</code>: </b>Controls each of the zone&apos;s constant air volume (CAV) terminal box and the chilled beam manifold control valve.</li>
</ol>
<p>The HVAC system consists a constant air volume(CAV) system with an energy recovery wheel, heating, and cooling coils in the air handling unit. </p>
<p>The chilled beam system is serviced by a single chilled water loop with an ideal source operated by the system controller. Each zone has a chilled beam manifold, an air circulation fan for cooling, and CAV terminal box with a hot water reheat coil and an exponential damper. The terminal controller operates the zone CAV terminal box as well as the zone chilled beam manifold control valve.</p>
</html>"));
end ClosedLoopValidation;
