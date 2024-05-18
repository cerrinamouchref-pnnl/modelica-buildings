within Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChilledBeamSystem.Testbed;
block ClosedLoopValidation
  parameter Real schTab[5,2]=[0,0; 8,1; 18,1; 21,0; 24,0]
    "Table defining schedule for enabling plant";

  TestBed_v2 testBed(
    TChiWatRet_nominal=273.15 + 25,
    mChiWatTot_flow_nominal=2.114,
    mAirTot_flow_nominal=1*0.676*1.225,
    mHotWatCoi_nominal=0.078,
    mChiWatCoi_nominal=0.645,
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
    mAChiBeaCor_flow_nominal=0.26*1.225)
    annotation (Placement(transformation(extent={{60,-20},{128,20}})));
  Terminal.TerminalController terCon[5](
    controllerTypeCoo=Buildings.Controls.OBC.CDL.Types.SimpleController.PID,
    TiCoo=fill(50, 5),
    TdCoo={0.1,100,0.1,0.1,0.1},
    VDes_occ={0.143,0.065,0.143,0.065,0.26},
    VDes_unoccSch={0.028,0.012,0.028,0.012,0.052},
    VDes_unoccUnsch={0.056,0.024,0.056,0.024,0.104})
                         "Terminal controllers"
    annotation (Placement(transformation(extent={{10,40},{30,60}})));
  System.SystemController sysCon(nPum=1, nVal=5,
    minPumSpe=0.7,
    TiPumSpe=50,
    kBypVal=10e-3,
    TiBypVal=900)
    annotation (Placement(transformation(extent={{10,-70},{30,-50}})));
  FDE.DOAS.DOAScontroller_modified DOAScon(
    minDDSPset=400,
    maxDDSPset=500,
    cvDDSPset=450)
    annotation (Placement(transformation(extent={{-4,-18},{16,18}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiMax TZonMax(nin=5)
    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiMax yDamPosMax(nin=5)
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.TimeTable                        enaSch(
    final table=schTab,
    final smoothness=Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments,
    final timeScale=3600) "Table defining when occupancy is expected"
    annotation (Placement(transformation(extent={{-150,70},{-130,90}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hys(uLow=0.45, uHigh=0.5)
    annotation (Placement(transformation(extent={{-120,70},{-100,90}})));
  Buildings.Controls.OBC.CDL.Routing.BooleanReplicator booRep(nout=5)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant uConSig[5](k=fill(false,
        5)) "Constant Boolean source"
    annotation (Placement(transformation(extent={{-90,30},{-70,50}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant chiWatSupTem(k=273.15
         + 7.22) "Chilled water supply temperature"
    annotation (Placement(transformation(extent={{-90,-30},{-70,-10}})));
  Modelica.Blocks.Sources.CombiTimeTable loads(
    tableOnFile=true,
    tableName="tab1",
    fileName="./VM_script/inputTable_constantSetpoint.txt",
    columns={2,3,4,5,6},
    timeScale=60) "Table defining thermal loads for zone"
    annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));

equation
  connect(terCon.yReh, testBed.uCAVReh)
    annotation (Line(points={{32,58},{50,58},{50,2.85714},{58.5217,2.85714}},
                                                           color={0,0,127}));
  connect(terCon.yChiVal, testBed.uChiWatVal) annotation (Line(points={{32,54},
          {48,54},{48,8.57143},{58.5217,8.57143}},
                                   color={0,0,127}));
  connect(terCon.yDam, testBed.uCAVDam)
    annotation (Line(points={{32,50},{52,50},{52,5.71429},{58.5217,5.71429}},
                                                             color={0,0,127}));
  connect(sysCon.yChiWatPum[1], testBed.uPumSta) annotation (Line(points={{32,-56},
          {54,-56},{54,-12.1429},{58.5217,-12.1429}},
                                    color={255,0,255}));
  connect(sysCon.yPumSpe, testBed.uPumSpe) annotation (Line(points={{32,-60},{
          52,-60},{52,-14.2857},{58.5217,-14.2857}},
                                 color={0,0,127}));
  connect(testBed.yPumSta, sysCon.uPumSta[1]) annotation (Line(points={{129.478,
          -12.8571},{142,-12.8571},{142,-80},{0,-80},{0,-56},{8,-56}},
                                                            color={255,0,255}));
  connect(testBed.dPChiWat, sysCon.dPChiWatLoo) annotation (Line(points={{129.478,
          -10.7143},{146,-10.7143},{146,-86},{-6,-86},{-6,-60},{8,-60}},
                                                           color={0,0,127}));
  connect(testBed.yChiWatVal, sysCon.uValPos) annotation (Line(points={{129.478,
          1.42857},{140,1.42857},{140,-74},{4,-74},{4,-64},{8,-64}},
                                                      color={0,0,127}));
  connect(testBed.yChiWatVal, terCon.uChiVal) annotation (Line(points={{129.478,
          1.42857},{140,1.42857},{140,80},{0,80},{0,42},{8,42}},
                                                  color={0,0,127}));
  connect(sysCon.yBypValPos, testBed.uBypValPos) annotation (Line(points={{32,-64},
          {44,-64},{44,11.4286},{58.5217,11.4286}},
                                    color={0,0,127}));
  connect(DOAScon.supFanStart, testBed.uFanSta) annotation (Line(points={{18.2,
          13.1294},{30,13.1294},{30,-1.42857},{58.5217,-1.42857}},
                                                 color={255,0,255}));
  connect(DOAScon.supFanSpeed, testBed.uFanSpe) annotation (Line(points={{18.2,
          9.74118},{28,9.74118},{28,-3.57143},{58.5217,-3.57143}},
                                                 color={0,0,127}));
  connect(DOAScon.yRHC, testBed.uHeaCoi) annotation (Line(points={{18.2,3.17647},
          {26,3.17647},{26,-7.85714},{58.5217,-7.85714}},
                                          color={0,0,127}));
  connect(DOAScon.yCC, testBed.uCooCoi) annotation (Line(points={{18.2,6.35294},
          {24,6.35294},{24,-10},{58.5217,-10}},
                                          color={0,0,127}));
  connect(testBed.dPDOASAir, DOAScon.DDSP) annotation (Line(points={{129.478,
          -5.71429},{138,-5.71429},{138,-40},{-20,-40},{-20,9.95294},{-6,
          9.95294}},                                           color={0,0,127}));
  connect(testBed.TDOASDis, DOAScon.saT) annotation (Line(points={{129.478,-15},
          {132,-15},{132,-30},{-10,-30},{-10,1.90588},{-6,1.90588}},
                                                           color={0,0,127}));
  connect(TZonMax.y, DOAScon.highSpaceT) annotation (Line(points={{-18,60},{-10,
          60},{-10,4.65882},{-6,4.65882}}, color={0,0,127}));
  connect(yDamPosMax.y, DOAScon.mostOpenDam) annotation (Line(points={{-18,30},
          {-14,30},{-14,15.0353},{-6,15.0353}}, color={0,0,127}));
  connect(testBed.yFanSta, DOAScon.supFanStatus) annotation (Line(points={{129.478,
          -7.85714},{134,-7.85714},{134,38},{-12,38},{-12,12.4941},{-6,12.4941}},
                                                                 color={255,0,255}));
  connect(testBed.relHumDOASRet, DOAScon.retHum) annotation (Line(points={{129.626,
          -16.8571},{134,-16.8571},{134,-36},{-16,-36},{-16,7.41176},{-6,
          7.41176}},                                                color={0,0,127}));
  connect(enaSch.y[1], hys.u)
    annotation (Line(points={{-128,80},{-122,80}}, color={0,0,127}));
  connect(hys.y, booRep.u)
    annotation (Line(points={{-98,80},{-92,80}}, color={255,0,255}));
  connect(booRep.y, terCon.uDetOcc) annotation (Line(points={{-68,80},{-4,80},{
          -4,58},{8,58}}, color={255,0,255}));
  connect(uConSig.y, terCon.uConSen) annotation (Line(points={{-68,40},{-60,40},
          {-60,48},{-4,48},{-4,54},{8,54}}, color={255,0,255}));
  connect(hys.y, DOAScon.occ) annotation (Line(points={{-98,80},{-96,80},{-96,
          17.5765},{-6,17.5765}}, color={255,0,255}));
  connect(chiWatSupTem.y, testBed.TChiWatSup) annotation (Line(points={{-68,-20},
          {40,-20},{40,-17.1429},{58.5217,-17.1429}}, color={0,0,127}));
  connect(testBed.TZon, terCon.TZon) annotation (Line(points={{129.478,14.2857},
          {132,14.2857},{132,72},{4,72},{4,50},{8,50}}, color={0,0,127}));
  connect(testBed.VDisAir_flow, terCon.VDis_flow) annotation (Line(points={{129.183,
          10.5714},{136,10.5714},{136,76},{2,76},{2,46},{8,46}},         color=
          {0,0,127}));
  connect(testBed.TZon, TZonMax.u[1:5]) annotation (Line(points={{129.478,
          14.2857},{132,14.2857},{132,72},{-44,72},{-44,58.4},{-42,58.4}},
        color={0,0,127}));
  connect(testBed.yDamPos, yDamPosMax.u[1:5]) annotation (Line(points={{129.478,
          -3.57143},{148,-3.57143},{148,86},{-50,86},{-50,28.4},{-42,28.4}},
        color={0,0,127}));
  connect(loads.y, testBed.QFlo) annotation (Line(points={{-39,-50},{-30,-50},{
          -30,14},{-16,14},{-16,28},{46,28},{46,16.4286},{58.5217,16.4286}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},
            {160,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,100}})),
    experiment(
      StartTime=16848000,
      StopTime=16934400,
      Interval=600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"));
end ClosedLoopValidation;
