within Buildings.Controls.OBC.FDE.DOAS.Subsequences;
block EnergyWheel "This block commands the energy recovery wheel and associated bypass dampers."

  parameter Real dTThrEneRec(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 7
  "Absolute temperature difference threshold between outdoor air and return air temperature above which energy recovery is enabled";

   parameter Real dThys(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 0.5
  "Delay time period after temperature difference threshold is crossed for enabling energy recovery mode";

  parameter Real timDelEneRec(
  final unit = "s",
  final quantity = "Time") = 300
  "Minimum delay after OAT/RAT delta falls below set point.";

  parameter CDL.Types.SimpleController controllerTypeHeaREc=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI controller type for energy recovery wheel heating recovery loop";

  parameter Real kHeaRec(
  final unit = "1") = 0.5
  "PID heating recovery loop gain value.";

  parameter Real TiHeaRec(
  final unit = "s") = 60
  "PID  heating recovery loop time constant of integrator.";

  parameter Real TdHeaRec(
  final unit = "s") = 0.1
  "PID heating recovery loop time constant of derivative block";

  parameter Real kCooRec(
  final unit = "1") = 0.5
  "PID cooling recovery loop gain value.";

  parameter Real TiCooRec(
  final unit = "s") = 60 "PID cooling recovery loop time constant of integrator.";

  parameter CDL.Types.SimpleController controllerTypeCooRec=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PID controller  type for energy recovery wheel cooling recovery loop";

  parameter Real TdCooRec(
  final unit = "s") = 0.1
  "PID cooling recovery loop time constant of derivative block";

// ---inputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when the supply fan is proven on." annotation (Placement(
        transformation(extent={{-140,60},{-100,100}}), iconTransformation(
          extent={{-140,60},{-100,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirOut
    "Outside air temperature sensor." annotation (Placement(transformation(
          extent={{-140,-40},{-100,0}}), iconTransformation(extent={{-140,-40},{
            -100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirRet
    "Return air temperature sensor." annotation (Placement(transformation(
          extent={{-140,0},{-100,40}}), iconTransformation(extent={{-140,0},{-100,
            40}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uEcoMod "True when economizer mode is active."
  annotation(Placement(transformation(extent={{-140,40},{-100,80}}),
    iconTransformation(extent={{-140,40},{-100,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupEneWhe
    "Energy recovery wheel supply air temperature." annotation (Placement(
        transformation(extent={{-142,-74},{-102,-34}}), iconTransformation(
          extent={{-140,-60},{-100,-20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupSetEneWhe
    "Primary supply air temperature set point." annotation (Placement(
        transformation(extent={{-142,-106},{-102,-66}}), iconTransformation(
          extent={{-140,-100},{-100,-60}})));

// ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yEneRecWheEna
    "Command to enable the energy recovery wheel." annotation (Placement(
        transformation(extent={{102,0},{142,40}}), iconTransformation(extent={{100,-20},
            {140,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yBypDam
    "Bypass damper command; true when commanded full open." annotation (
      Placement(transformation(extent={{102,46},{142,86}}), iconTransformation(
          extent={{100,40},{140,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yEneRecWheSpe(
    final unit="1",
    final min=0,
    final max=1) "Energy recovery wheel speed command." annotation (Placement(
        transformation(extent={{102,-96},{142,-56}}), iconTransformation(extent={{100,-80},
            {140,-40}})));

protected
  Buildings.Controls.OBC.CDL.Reals.Subtract                   difference
  "Subtract outside air temperature from return air temperature."
  annotation(Placement(visible = true, transformation(origin = {24, 2}, extent = {{-90, -10}, {-70, 10}}, rotation = 0)));

  Buildings.Controls.OBC.CDL.Reals.Abs abs
  "Absolute value of OAT-RAT difference."
  annotation(Placement(visible = true, transformation(origin = {26, 2}, extent = {{-62, -10}, {-42, 10}}, rotation = 0)));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay EneWhe(delayTime=timDelEneRec,
  delayOnInit = true)
  "Recovery set point delay before disabling energy wheel."
  annotation(Placement(visible = true, transformation(origin = {30, 28}, extent = {{2, -38}, {22, -18}}, rotation = 0)));

  Buildings.Controls.OBC.CDL.Logical.MultiAnd mulAndEneRecRegOpe(nin=3) "Logical AND; true when fan is proven, economizer mode is off, and ERW 
   temperature start conditions are met."
    annotation (Placement(transformation(extent={{66,8},{86,28}})));

  Buildings.Controls.OBC.CDL.Logical.Not not1
  "Logical NOT; true when economizer mode is off."
  annotation(Placement(transformation(extent = {{-26, 40}, {-6, 60}})));

  Buildings.Controls.OBC.CDL.Reals.Max max
  "Outputs maximum value of two ERW temperature PI loops."
  annotation (
    Placement(transformation(extent={{-24,-72},{-4,-52}})));

  Buildings.Controls.OBC.CDL.Reals.Switch swiTEneRec
    "Logical switch outputs ERW temperature PI maximum output  when erwStart command is true."
    annotation (Placement(transformation(extent={{66,-86},{86,-66}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conZer(final k=0)
    "Real constant 0."
    annotation (Placement(transformation(extent={{30,-94},{50,-74}})));

  Buildings.Controls.OBC.CDL.Logical.And and1
  "Logical AND; true when fan is proven and unit is in economizer mode."
  annotation(Placement(transformation(extent = {{-64, 56}, {-44, 76}})));

  Buildings.Controls.OBC.CDL.Logical.Or or2
  "Logical OR."
  annotation(Placement(transformation(extent = {{62, 56}, {82, 76}})));

  Buildings.Controls.OBC.CDL.Logical.Not not2
  "Logical NOT; true when ERW start command is off."
  annotation(Placement(transformation(extent = {{34, 40}, {54, 60}})));

  Buildings.Controls.OBC.CDL.Reals.PID conPIDHeaRec(
    controllerType=controllerTypeHeaRec,
    Ti=TiHeaRec,
    k=kHeaRec,
    Td=TdHeaRec) "PID loop for heating recovery" annotation (Placement(visible=
          true, transformation(
        origin={-70,-38},
        extent={{-10,-10},{10,10}},
        rotation=0)));

  Buildings.Controls.OBC.CDL.Reals.PID conPIDCooRec(
    controllerType=controllerTypeCooRec,
    Ti=TiCooRec,
    k=kCooRec,
    Td=TdCooRec) "PID loop for cooling recovery" annotation (Placement(visible=
          true, transformation(
        origin={-66,-78},
        extent={{-10,-10},{10,10}},
        rotation=0)));

  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uHigh=dTThrEneRec, uLow=
        dTThrEneRec - dThys)                                                             annotation (
    Placement(visible = true, transformation(origin={8,2},     extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
  connect(not1.u,uEcoMod)  annotation (
    Line(points={{-28,50},{-74,50},{-74,60},{-120,60}},
                                           color = {255, 0, 255}));

  connect(mulAndEneRecRegOpe.y, yEneRecWheEna) annotation (Line(points={{88,
          18},{104,18},{104,20},{122,20}}, color={255,0,255}));

  connect(max.y, swiTEneRec.u1) annotation (Line(points={{-2,-62},{14,-62},{14,-68},
          {64,-68}}, color={0,0,127}));

  connect(mulAndEneRecRegOpe.y, swiTEneRec.u2) annotation (Line(points={{88,18},
          {88,-20},{92,-20},{92,-58},{58,-58},{58,-76},{64,-76}}, color={255,0,255}));

  connect(conZer.y, swiTEneRec.u3)
    annotation (Line(points={{52,-84},{64,-84}}, color={0,0,127}));

  connect(swiTEneRec.y, yEneRecWheSpe)
    annotation (Line(points={{88,-76},{122,-76}}, color={0,0,127}));
  connect(uEcoMod, and1.u2) annotation (
    Line(points={{-120,60},{-94,60},{-94,58},{-66,58}},          color = {255, 0, 255}));

  connect(uFanSupPro, and1.u1) annotation (Line(points={{-120,80},{-94,80},{-94,
          66},{-66,66}}, color={255,0,255}));

  connect(and1.y, or2.u1) annotation (
    Line(points = {{-42, 66}, {60, 66}}, color = {255, 0, 255}));

  connect(not2.y, or2.u2) annotation (
    Line(points = {{56, 50}, {58, 50}, {58, 58}, {60, 58}}, color = {255, 0, 255}));

  connect(mulAndEneRecRegOpe.y, not2.u) annotation (Line(points={{88,18},{88,20},
          {98,20},{98,36},{28,36},{28,50},{32,50}}, color={255,0,255}));

  connect(or2.y, yBypDam)
    annotation (Line(points={{84,66},{122,66}}, color={255,0,255}));

  connect(TAirSupSetEneWhe, conPIDCooRec.u_s) annotation (Line(points={{-122,-86},
          {-105,-86},{-105,-78},{-78,-78}}, color={0,0,127}));

  connect(TAirSupSetEneWhe, conPIDHeaRec.u_s) annotation (Line(points={{-122,-86},
          {-122,-84},{-82,-84},{-82,-38}}, color={0,0,127}));

  connect(TAirSupEneWhe, conPIDHeaRec.u_m) annotation (Line(points={{-122,-54},
          {-102,-54},{-102,-50},{-70,-50}}, color={0,0,127}));

  connect(TAirSupEneWhe, conPIDCooRec.u_m) annotation (Line(points={{-122,-54},
          {-100,-54},{-100,-90},{-66,-90}}, color={0,0,127}));

  connect(conPIDHeaRec.y, max.u1)
    annotation (Line(points={{-58,-38},{-58,-56},{-26,-56}}, color={0,0,127}));

  connect(conPIDCooRec.y, max.u2)
    annotation (Line(points={{-54,-78},{-54,-68},{-26,-68}}, color={0,0,127}));

  connect(TAirRet, difference.u1)
    annotation (Line(points={{-120,20},{-120,8},{-68,8}}, color={0,0,127}));

  connect(TAirOut, difference.u2)
    annotation (Line(points={{-120,-20},{-120,-4},{-68,-4}}, color={0,0,127}));

  connect(difference.y, abs.u) annotation (
    Line(points = {{-44, 2}, {-38, 2}}, color = {0, 0, 127}));

  connect(hys.y,EneWhe. u) annotation (
    Line(points={{20,2},{25,2},{25,0},{30,0}},
                                         color = {255, 0, 255}));

  connect(abs.y, hys.u)
    annotation (Line(points={{-14, 2}, {-4, 2}}, color={0,0,127}));
  connect(uFanSupPro, mulAndEneRecRegOpe.u[1]) annotation (Line(points={{-120,80},
          {-70,80},{-70,88},{20,88},{20,15.6667},{64,15.6667}}, color={255,0,255}));
  connect(not1.y, mulAndEneRecRegOpe.u[2]) annotation (Line(points={{-4,50},{10,
          50},{10,18},{64,18}}, color={255,0,255}));
  connect(EneWhe.y, mulAndEneRecRegOpe.u[3]) annotation (Line(points={{54,0},{
          56,0},{56,20.3333},{64,20.3333}},
                                         color={255,0,255}));
  annotation (
    defaultComponentName = "ERWcon",
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius=0),    Line(points = {{48, 0}, {-2, 0}}), Text(textColor = {28, 108, 200}, extent={{-98,180},
              {82,76}},                                                                                                                                                    textString = "%name", textStyle = {TextStyle.Bold}), Text(textColor = {28, 108, 200}, extent={{-98,86},
              {-54,72}},                                                                                                                                                                                                        textString = "supFanProof"), Text(textColor = {28, 108, 200}, extent={{-98,64},
              {-62,54}},                                                                                                                                                                                                        textString = "ecoMode"), Text(textColor = {28, 108, 200}, extent = {{-108, 24}, {-72, 14}}, textString = "raT"), Text(textColor = {28, 108, 200}, extent = {{-108, -12}, {-72, -22}}, textString = "oaT"), Text(textColor = {28, 108, 200}, extent={{-106,
              -36},{-70,-46}},                                                                                                                                                                                                        textString = "erwT"), Text(textColor = {28, 108, 200}, extent = {{-96, -70}, {-56, -82}}, textString = "supPrimSP"), Text(textColor = {28, 108, 200}, extent = {{62, -54}, {98, -64}}, textString = "erwSpeed"), Text(textColor = {28, 108, 200}, extent = {{60, 4}, {96, -6}}, textString = "erwStart"), Text(textColor = {28, 108, 200}, extent = {{60, 66}, {96, 56}}, textString = "bypDam"), Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-14, 64}, {28, -68}}), Rectangle(lineColor = {170, 255, 255}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-8, 64}, {8, -68}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-28, 64}, {14, -68}}), Rectangle(lineColor = {0, 140, 72}, fillColor = {0, 140, 72},
            fillPattern=
FillPattern.Solid, extent = {{32, -34}, {72, -36}}), Polygon(lineColor = {0, 140, 72}, fillColor = {0, 140, 72},
            fillPattern = FillPattern.Solid, points = {{72, -34}, {58, -28}, {58, -34}, {72, -34}}), Rectangle(lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
            fillPattern=
FillPattern.Solid, extent = {{-70, -34}, {-30, -36}}), Polygon(lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
            fillPattern=
FillPattern.Solid, points = {{-30, -34}, {-44, -28}, {-44, -34}, {-30, -34}}), Line(points = {{-8, 0}, {-58, 0}}), Rectangle(lineColor = {0, 140, 72}, fillColor = {0, 140, 72},
            fillPattern=
FillPattern.Solid, extent = {{-70, 30}, {-30, 28}}), Polygon(lineColor = {0, 140, 72}, fillColor = {0, 140, 72},
            fillPattern=
FillPattern.Solid, points = {{-56, 30}, {-56, 36}, {-70, 30}, {-56, 30}}), Polygon(lineColor = {238, 46, 47}, fillColor = {238, 46, 47},
            fillPattern=
FillPattern.Solid, points = {{44, 30}, {44, 36}, {30, 30}, {44, 30}}), Rectangle(lineColor = {238, 46, 47}, fillColor = {238, 46, 47},
            fillPattern=
FillPattern.Solid, extent = {{30, 30}, {70, 28}})}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Documentation(revisions = "<html>
<ul>
<li>
September 15, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info="<html>
<p><b>Energy Recovery Wheel Start/Stop.</b></p>
<p>This block commands the ERW to start (<span style=\"font-family: Courier New;\">yEneRecWheEna</span>) when the DOAS is energized (<span style=\"font-family: Courier New;\">uFanSupPro</span>) and the absolute difference between return air temperature (<span style=\"font-family: Courier New;\">TAirRet</span>) and outside air temperature (<span style=\"font-family: Courier New;\">TAirOut</span>) is greater than the recovery set point (<span style=\"font-family: Courier New;\">dTThrEneRec</span>). When the DOAS is not energized, economizer mode is enabled (<span style=\"font-family: Courier New;\">uEcoMod)</span>, or the TAirRet/TAirOut difference falls below the recovery set point for longer than the recovery set point delay (<span style=\"font-family: Courier New;\">timDelEneRec</span>) the ERW will be commanded to stop.</p>
<p><b>ERW Speed Control</b></p>
<p>The ERW speed (<span style=\"font-family: Courier New;\">yEneRecWheSpe</span>) is modulated to maintain the energy recovery supply temperature (<span style=\"font-family: Courier New;\">TAirSupEneWhe</span>) at the primary supply air temperature set point (<span style=\"font-family: Courier New;\">TAirSupSetEneWhe</span>).</p>
<p><b>Bypass Damper Control</b></p>
<p>When the DOAS is energized and in economizer mode or the ERW is stopped, the bypass dampers shall be commanded fully open to bypass (<span style=\"font-family: Courier New;\">yBypDam</span>). When the DOAS is de-energized or the DOAS is energized and the ERW is started, the bypass dampers shall be commanded closed to bypass. </p>
</html>"));
end EnergyWheel;
