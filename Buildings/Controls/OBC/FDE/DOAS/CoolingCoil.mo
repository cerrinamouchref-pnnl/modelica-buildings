within Buildings.Controls.OBC.FDE.DOAS;
block CoolingCoil "This block commands the cooling coil."

  parameter Real erwDPadj(
  final unit = "K",
  final quantity = "TemperatureDifference") = 5
  "Value subtracted from ERW supply air dewpoint.";

  parameter CDL.Types.SimpleController controllerTypeDeh=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for cooling air in dehumidification mode";

  parameter Real kDeh(
  final unit="1") = 1
    "Gain of conPIDDeh controller";

  parameter Real TiDeh(
  final unit="s") = 60
    "Time constant of integrator block for conPIDDeh controller";

  parameter Real TdDeh(
  final unit="s") = 0.1
    "Time constant of derivative block for conPIDDeh controller";

  parameter CDL.Types.SimpleController controllerTypeRegOpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for regular cooling coil operation mode";

  parameter Real kRegOpe(
  final unit="1") = 1
    "Gain of conPIDRegOpe controller";

  parameter Real TiRegOpe(
  final unit="s")=60
    "Time constant of integrator block for conPIDRegOpe controller";

  parameter Real TdRegOpe(
  final unit="s")=0.1
    "Time constant of derivative block for conPIDRegOpe controller";



  // ---inputs---

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "Supply fan proven on signal" annotation (Placement(transformation(extent={{
            -142,-116},{-102,-76}}), iconTransformation(extent={{-142,64},{-102,
            104}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uDeh
    "Dehumidification mode enable signal" annotation (Placement(transformation(
          extent={{-142,-24},{-102,16}}), iconTransformation(extent={{-142,-26},
            {-102,14}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSup(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Measured supply air temperature" annotation (Placement(transformation(
          extent={{-142,-56},{-102,-16}}), iconTransformation(extent={{-142,36},
            {-102,76}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupSetCoo(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Supply air cooling setpoint temperature" annotation (Placement(
        transformation(extent={{-142,-86},{-102,-46}}), iconTransformation(
          extent={{-142,8},{-102,48}})));


  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirEneRecWhe(
    final min=0,
    final max=1,
    unit="1")
    "Measured relative humidity of air conditioned by energy recovery wheel"
    annotation (Placement(transformation(extent={{-142,34},{-102,74}}),
        iconTransformation(extent={{-142,-78},{-102,-38}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirEneRecWhe(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Measured dry bulb temperature of air conditioned by energy recovery wheel"
    annotation (Placement(transformation(extent={{-142,4},{-102,44}}),
        iconTransformation(extent={{-142,-104},{-102,-64}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirDis(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Measured discharge air temperature"             annotation (Placement(
        transformation(extent={{-142,64},{-102,104}}), iconTransformation(
          extent={{-142,-52},{-102,-12}})));



  // ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoiCoo
    "Cooling coil control signal" annotation (Placement(transformation(extent={{
            102,56},{142,96}}), iconTransformation(extent={{102,-20},{142,20}})));

  Buildings.Controls.OBC.CDL.Psychrometrics.DewPoint_TDryBulPhi TAirDewEneRecWhe
    "Calculates dewpoint temperature for air conditioned by energy recovery wheel"
    annotation (Placement(visible=true, transformation(
        origin={-66,48},
        extent={{-10,-10},{10,10}},
        rotation=0)));
//protected
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conZer(final k=0)
    "Real constant zero"
    annotation (Placement(transformation(extent={{-30,-80},{-10,-60}})));

  Buildings.Controls.OBC.CDL.Reals.Switch swiFanSupPro
    "Output non-zero cooling coil signal only when supply fan is proven on"
    annotation (Placement(transformation(extent={{22,-50},{42,-30}})));

  Buildings.Controls.OBC.CDL.Reals.Switch swiTSetCoo
    "Switch cooling coil signal between regular operation and dehumidification mode"
    annotation (Placement(transformation(extent={{58,66},{78,86}})));

  Buildings.Controls.OBC.CDL.Logical.And andDehEna
    "Enable dehumidification mode only when supply fan is proven on"
    annotation (Placement(transformation(extent={{-20,-14},{0,8}})));

  CDL.Reals.PIDWithReset               conPIDRegOpe(
    controllerType=controllerTypeRegOpe,
    k=kRegOpe,
    Ti=TiRegOpe,
    Td=TdRegOpe,
    reverseActing=false)
    "PID controller for regular cooling coil operation mode" annotation (
      Placement(visible=true, transformation(
        origin={-62,-42},
        extent={{-10,-10},{10,10}},
        rotation=0)));

  CDL.Reals.PIDWithReset               conPIDDeh(
    controllerType=controllerTypeDeh,
    k=kDeh,
    Ti=TiDeh,
    Td=TdDeh,
    reverseActing=false)
    "PID controller for cooling air in dehumidification mode" annotation (
      Placement(visible=true, transformation(
        origin={10,80},
        extent={{-10,-10},{10,10}},
        rotation=0)));


  Buildings.Controls.OBC.CDL.Reals.AddParameter TSetCooDeh(p=-erwDPadj)
    "Calculate cooling setpoint temperature for air in dehumidification mode"
    annotation (Placement(visible=true, transformation(
        origin={-30,36},
        extent={{-10,-10},{10,10}},
        rotation=0)));

equation
  connect(conZer.y, swiFanSupPro.u3) annotation (Line(points={{-8,-70},{18,-70},
          {18,-48},{20,-48}}, color={0,0,127}));

  connect(swiFanSupPro.u2, uFanSupPro) annotation (Line(points={{20,-40},{6,-40},
          {6,-96},{-122,-96}}, color={255,0,255}));

  connect(andDehEna.u1, uDeh) annotation (Line(points={{-22,-3},{-78,-3},{-78,-4},
          {-122,-4}}, color={255,0,255}));

  connect(uFanSupPro, andDehEna.u2) annotation (Line(points={{-122,-96},{-42,-96},
          {-42,-11.8},{-22,-11.8}}, color={255,0,255}));

  connect(andDehEna.y, swiTSetCoo.u2) annotation (Line(points={{2,-3},{36,-3},{36,
          76},{56,76}}, color={255,0,255}));

  connect(swiTSetCoo.y, yCoiCoo)
    annotation (Line(points={{80,76},{122,76}}, color={0,0,127}));

  connect(swiFanSupPro.y, swiTSetCoo.u3) annotation (Line(points={{44,-40},{46,-40},
          {46,68},{56,68}}, color={0,0,127}));

  connect(TAirSup, conPIDRegOpe.u_m) annotation (Line(points={{-122,-36},{-90,-36},
          {-90,-54},{-62,-54}}, color={0,0,127}));

  connect(TAirSupSetCoo, conPIDRegOpe.u_s) annotation (Line(points={{-122,-66},{
          -80,-66},{-80,-42},{-74,-42}}, color={0,0,127}));

  connect(conPIDRegOpe.y, swiFanSupPro.u1) annotation (Line(points={{-50,-42},{-17,
          -42},{-17,-32},{20,-32}}, color={0,0,127}));

  connect(TAirDis, conPIDDeh.u_m) annotation (Line(points={{-122,84},{-22,84},{-22,
          68},{10,68}}, color={0,0,127}));

  connect(conPIDDeh.y, swiTSetCoo.u1)
    annotation (Line(points={{22,80},{56,80},{56,84}}, color={0,0,127}));

  connect(TAirEneRecWhe, TAirDewEneRecWhe.TDryBul) annotation (Line(points={{-122,
          24},{-88,24},{-88,54},{-78,54}}, color={0,0,127}));

  connect(phiAirEneRecWhe, TAirDewEneRecWhe.phi) annotation (Line(points={{-122,
          54},{-94,54},{-94,42},{-78,42}}, color={0,0,127}));


  connect(TAirDewEneRecWhe.TDewPoi, TSetCooDeh.u)
    annotation (Line(points={{-54,48},{-54,36},{-42,36}}, color={0,0,127}));

  connect(TSetCooDeh.y, conPIDDeh.u_s) annotation (Line(points={{-18,36},{-10,36},
          {-10,80},{-2,80}}, color={0,0,127}));
  connect(uFanSupPro, conPIDRegOpe.trigger) annotation (Line(points={{-122,-96},
          {-68,-96},{-68,-54}}, color={255,0,255}));
  connect(uFanSupPro, conPIDDeh.trigger) annotation (Line(points={{-122,-96},{
          10,-96},{10,54},{4,54},{4,68}}, color={255,0,255}));
  annotation (
    defaultComponentName = "conCoiCoo",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(lineColor = {179, 151, 128}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                  FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 10), Text(textColor = {28, 108, 200}, extent = {{-90, 180}, {90, 76}}, textString = "%name", textStyle = {TextStyle.Bold}), Text(textColor = {28, 108, 200}, extent = {{-98, 92}, {-54, 78}}, textString = "supFanProof"), Text(textColor = {28, 108, 200}, extent = {{-110, 62}, {-70, 50}}, textString = "saT"), Text(textColor = {28, 108, 200}, extent = {{-98, 34}, {-58, 22}}, textString = "supCooSP"), Text(textColor = {28, 108, 200}, extent = {{-108, -26}, {-68, -38}}, textString = "ccT"), Text(textColor = {28, 108, 200}, extent = {{-100, -52}, {-60, -64}}, textString = "erwHum"), Text(textColor = {28, 108, 200}, extent = {{-108, -78}, {-68, -90}}, textString = "erwT"), Text(textColor = {28, 108, 200}, extent = {{-98, 0}, {-58, -12}}, textString = "dehumMode"), Text(textColor = {28, 108, 200}, extent = {{62, 8}, {106, -8}}, textString = "yCC"), Rectangle(lineColor = {28, 108, 200}, fillColor = {85, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-22, 68}, {6, -66}}), Rectangle(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-14, 58}, {68, 56}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-16, 60}, {-10, 54}}), Rectangle(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-2, -56}, {80, -58}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-4, -54}, {2, -60}}), Polygon(lineColor = {127, 0, 0}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{42, -58}, {30, -66}, {30, -50}, {42, -58}}), Polygon(lineColor = {127, 0, 0}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{42, -58}, {54, -66}, {54, -50}, {42, -58}}), Ellipse(lineColor = {127, 0, 0}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{38, -54}, {46, -62}}), Ellipse(lineColor = {127, 0, 0}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{36, -32}, {48, -46}}), Rectangle(lineColor = {127, 0, 0}, fillColor = {255, 255, 255},
            fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{36, -38}, {48, -48}}), Line(points = {{42, -48}, {42, -54}}, color = {127, 0, 0})}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Documentation(revisions = "<html>
<ul>
<li>
September 17, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info="<html>
<h4>Normal Operation</h4>
<p>When the DOAS is energized (<span style=\"font-family: Courier New;\">uFanSupPro</span>) the cooling coil will be commanded (<span style=\"font-family: Courier New;\">yCoiCoo</span>) to maintain the supply air temperature (<span style=\"font-family: Courier New;\">TAirSup</span>) at the supply air temperature cooling set point (<span style=\"font-family: Courier New;\">TAirSupSetCoo</span>). </p>
<h4>Dehumidification Operation</h4>
<p>When the DOAS is energized (<span style=\"font-family: Courier New;\">uFanSupPro</span>) and in dehumidification mode (<span style=\"font-family: Courier New;\">uDeh</span>) the cooling coil will be commanded (<span style=\"font-family: Courier New;\">yCoiCoo</span>) to maintain the cooling coil temperature (<span style=\"font-family: Courier New;\">ccT</span>) at an adjustable value (<span style=\"font-family: Courier New;\">erwDPadj</span>) below the energy recovery supply dewpoint (TDewPoi). The dewpoint value is calculated from the energy recovery supply relative humidity <span style=\"font-family: Courier New;\">(phiAirEneRecWhe</span>) and temperature <span style=\"font-family: Courier New;\">(TAirEneRecWhe</span>).</p>
</html>"));
end CoolingCoil;
