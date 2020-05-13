within Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.Validation;
block CapacityRequirement
  "Validation model for CapacityRequirement"

  parameter Real TNomSupSet(final unit="K",
                            final displayUnit="degC",
                            final quantity="ThermodynamicTemperature") = 333.15
  "Nominal hot water supply temperature setpoint";

  parameter Real TNomRet(final unit="K",
                         final displayUnit="degC",
                         final quantity="ThermodynamicTemperature") = 322.04
  "Nominal measured hot water return temperature";

  parameter Real VNomFloRat(final unit="m3/s",
                            final displayUnit="m3/s",
                            final quantity="VolumeFlowRate") = 1
  "Nominal measured primary flow-rate";

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement
    capReq(final avePer=300)
    "Scenario with sine input for return temperature"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement
    capReq1(final avePer=300)
    "Scenario with sine input for supply setpoint temperature"
    annotation (Placement(transformation(extent={{60,40},{80,60}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement
    capReq2(final avePer=300)
    "Scenario with sine input for flow-rate"
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement
    capReq3(final avePer=300)
    "Scenario with sine input for all inputs"
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));

protected
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con(final k=TNomSupSet)
    "Constant input"
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin(
    final amplitude=TNomSupSet - TNomRet,
    final freqHz=1/3600,
    final offset=TNomRet)
    "Sine input"
    annotation (Placement(transformation(extent={{-90,40},{-70,60}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con1(final k=VNomFloRat)
    "Constant input"
    annotation (Placement(transformation(extent={{-90,10},{-70,30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con2(final k=TNomRet)
    "Constant input"
    annotation (Placement(transformation(extent={{10,40},{30,60}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin1(
    final amplitude=TNomSupSet - TNomRet,
    final freqHz=1/3600,
    final offset=TNomSupSet)
    "Sine input"
    annotation (Placement(transformation(extent={{10,70},{30,90}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con3(
    final k=VNomFloRat)
    "Constant input"
    annotation (Placement(transformation(extent={{10,10},{30,30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con4(
    final k=TNomSupSet)
    "Constant input"
    annotation (Placement(transformation(extent={{-90,-30},{-70,-10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin2(
    final amplitude=VNomFloRat,
    final freqHz=1/3600,
    final offset=VNomFloRat)
    "Sine input"
    annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant con5(final k=TNomRet)
    "Constant input"
    annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin3(
    final amplitude=TNomSupSet - TNomRet,
    final freqHz=2/3600,
    final offset=TNomRet)
    "Sine input"
    annotation (Placement(transformation(extent={{10,-60},{30,-40}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin4(
    final amplitude=TNomSupSet - TNomRet,
    final freqHz=1/3600,
    final offset=TNomSupSet)
    "Sine input"
    annotation (Placement(transformation(extent={{10,-30},{30,-10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin5(
    final amplitude=VNomFloRat,
    final freqHz=3/3600,
    final offset=VNomFloRat)
    "Sine input"
    annotation (Placement(transformation(extent={{10,-90},{30,-70}})));

equation
  connect(con.y, capReq.TSupSet) annotation (Line(points={{-68,80},{-50,80},{-50,
          57},{-42,57}}, color={0,0,127}));
  connect(sin.y, capReq.TRet)
    annotation (Line(points={{-68,50},{-42,50}}, color={0,0,127}));
  connect(con1.y, capReq.VHotWat_flow) annotation (Line(points={{-68,20},{-50,20},
          {-50,43},{-42,43}}, color={0,0,127}));
  connect(con3.y, capReq1.VHotWat_flow) annotation (Line(points={{32,20},{50,20},
          {50,43},{58,43}}, color={0,0,127}));
  connect(con4.y, capReq2.TSupSet) annotation (Line(points={{-68,-20},{-50,-20},
          {-50,-43},{-42,-43}}, color={0,0,127}));
  connect(sin3.y, capReq3.TRet)
    annotation (Line(points={{32,-50},{58,-50}}, color={0,0,127}));
  connect(con2.y, capReq1.TRet)
    annotation (Line(points={{32,50},{58,50}}, color={0,0,127}));
  connect(sin1.y, capReq1.TSupSet) annotation (Line(points={{32,80},{50,80},{50,
          57},{58,57}}, color={0,0,127}));
  connect(con5.y, capReq2.TRet)
    annotation (Line(points={{-68,-50},{-42,-50}}, color={0,0,127}));
  connect(sin2.y, capReq2.VHotWat_flow) annotation (Line(points={{-68,-80},{-50,
          -80},{-50,-57},{-42,-57}}, color={0,0,127}));
  connect(sin5.y, capReq3.VHotWat_flow) annotation (Line(points={{32,-80},{50,-80},
          {50,-57},{58,-57}}, color={0,0,127}));
  connect(sin4.y, capReq3.TSupSet) annotation (Line(points={{32,-20},{50,-20},{50,
          -43},{58,-43}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(file="./Resources/Scripts/Dymola/Controls/OBC/ASHRAE/PrimarySystem/BoilerPlant/Staging/Validation/CapacityRequirement.mos"
        "Simulate and plot"),
    experiment(
      StopTime=7200,
      Interval=1),
    Documentation(info="<html>
        <p>
        This example validates
        <a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement\">
        Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.CapacityRequirement</a>.
        </p>
        </html>", revisions="<html>
        <ul>
        <li>
        May 13, 2020, by Karthik Devaprasad:<br/>
        First implementation.
        </li>
        </ul>
        </html>"));
end CapacityRequirement;
