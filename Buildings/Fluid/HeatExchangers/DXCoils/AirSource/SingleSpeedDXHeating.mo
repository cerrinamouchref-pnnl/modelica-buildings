<<<<<<< HEAD:Buildings/Fluid/HeatExchangers/DXCoils/AirCooled/SingleSpeedDXHeating.mo
within Buildings.Fluid.HeatExchangers.DXCoils.AirCooled;
model SingleSpeedDXHeating "Single speed DX cooling coil"
  extends
    Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.PartialDXHeatingCoil(
    dryCoi(
      final variableSpeedCoil=false,
      redeclare
          Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.CoolingCapacityAirCooled
          cooCap),
    computeReevaporation=false,
    redeclare
      Buildings.Fluid.HeatExchangers.DXCoils.AirCooled.Data.Generic.DXCoil
=======
within Buildings.Fluid.HeatExchangers.DXCoils.AirSource;
model SingleSpeedDXHeating "Single speed DX heating coil"
  extends
    Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.PartialDXHeatingCoil(
    dxCoi(final variableSpeedCoil=false, redeclare
        Buildings.Fluid.HeatExchangers.DXCoils.BaseClasses.CoolingCapacityAirCooled
        cooCap),
    computeReevaporation=false,
    redeclare
      Buildings.Fluid.HeatExchangers.DXCoils.AirSource.Data.Generic.DXCoil
>>>>>>> Xing_fork/issue3288_dxCoilHeating_renameCooledSource:Buildings/Fluid/HeatExchangers/DXCoils/AirSource/SingleSpeedDXHeating.mo
      datCoi,
    use_mCon_flow=false);
  Modelica.Blocks.Sources.Constant speRat(final k=1) "Speed ratio"
    annotation (Placement(transformation(extent={{-56,58},{-44,70}})));
  Modelica.Blocks.Interfaces.BooleanInput on
    "Set to true to enable compressor, or false to disable compressor"
    annotation (Placement(transformation(extent={{-120,70},{-100,90}})));
protected
  Modelica.Blocks.Math.BooleanToInteger onSwi(
    final integerTrue=1,
    final integerFalse=0) "On/off switch"
    annotation (Placement(transformation(extent={{-56,74},{-44,86}})));
initial equation
  assert(datCoi.nSta == 1, "Must have one stage only for single speed performance data");
equation
<<<<<<< HEAD:Buildings/Fluid/HeatExchangers/DXCoils/AirCooled/SingleSpeedDXHeating.mo
  connect(speRat.y, dryCoi.speRat) annotation (Line(
      points={{-43.4,64},{-40,64},{-40,57.6},{-21,57.6}},
=======
  connect(speRat.y, dxCoi.speRat) annotation (Line(
      points={{-43.4,64},{-40,64},{-40,59.6},{-21,59.6}},
>>>>>>> Xing_fork/issue3288_dxCoilHeating_renameCooledSource:Buildings/Fluid/HeatExchangers/DXCoils/AirSource/SingleSpeedDXHeating.mo
      color={0,0,127},
      smooth=Smooth.None));
  connect(on, onSwi.u) annotation (Line(
      points={{-110,80},{-57.2,80}},
      color={255,0,255},
      smooth=Smooth.None));
<<<<<<< HEAD:Buildings/Fluid/HeatExchangers/DXCoils/AirCooled/SingleSpeedDXHeating.mo
  connect(onSwi.y, dryCoi.stage) annotation (Line(
      points={{-43.4,80},{-34,80},{-34,60},{-21,60}},
=======
  connect(onSwi.y, dxCoi.stage) annotation (Line(
      points={{-43.4,80},{-34,80},{-34,62},{-21,62}},
>>>>>>> Xing_fork/issue3288_dxCoilHeating_renameCooledSource:Buildings/Fluid/HeatExchangers/DXCoils/AirSource/SingleSpeedDXHeating.mo
      color={255,127,0},
      smooth=Smooth.None));
  annotation (defaultComponentName="sinSpeDX", Documentation(info="<html>
<p>
<<<<<<< HEAD:Buildings/Fluid/HeatExchangers/DXCoils/AirCooled/SingleSpeedDXHeating.mo
This model can be used to simulate an air-cooled DX heating coil with single speed compressor.
=======
This model can be used to simulate an air-source DX heating coil with single speed compressor.
>>>>>>> Xing_fork/issue3288_dxCoilHeating_renameCooledSource:Buildings/Fluid/HeatExchangers/DXCoils/AirSource/SingleSpeedDXHeating.mo
</p>
</html>",
revisions="<html>
<ul>
<li>
March 8, 2023, by Xing Lu:<br/>
<<<<<<< HEAD:Buildings/Fluid/HeatExchangers/DXCoils/AirCooled/SingleSpeedDXHeating.mo
Initial commit.
=======
Initial implementation.
>>>>>>> Xing_fork/issue3288_dxCoilHeating_renameCooledSource:Buildings/Fluid/HeatExchangers/DXCoils/AirSource/SingleSpeedDXHeating.mo
</li>
</ul>
</html>"),
    Icon(graphics={Text(
          extent={{-140,132},{-96,112}},
          textColor={0,0,255},
          textString="on")}));
end SingleSpeedDXHeating;
