within Buildings.Fluid.ZoneEquipment.FanCoilUnit;
model FourPipe "System model for a four-pipe fan coil unit"

  extends Buildings.Fluid.ZoneEquipment.BaseClasses.EquipmentInterfaces(
    final cooCoiTyp=Buildings.Fluid.ZoneEquipment.BaseClasses.Types.CooSou.chiWat,
    oaPorTyp=Buildings.Fluid.ZoneEquipment.BaseClasses.Types.OAPorts.oaMix);

  parameter Modelica.Units.SI.HeatFlowRate QHeaCoi_flow_nominal(
    final min = 0)
    "Nominal heat flow rate of electric heating coil"
    annotation(Dialog(enable=not has_HW, group="Heating coil parameters"));

  parameter Modelica.Units.SI.PressureDifference dpAir_nominal
    "Total pressure difference across supply and return ports in airloop"
    annotation(Dialog(group="System parameters"));

  parameter Modelica.Units.SI.ThermalConductance UAHeaCoi_nominal
    "Thermal conductance at nominal flow, used to compute heat capacity"
    annotation(Dialog(enable=has_HW, group="Heating coil parameters"));

  parameter Modelica.Units.SI.ThermalConductance UACooCoi_nominal
    "Nominal Thermal conductance, used to compute heat capacity"
    annotation(Dialog(group="Cooling coil parameters"));

  Buildings.Fluid.Sensors.TemperatureTwoPort TAirHea(redeclare final package
      Medium =
        MediumA, final m_flow_nominal=mAir_flow_nominal)
    "Heating coil discharge air temperature sensor"
    annotation (Placement(transformation(extent={{30,-20},{50,0}})));

  Buildings.Fluid.HeatExchangers.DryCoilCounterFlow heaCoiHW(
    redeclare final package Medium1 = MediumHW,
    redeclare final package Medium2 = MediumA,
    final m1_flow_nominal=mHotWat_flow_nominal,
    final m2_flow_nominal=mAir_flow_nominal,
    final dp1_nominal=0,
    final dp2_nominal=0,
    final UA_nominal=UAHeaCoi_nominal,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) if
       has_HW "Hot water heating coil" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,-30})));
  Buildings.Fluid.HeatExchangers.WetCoilCounterFlow cooCoi(
    redeclare final package Medium1 = MediumCHW,
    redeclare final package Medium2 = MediumA,
    final m1_flow_nominal=mChiWat_flow_nominal,
    final m2_flow_nominal=mAir_flow_nominal,
    final dp1_nominal=0,
    final dp2_nominal=0,
    final UA_nominal=UACooCoi_nominal,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Chilled-water cooling coil" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={130,-10})));

  Buildings.Fluid.Movers.FlowControlled_m_flow fan(
    redeclare final package Medium = MediumA,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final m_flow_nominal=mAir_flow_nominal,
    final per=fanPer,
    final dp_nominal=dpAir_nominal,
    dpMax=15000)
    "Supply fan"
    annotation (Placement(transformation(extent={{200,-20},{220,0}})));

  replaceable parameter Buildings.Fluid.Movers.Data.Generic fanPer
    constrainedby Buildings.Fluid.Movers.Data.Generic
    "Record with performance data for supply fan"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{50,100},{70,120}})),
      Dialog(group="Fan parameters"));

  Buildings.Fluid.FixedResistances.PressureDrop totRes(
    final m_flow_nominal=mAir_flow_nominal,
    final dp_nominal=dpAir_nominal,
    final allowFlowReversal=true,
    redeclare final package Medium = MediumA)
    "Total resistance"
    annotation (Placement(transformation(extent={{156,-14},{176,6}})));

  Buildings.Fluid.HeatExchangers.HeaterCooler_u heaCoiEle(
    redeclare final package Medium = MediumA,
    final m_flow_nominal=mAir_flow_nominal,
    final dp_nominal=0,
    final Q_flow_nominal=QHeaCoi_flow_nominal) if not has_HW
    "Electric heating coil"
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));

  Modelica.Blocks.Math.Gain gai(
    final k=mAir_flow_nominal)
    "Find mass flowrate value by multiplying nominal flowrate by normalized fan speed signal"
    annotation (Placement(transformation(extent={{-20,70},{0,90}})));

  Modelica.Blocks.Math.Gain gaiFanNor(
    final k=1/mAir_flow_nominal)
    "Normalized fan signal"
    annotation (Placement(transformation(extent={{300,100},{320,120}})));

equation
  connect(heaCoiHW.port_b2, TAirHea.port_a) annotation (Line(points={{0,-24},{
          20,-24},{20,-10},{30,-10}},  color={0,127,255}));
  connect(TAirHea.port_b, cooCoi.port_a2) annotation (Line(points={{50,-10},{80,
          -10},{80,-4},{120,-4}}, color={0,127,255}));
  connect(cooCoi.port_b2, totRes.port_a)
    annotation (Line(points={{140,-4},{156,-4}}, color={0,127,255}));
  connect(totRes.port_b, fan.port_a) annotation (Line(points={{176,-4},{180,-4},
          {180,-10},{200,-10}}, color={0,127,255}));
  connect(heaCoiEle.port_b, TAirHea.port_a) annotation (Line(points={{0,30},{20,
          30},{20,-10},{30,-10}}, color={0,127,255}));
  connect(gai.y, fan.m_flow_in)
    annotation (Line(points={{1,80},{210,80},{210,2}}, color={0,0,127}));
  connect(fan.m_flow_actual, gaiFanNor.u) annotation (Line(points={{221,-5},{
          240,-5},{240,110},{298,110}}, color={0,0,127}));
  connect(uFan, gai.u)
    annotation (Line(points={{-380,120},{-202,120},{-202,80},{-22,80}},
                                                  color={0,0,127}));
  connect(uHea, valHW.y) annotation (Line(points={{-380,-80},{-60,-80},{-60,-80},
          {-48,-80}}, color={0,0,127}));
  connect(uHea, heaCoiEle.u) annotation (Line(points={{-380,-80},{-60,-80},{-60,
          36},{-22,36}}, color={0,0,127}));
  connect(uCoo, valCHW.y) annotation (Line(points={{-380,-40},{-80,-40},{-80,
          -148},{80,-148},{80,-80},{92,-80}},
                                        color={0,0,127}));
  connect(valHW.port_b, heaCoiHW.port_b1) annotation (Line(points={{-36,-70},{
          -36,-36},{-20,-36}},
                           color={0,127,255}));
  connect(heaCoiHW.port_a1, VHW_flow.port_b)
    annotation (Line(points={{0,-36},{4,-36},{4,-80}}, color={0,127,255}));
  connect(valCHW.port_b, cooCoi.port_b1) annotation (Line(points={{104,-70},{104,
          -16},{120,-16}}, color={0,127,255}));
  connect(cooCoi.port_a1, VCHW_flow.port_b) annotation (Line(points={{140,-16},{
          144,-16},{144,-80}}, color={0,127,255}));
  connect(gaiFanNor.y, yFan_actual)
    annotation (Line(points={{321,110},{370,110}}, color={0,0,127}));
  if not has_ven then
  end if;
  connect(vAirMix.port_b, heaCoiEle.port_a) annotation (Line(points={{-80,0},{
          -40,0},{-40,30},{-20,30}},  color={0,127,255}));
  connect(vAirMix.port_b, heaCoiHW.port_a2) annotation (Line(points={{-80,0},{
          -40,0},{-40,-24},{-20,-24}},  color={0,127,255}));
  connect(fan.port_b, TAirLvg.port_a)
    annotation (Line(points={{220,-10},{240,-10},{240,0}}, color={0,127,255}));
  annotation (defaultComponentName = "fanCoiUni",
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
            -200},{200,200}}), graphics={Rectangle(
          extent={{-200,200},{200,-200}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,200},{100,240}},
          textString="%name",
          textColor={0,0,255})}),
    Diagram(coordinateSystem(preserveAspectRatio=false,
      extent={{-360,-180},{360,140}})),
    Documentation(info="<html>
    <p>
    This is a four-pipe fan coil unit system model. The system contains
    a supply fan, an electric or hot-water heating coil, a chilled-water cooling coil,
    and a mixing box. 
    </p>
    The control modules for the system are implemented separately in
    <a href=\"modelica://Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls\">
    Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls</a>:
    <ul>
    <li>
    <a href=\"modelica://Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls.ConstantFanVariableWaterFlowrate\">
    ConstantFanVariableWaterFlowrate</a>:
    Modulate the cooling coil and heating coil valve positions to regulate the zone temperature
    between the heating and cooling setpoints. The fan is enabled and operated at the 
    maximum speed when there are zone heating or cooling loads. It is run at minimum speed when
    zone is occupied but there are no loads.
    </li>
    <li>
    <a href=\"modelica://Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls.VariableFanConstantWaterFlowrate\">
    VariableFanConstantWaterFlowrate</a>:
    Modulate the fan speed to regulate the zone temperature between the heating 
    and cooling setpoints. It is run at minimum speed when zone is occupied but 
    there are no loads. The heating and cooling coil valves are completely opened 
    when there are zone heating or cooling loads, respectively.
    </li>
    <li>
    <a href=\"modelica://Buildings.Fluid.ZoneEquipment.FanCoilUnit.Controls.MultispeedFanConstantWaterFlowrate\">
    MultispeedFanConstantWaterFlowrate</a>:
    Modulate the fan speed to regulate the zone temperature between the heating 
    and cooling setpoints. It is set at a range of fixed values between the maximum 
    and minimum speed, based on the heating and cooling loop signals generated. 
    It is run at minimum speed when zone is occupied but there are no loads. The 
    heating and cooling coil valves are completely opened when there are zone 
    heating or cooling loads, respectively.
    </li>
    </ul>
    </html>
    ", revisions="<html>
    <ul>
    <li>
    August 03, 2022 by Karthik Devaprasad, Sen Huang:<br/>
    First implementation.
    </li>
    </ul>
    </html>"));
end FourPipe;
