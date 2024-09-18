within Buildings.Controls.OBC.FDE.DOAS;
block DOAScontroller "DOAS controller built from DOAS blocks."

 parameter Real erwDPadj(
  final unit = "K",
  final quantity = "TemperatureDifference") = 5
  "Value subtracted from ERW supply air dewpoint."
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter CDL.Types.SimpleController controllerTypeDeh=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for cooling air in dehumidification mode"
    annotation(Dialog(tab="Controller parameters", group = "Dehmidification mode parameters"));

  parameter Real kDeh(
  final unit="1") = 1
    "Gain of conPIDDeh controller"
    annotation(Dialog(tab="Controller parameters", group = "Dehmidification mode parameters"));

  parameter Real TiDeh(
  final unit="s") = 60
    "Time constant of integrator block for conPIDDeh controller"
    annotation(Dialog(tab="Controller parameters", group = "Dehmidification mode parameters"));

  parameter Real TdDeh(
  final unit="s") = 0.1
    "Time constant of derivative block for conPIDDeh controller"
    annotation(Dialog(tab="Controller parameters", group = "Dehmidification mode parameters"));

  parameter CDL.Types.SimpleController controllerTypeRegOpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller for regular cooling coil operation mode"
    annotation(Dialog(tab="Controller parameters", group = "Cooling Coil parameters"));

  parameter Real kRegOpe(
  final unit="1") = 1
    "Gain of conPIDRegOpe controller"
    annotation(Dialog(tab="Controller parameters", group = "Cooling Coil parameters"));


  parameter Real TiRegOpe(
  final unit="s")=60
    "Time constant of integrator block for conPIDRegOpe controller"
    annotation(Dialog(tab="Controller parameters", group = "Cooling Coil parameters"));


  parameter Real TdRegOpe(
  final unit="s")=0.1
    "Time constant of derivative block for conPIDRegOpe controller"
    annotation(Dialog(tab="Controller parameters", group = "Cooling Coil parameters"));


  parameter Real dehumSet(
    final min=0,
    final max=100)=0.6
    "Dehumidification set point."
    annotation(Dialog(tab="Setpoints", group = "Dehumidification mode parameters"));


  parameter Real dehumOff(
    final min=0,
    final max=100)=0.05
   "Relative humidity offset"
   annotation(Dialog(tab="Limits and Thresholds", group = "Dehumidification mode parameters"));


  parameter Real timThrDehDis(
    final unit="s",
    final quantity="Time")=600
    "Continuous time period for which measured relative humidity needs to fall below relative humidity threshold before dehumidification mode is disabled"
    annotation(Dialog(tab="Limits and Thresholds", group = "Dehumidification mode parameters"));

  parameter Real timDelDehEna(
    final unit="s",
    final quantity="Time")=120
    "Continuous time period for which supply fan needs to be on before enabling dehumidifaction mode"
    annotation(Dialog(tab="Limits and Thresholds", group = "Dehumidification mode parameters"));

  parameter Real timThrDehEna(
    final unit="s",
    final quantity="Time")=5
    "Continuous time period for which relative humidity rises above set point before dehumidifcation mode is enabled"
    annotation(Dialog(tab="Limits and Thresholds", group = "Dehumidification mode parameters"));

  parameter Real dTEcoThr(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 2
  "Threshold temperature difference between return air and outdoor air temperature above which economizer mode is enabled"
  annotation(Dialog(tab="Limits and Thresholds", group = "Economizer mode parameters"));

    parameter Real dTThrEneRec(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 7
  "Absolute temperature difference threshold between outdoor air and return air temperature above which energy recovery is enabled"
  annotation(Dialog(tab="Limits and Thresholds", group = "Energy recovery wheel parameters"));


   parameter Real dThys(
  final unit = "K",
  final displayUnit = "degC",
  final quantity = "ThermodynamicTemperature") = 0.5
  "Delay time period after temperature difference threshold is crossed for enabling energy recovery mode"
  annotation(Dialog(tab="Limits and Thresholds", group = "Energy recovery wheel parameters"));


  parameter Real timDelEneRec(
  final unit = "s",
  final quantity = "Time") = 300
  "Minimum delay after OAT/RAT delta falls below set point."
  annotation(Dialog(tab="Limits and Thresholds", group = "Energy recovery wheel parameters"));


  parameter CDL.Types.SimpleController controllerTypeEneWheHea=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI controller for heating loop"
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));


  parameter Real kEneWheHea(
  final unit = "1") = 0.5
  "PID heating loop gain value."
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));


  parameter Real TiEneWheHea(
  final unit = "s") = 60
  "PID  heating loop time constant of integrator."
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter Real TdEneWheHea(
  final unit = "s") = 0.1
  "PID heatig loop time constant of derivative block"
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter Real kEneWheCoo(
  final unit = "1") = 0.5
  "PID cooling loop gain value."
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter Real TiEneWheCoo(
  final unit = "s") = 60 "PID cooling loop time constant of integrator."
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter CDL.Types.SimpleController controllerTypeEneWheCoo=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI controller for cooling loop"
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

  parameter Real TdEneWheCoo(
  final unit = "s") = 0.1
  "PID cooling loop time constant of derivative block"
  annotation(Dialog(tab="Controller parameters", group = "Energy recovery wheel parameters"));

   parameter Real dPSetBui(
  final unit = "Pa",
  final quantity = "PressureDifference") = 15
  "Building static pressure difference set point"
  annotation(Dialog(tab="Setpoints", group = "Exhuast fan parameters"));

  parameter Real kExhFan(
  final unit = "1") = 0.5
  "PID heating loop gain value."
  annotation(Dialog(tab="Controller parameters", group = "Exhuast fan parameters"));

  parameter Real TiExhFan(
  final unit = "s") = 60
  "PID loop time constant of integrator."
  annotation(Dialog(tab="Controller parameters", group = "Exhuast fan parameters"));

  parameter Real TdExhFan(
  final unit= "s") = 0.1 "Time constant of derivative block"
  annotation(Dialog(tab="Controller parameters", group = "Exhuast fan parameters"));

  parameter CDL.Types.SimpleController controllerTypeExhFan=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "Type of controller"
    annotation(Dialog(tab="Controller parameters", group = "Exhuast fan parameters"));

  parameter CDL.Types.SimpleController controllerTypeCoiHea=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
   "Type of controller"
   annotation(Dialog(tab="Controller parameters", group = "Heaing coil parameters"));

  parameter Real kCoiHea(
   final unit= "1") = 0.5
  "Heating coil SAT PI gain value k."
  annotation(Dialog(tab="Controller parameters", group = "Heaing coil parameters"));

  parameter Real TiCoiHea(
   final unit= "s") = 60
  "Heating coil SAT PI time constant value Ti."
  annotation(Dialog(tab="Controller parameters", group = "Heaing coil parameters"));

  parameter Real TdCoiHea(
  final unit= "s") = 0.1 "Time constant of derivative block"
  annotation(Dialog(tab="Controller parameters", group = "Heaing coil parameters"));

     parameter Boolean is_vav = true
  "True: System has zone terminals with variable damper position. False: System has zone terminals with constant damper position."
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real yMinDamSet(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 125
  "Minimum down duct static pressure reset value"
  annotation(Dialog(tab="Limits and Thresholds", group = "Supply fan parameters"));

  parameter Real yMaxDamSet(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 500
  "Maximum down duct static pressure reset value"
  annotation(Dialog(tab="Limits and Thresholds", group = "Supply fan parameters"));

  parameter Real damSet(
  min = 0,
  max = 1,
  final unit = "1") = 0.9
  "DDSP terminal damper percent open set point"
  annotation(Dialog(tab="Setpoints", group = "Supply fan parameters"));

  parameter Real kDam(
   final unit= "1") = 0.5
  "Damper position setpoint PI gain value k."
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real TiDam(
   final unit= "s") = 60
  "Damper position setpoint PI time constant value Ti."
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real TdDam(
   final unit= "s") = 0.1 "Time constant of derivative block for conPIDDam"
   annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter CDL.Types.SimpleController controllerTypeDam=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "Type of controller"
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real dPDucSetCV(
  min = 0,
  final unit = "Pa",
  final quantity = "PressureDifference") = 250 "Constant volume down duct static pressure set point"
  annotation(Dialog(tab="Setpoints", group = "Supply fan parameters"));

  parameter Real fanSpeMin(
   final unit= "m/s") = 0.0000001
  "Minimum Fan Speed"
  annotation(Dialog(tab="Limits and Thresholds", group = "Supply fan parameters"));

  parameter Real kFanSpe(
   final unit= "1") = 0.5 "
   Fan speed set point SAT PI gain value k."
   annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real TdFanSpe(
   final unit= "s") = 60
   "Time constant of derivative block for conPIDFanSpe"
   annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real TiFanSpe(
   final unit= "s") = 0.000025
  "Fan speed set point SAT PI time constant value Ti."
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter CDL.Types.SimpleController controllerTypeFanSpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "Type of controller"
    annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"));

  parameter Real TSupLowSet(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=273.15+25
   "Minimum primary supply air temperature reset value"
   annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));

  parameter Real TSupHigSet(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=273.15+28
   "Maximum primary supply air temperature reset value"
   annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));

  parameter Real THigZon(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=273.15+25
   "Maximum zone temperature reset value"
    annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));

  parameter Real TLowZon(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=273.15+21
   "Minimum zone temperature reset value"
    annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));

  parameter Real TSupCooOff(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=2
   "Supply air temperature cooling set point offset."
    annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));

  parameter Real TSupHeaOff(
   final unit="K",
   final displayUnit="degC",
   final quantity="ThermodynamicTemperature")=2
   "Supply air temperature heating set point offset."
    annotation(Dialog(tab="Limits and Thresholds", group = "Supply temperature setpoint parameters"));


// ---inputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput Occ
    "True when occupied mode is active"
    annotation(Placement(transformation(extent = {{-142, 56}, {-102, 96}}),
      iconTransformation(extent = {{-140, 76}, {-100, 116}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uDamMaxOpe
    "Most open damper position from all terminal units served." annotation (
      Placement(transformation(extent={{-142,30},{-102,70}}),
        iconTransformation(extent={{-140,52},{-100,92}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when supply fan is proven on." annotation (Placement(transformation(
          extent={{-140,4},{-100,44}}), iconTransformation(extent={{-140,28},{-100,
            68}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dPAirDucSta(final unit="Pa",
      final quantity="PressureDifference")
    "Down duct static pressure measurement." annotation (Placement(
        transformation(extent={{-140,-22},{-100,18}}), iconTransformation(
          extent={{-140,4},{-100,44}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirRet(
    final min=0,
    final max=100,
    final displayUnit="rh") "Return air relative humidity sensor." annotation (
      Placement(transformation(extent={{-140,-80},{-100,-40}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSup(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Supply air temperature sensor."
    annotation (Placement(transformation(extent={{-140,-106},{-100,-66}}),
        iconTransformation(extent={{-140,-72},{-100,-32}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirDisCoiCoo(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Cooling coil discharge air temperature sensor." annotation (Placement(
        transformation(extent={{-140,-184},{-100,-144}}), iconTransformation(
          extent={{-140,-146},{-100,-106}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirEneRecWhe(
    final min=0,
    final max=100,
    final displayUnit="rh") "ERW relative humidity sensor" annotation (
      Placement(transformation(extent={{-140,-210},{-100,-170}}),
        iconTransformation(extent={{-140,-172},{-100,-132}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupEneWhe(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "ERW dry bulb temperature sensor." annotation (Placement(transformation(
          extent={{-140,-236},{-100,-196}}), iconTransformation(extent={{-140,-196},
            {-100,-156}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirOut(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Outside air temperature"
    annotation (Placement(transformation(extent={{-140,-158},{-100,-118}}),
        iconTransformation(extent={{-140,-122},{-100,-82}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirRet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Return air temperature sensor."
    annotation (Placement(transformation(extent={{-140,-132},{-100,-92}}),
        iconTransformation(extent={{-140,-96},{-100,-56}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanExhPro
    "True when exhaust fan is proven on." annotation (Placement(transformation(
          extent={{-140,-262},{-100,-222}}), iconTransformation(extent={{-140,-220},
            {-100,-180}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dPAirStaBui(final unit="Pa",
      final quantity="PressureDifference") "Building static pressure"
    annotation (Placement(transformation(extent={{-140,-290},{-100,-250}}),
        iconTransformation(extent={{-140,-244},{-100,-204}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirHig(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Highest space temperature reported from all terminal units." annotation (
      Placement(transformation(extent={{-140,-50},{-100,-10}}),
        iconTransformation(extent={{-140,-46},{-100,-6}})));

// ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yFanSup
    "Command supply fan to start when true." annotation (Placement(
        transformation(extent={{102,54},{142,94}}), iconTransformation(extent={
            {102,34},{142,74}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yFanSupSpe
    "Supply fan speed command" annotation (Placement(transformation(extent={{
            102,28},{142,68}}), iconTransformation(extent={{102,2},{142,42}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoiCoo
    "Cooling coil control signal" annotation (Placement(transformation(extent={
            {102,2},{142,42}}), iconTransformation(extent={{102,-30},{142,10}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoiHea
    "Reheat coil valve command." annotation (Placement(transformation(extent={{
            102,-24},{142,16}}), iconTransformation(extent={{102,-60},{142,-20}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yBypDam
    "Bypass damper command; true when commanded full open." annotation (
      Placement(transformation(extent={{102,-50},{142,-10}}),
        iconTransformation(extent={{102,-90},{142,-50}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yEneRecWheEna
    "Command to start the energy recovery wheel." annotation (Placement(
        transformation(extent={{102,-74},{142,-34}}), iconTransformation(extent=
           {{102,-120},{142,-80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yEneRecWheSpe
    "Energy recovery wheel speed command." annotation (Placement(transformation(
          extent={{102,-108},{142,-68}}), iconTransformation(extent={{102,-152},
            {142,-112}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yExhFanSta
    "Command exhaust fan to start when true." annotation (Placement(
        transformation(extent={{100,-140},{140,-100}}), iconTransformation(
          extent={{102,-184},{142,-144}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yExhFanSpe
    "Exhaust fan speed command" annotation (Placement(transformation(extent={{
            104,-188},{144,-148}}), iconTransformation(extent={{102,-216},{142,
            -176}})));

  Buildings.Controls.OBC.FDE.DOAS.SupplyFanController SFcon(
    is_vav=is_vav,
    yMinDamSet=yMinDamSet,
    yMaxDamSet=yMaxDamSet,
    damSet=damSet,
    kDam=kDam,
    TiDam=TiDam,
    TdDam=TdDam,
    controllerTypeDam=controllerTypeDam,
    dPDucSetCV=dPDucSetCV,
    fanSpeMin=fanSpeMin,
    kFanSpe=kFanSpe,
    TdFanSpe=TdFanSpe,
    TiFanSpe=TiFanSpe,
    controllerTypeFanSpe=controllerTypeFanSpe)
    "This block manages start, stop, status, and speed of the supply fan."
    annotation(Placement(transformation(extent={{-50,54},{-30,74}})));

  Buildings.Controls.OBC.FDE.DOAS.EnergyWheel ERWcon(
    dTThrEneRec=dTThrEneRec,
    dThys=dThys,
    timDelEneRec=timDelEneRec,
    controllerTypeEneWheHea=controllerTypeEneWheHea,
    kEneWheHea=kEneWheHea,
    TiEneWheHea=TiEneWheHea,
    TdEneWheHea=TdEneWheHea,
    kEneWheCoo=kEneWheCoo,
    TiEneWheCoo=TiEneWheCoo,
    controllerTypeEneWheCoo=controllerTypeEneWheCoo,
    TdEneWheCoo=TdEneWheCoo)
    "This block commands the energy recovery wheel and associated bypass dampers."
    annotation(Placement(transformation(extent = {{58, -64}, {78, -44}})));

  Buildings.Controls.OBC.FDE.DOAS.CoolingCoil Cooling(
    erwDPadj=erwDPadj,
    controllerTypeDeh=controllerTypeDeh,
    kDeh=kDeh,
    TiDeh=TiDeh,
    TdDeh=TdDeh,
    controllerTypeRegOpe=controllerTypeRegOpe,
    kRegOpe=kRegOpe,
    TiRegOpe=TiRegOpe,
    TdRegOpe=TdRegOpe)
    "This block commands the cooling coil."
     annotation(Placement(transformation(extent = {{58, 18}, {78, 38}})));

  Buildings.Controls.OBC.FDE.DOAS.HeatingCoil Heating(
    controllerTypeCoiHea=controllerTypeCoiHea,
    kCoiHea=kCoiHea,
    TiCoiHea=TiCoiHea,
    TdCoiHea=TdCoiHea)
    "This block commands the heating coil."
    annotation(Placement(transformation(extent = {{58, -18}, {78, 2}})));

  Buildings.Controls.OBC.FDE.DOAS.DehumMode DehumMod(
    dehumSet=dehumSet,
    timThrDehDis=timThrDehDis,
    timDelDehEna=timDelDehEna,
    timThrDehEna=timThrDehEna)
    "This block calculates when dehumidification mode is active."
    annotation(Placement(transformation(extent = {{-48, 8}, {-28, 28}})));

  ExhaustFan EFcon1(
    dPSetBui=dPSetBui,
    kExhFan=kExhFan,
    TiExhFan=TiExhFan,
    TdExhFan=TdExhFan,
    controllerTypeExhFan=controllerTypeExhFan)
    annotation (Placement(transformation(extent={{44,-124},{64,-104}})));
  EconomizerMode EconMod1(dTEcoThr=dTEcoThr)
    annotation (Placement(transformation(extent={{32,-42},{52,-22}})));
  SupplyTemperatureSetpoint TSupSetpt(
    TSupLowSet=TSupLowSet,
    TSupHigSet=TSupHigSet,
    THigZon=THigZon,
    TLowZon=TLowZon,
    TSupCooOff=TSupCooOff,
    TSupHeaOff=TSupHeaOff)
    annotation (Placement(transformation(extent={{0,-8},{20,12}})));
  CDL.Interfaces.RealInput TZonHeaSet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Zone heating setpoint"
    annotation (Placement(transformation(extent={{-140,78},{-100,118}}),
        iconTransformation(extent={{-140,116},{-100,156}})));
  CDL.Interfaces.RealInput TZonCooSet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Zone cooling setpoint"
    annotation (Placement(transformation(extent={{-140,110},{-100,150}}),
        iconTransformation(extent={{-140,96},{-100,136}})));
equation
  connect(SFcon.Occ,Occ)  annotation (
    Line(points={{-52,71},{-96,71},{-96,76},{-122,76}},          color = {255, 0, 255}));
  connect(SFcon.uDamMaxOpe, uDamMaxOpe) annotation (Line(points={{-52,67.4},{
          -98,67.4},{-98,50},{-122,50}}, color={0,0,127}));
  connect(SFcon.dPAirDucSta, dPAirDucSta) annotation (Line(points={{-52,56.8},{
          -94,56.8},{-94,-2},{-120,-2}}, color={0,0,127}));
  connect(SFcon.yFanSup, yFanSup) annotation (Line(points={{-28,69.2},{122,69.2},
          {122,74}}, color={255,0,255}));
  connect(SFcon.yFanSupSpe, yFanSupSpe) annotation (Line(points={{-28,59.6},{84,
          59.6},{84,48},{122,48}}, color={0,0,127}));
//connect(SFcon.supFanProof, DehumMod.supFanProof) annotation (
// Line(points={{-62,68},{-56,68},{-56,25.2},{-50.2,25.2}},color={255,0,255}));
//connect(SFcon.supFanProof, Cooling.supFanProof) annotation (
// Line(points={{-62,68},{-56,68},{-56,36.4},{55.8,36.4}},color={255,0,255}));
  connect(DehumMod.yDehMod, Cooling.uDeh) annotation (Line(points={{-25.8,18},{
          -22,18},{-22,27.4},{55.8,27.4}}, color={255,0,255}));
  connect(DehumMod.phiAirRet, phiAirRet) annotation (Line(points={{-50.2,18},{-92,
          18},{-92,-60},{-120,-60}}, color={0,0,127}));
  connect(Cooling.TAirSup, TAirSup) annotation (Line(points={{55.8,33.6},{-90,
          33.6},{-90,-86},{-120,-86}}, color={0,0,127}));
  connect(Cooling.TAirDis, TAirDisCoiCoo) annotation (Line(points={{55.8,24.8},
          {-20,24.8},{-20,-164},{-120,-164}}, color={0,0,127}));
  connect(Cooling.phiAirEneRecWhe, phiAirEneRecWhe) annotation (Line(points={{
          55.8,22.2},{-18,22.2},{-18,-190},{-120,-190}}, color={0,0,127}));
  connect(Cooling.TAirEneRecWhe, TAirSupEneWhe) annotation (Line(points={{55.8,
          19.6},{-16,19.6},{-16,-216},{-120,-216}}, color={0,0,127}));
  connect(Cooling.yCoiCoo, yCoiCoo) annotation (Line(points={{80.2,28},{92,28},
          {92,22},{122,22}}, color={0,0,127}));
//connect(SFcon.supFanProof, Heating.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-8},{55.8,-8}},color={255,0,255}));
  connect(TAirSup, Heating.TAirSup) annotation (Line(points={{-120,-86},{-90,-86},
          {-90,-13},{55.8,-13}}, color={0,0,127}));
  connect(Heating.yCoiHea, yCoiHea) annotation (Line(points={{80.2,-8},{90,-8},
          {90,-4},{122,-4}}, color={0,0,127}));
  connect(ERWcon.yEneRecWheEna, yEneRecWheEna)
    annotation (Line(points={{80.2,-54},{122,-54}}, color={255,0,255}));
  connect(ERWcon.yEneRecWheSpe, yEneRecWheSpe) annotation (Line(points={{80.2,
          -60},{92,-60},{92,-88},{122,-88}}, color={0,0,127}));
// connect(SFcon.supFanProof, ERWcon.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-46.2},{55.8,-46.2}},color={255,0,255}));
  connect(ERWcon.yBypDam, yBypDam) annotation (Line(points={{80.2,-48},{92,-48},
          {92,-30},{122,-30}}, color={255,0,255}));
//connect(SFcon.supFanProof, EconMod.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-25},{23.8,-25}}, color={255,0,255}));
  connect(TAirOut, ERWcon.TAirOut) annotation (Line(points={{-120,-138},{-22,-138},
          {-22,-55.8},{55.8,-55.8}}, color={0,0,127}));
  connect(TAirSupEneWhe, ERWcon.TAirSupEneWhe) annotation (Line(points={{-120,-216},
          {-16,-216},{-16,-58.8},{55.8,-58.8}}, color={0,0,127}));
// connect(SFcon.supFanProof, EFcon.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-90},{56,-90}}, color={255,0,255}));
  connect(uFanSupPro, SFcon.uFanSupPro) annotation (Line(points={{-120,24},{-96,
          24},{-96,60.4},{-52,60.4}}, color={255,0,255}));
  connect(uFanSupPro, DehumMod.uFanSupPro) annotation (Line(points={{-120,24},{
          -50.2,24},{-50.2,25.2}}, color={255,0,255}));
  connect(uFanSupPro, Cooling.uFanSupPro) annotation (Line(points={{-120,24},{-56,
          24},{-56,36.4},{55.8,36.4}}, color={255,0,255}));
  connect(uFanSupPro, Heating.uFanSupPro) annotation (Line(points={{-120,24},{
          24,24},{24,-8},{55.8,-8}}, color={255,0,255}));
  connect(uFanSupPro, ERWcon.uFanSupPro) annotation (Line(points={{-120,24},{44,
          24},{44,-46.2},{55.8,-46.2}}, color={255,0,255}));
  connect(ERWcon.TAirRet, TAirRet) annotation (Line(points={{55.8,-52.2},{-88,-52.2},
          {-88,-112},{-120,-112}}, color={0,0,127}));
  connect(EFcon1.yExhFanSta, yExhFanSta) annotation (Line(points={{66,-108},{94,
          -108},{94,-120},{120,-120}}, color={255,0,255}));
  connect(EFcon1.yExhFanSpe, yExhFanSpe) annotation (Line(points={{66,-120},{76,
          -120},{76,-132},{124,-132},{124,-168}}, color={0,0,127}));
  connect(uFanExhPro, EFcon1.uFanExhPro) annotation (Line(points={{-120,-242},{
          -46,-242},{-46,-230},{14,-230},{14,-114},{42,-114}}, color={255,0,255}));
  connect(dPAirStaBui, EFcon1.dPAirStaBui) annotation (Line(points={{-120,-270},
          {-40,-270},{-40,-272},{30,-272},{30,-120.4},{42,-120.4}}, color={0,0,
          127}));
  connect(DehumMod.yDehMod, TSupSetpt.uDehMod) annotation (Line(points={{-25.8,
          18},{-24,18},{-24,7},{-2,7}}, color={255,0,255}));
  connect(TAirHig, TSupSetpt.TAirHig) annotation (Line(points={{-120,-30},{-6,
          -30},{-6,0.4},{-2,0.4}},
                                 color={0,0,127}));
  connect(TSupSetpt.ySupHeaSet, Heating.TAirSupSetHea) annotation (Line(points=
          {{22,-1.8},{40,-1.8},{40,-3.2},{55.8,-3.2}}, color={0,0,127}));
  connect(TAirOut, EconMod1.TAirOut) annotation (Line(points={{-120,-138},{-78,
          -138},{-78,-140},{-34,-140},{-34,-32},{29.8,-32}}, color={0,0,127}));
  connect(TSupSetpt.ySupCooSet, EconMod1.TAirSupSetCoo) annotation (Line(points=
         {{22,6.8},{26,6.8},{26,-39},{29.8,-39}}, color={0,0,127}));
  connect(uFanSupPro, EconMod1.uFanSupPro) annotation (Line(points={{-120,24},{
          -46,24},{-46,-25},{29.8,-25}}, color={255,0,255}));
  connect(uFanSupPro, EFcon1.uFanSupPro) annotation (Line(points={{-120,24},{
          -40,24},{-40,-108},{42,-108}}, color={255,0,255}));
  connect(EconMod1.yEcoMod, ERWcon.uEcoMod) annotation (Line(points={{54.2,-32},
          {42,-32},{42,-50},{32,-50},{32,-49.2},{55.8,-49.2}}, color={255,0,255}));
  connect(TSupSetpt.ySupCooSet, Cooling.TAirSupSetCoo) annotation (Line(points=
          {{22,6.8},{32,6.8},{32,30.8},{55.8,30.8}}, color={0,0,127}));
  connect(TSupSetpt.ySupSet, ERWcon.TAirSupSetEneWhe) annotation (Line(points={
          {22,3.4},{18,3.4},{18,-61.8},{55.8,-61.8}}, color={0,0,127}));
  connect(TZonHeaSet, TSupSetpt.TZonHeaSet) annotation (Line(points={{-120,98},
          {-62,98},{-62,-2},{-2,-2}}, color={0,0,127}));
  connect(TZonCooSet, TSupSetpt.TZonCooSet) annotation (Line(points={{-120,130},
          {-62,130},{-62,5},{-2,5}}, color={0,0,127}));
  annotation(Dialog(tab="Controller parameters", group = "Supply fan parameters"),
    defaultComponentName = "DOAScon",
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -240}, {100, 100}}), graphics={  Text(textColor = {28, 108, 200}, extent = {{-90, 180}, {90, 76}}, textString = "%name", textStyle = {TextStyle.Bold}), Rectangle(lineColor = {179, 151, 128}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-100, 100}, {100, -240}}, radius = 10), Rectangle(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-92, 70}, {-50, 44}}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-76, 72}, {-30, 26}}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-58, 54}, {-48, 44}}), Rectangle(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{50, -168}, {94, -194}}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{32, -166}, {78, -212}}), Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{50, -184}, {60, -194}}), Line(points = {{52, -52}, {2, -52}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-10, 6}, {32, -120}}), Rectangle(lineColor = {170, 255, 255}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-4, 6}, {12, -120}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-24, 6}, {18, -120}}), Line(points = {{-4, -52}, {-54, -52}}), Rectangle(lineColor = {28, 108, 200}, fillColor = {85, 255, 255},
            fillPattern=
FillPattern.Solid, extent = {{-4, -168}, {4, -210}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern=
FillPattern.Solid, extent = {{-2, -204}, {2, -208}}), Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern=
FillPattern.Solid, extent = {{-2, -172}, {2, -176}}), Rectangle(lineColor = {162, 29, 33}, fillColor = {238, 46, 47},
            fillPattern=
FillPattern.Solid, extent = {{8, -168}, {16, -210}}), Ellipse(fillColor = {127, 0, 0},
            fillPattern=
FillPattern.Solid, extent = {{10, -204}, {14, -208}}), Ellipse(fillColor = {162, 29, 33},
            fillPattern=
FillPattern.Solid, extent = {{10, -172}, {14, -176}})}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -240}, {100, 100}})),
    Documentation(revisions = "<html>
    <ul>
    <li>
    January 31, 2023, by Cerrina Mouchref, Karthik Devaprasad:</br>
    Improved code as per library conventions. Fixed missing class referenvces due 
    to CDL package updates.
    </li>
<li>
September 25, 2020, by Henry Nickels:</br>
First implementation.</li>
</ul>
</html>", info = "<html>
<h4>BAS Controller</h4>
<p>This controller combines the functions of the
supply fan controller
(<code>SupplyFanController</code>), exhaust fan controller 
(<code>ExhaustFanController</code>), temperature set point block 
(<code>TSupSet</code>), economizer mode block 
(<code>EconMode</code>), dehumidification mode block 
(<code>DehumMode</code>), dewpoint calculation block 
(<code>Dewpoint</code>), cooling coil 
(<code>CoolingCoil</code>), heating coil 
(<code>HeatingCoil</code>), and total energy wheel controller 
(<code>EnergyWheel</code>). 
</p>
</html>"),
    experiment(
      StartTime=16848000,
      StopTime=17020800,
      __Dymola_Algorithm="Dassl"));
end DOAScontroller;
