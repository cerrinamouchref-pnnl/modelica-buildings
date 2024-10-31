within Buildings.Controls.OBC.FDE.DOAS;
block Controller "DOAS controller built from DOAS blocks."

 parameter Real dTEneWheDewPoi(
  final unit = "K",
  final quantity = "TemperatureDifference") = 5
  "Value subtracted from ERW supply air dewpoint."
  annotation(Dialog(tab="Limits and Thresholds", group = "Energy recovery wheel parameters"));

  parameter CDL.Types.SimpleController controllerTypeCoiCooDeh=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller type for cooling air in dehumidification mode"
    annotation(Dialog(tab="Cooling coil", group = "Dehumidification mode controller parameters"));

  parameter Real kCoiCooDeh(
  final unit="1") = 1
    "Gain of conPIDDeh controller"
    annotation(Dialog(tab="Cooling coil", group = "Dehumidification mode controller parameters"));

  parameter Real TiCoiCooDeh(
  final unit="s") = 60
    "Time constant of integrator block for conPIDDeh controller"
    annotation(Dialog(tab="Cooling coil", group = "Dehumidification mode controller parameters"));

  parameter Real TdCoiCooDeh(
  final unit="s") = 0.1
    "Time constant of derivative block for conPIDDeh controller"
    annotation(Dialog(tab="Cooling coil", group = "Dehumidification mode controller parameters"));

  parameter CDL.Types.SimpleController controllerTypeCoiCooRegOpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller type for regular cooling coil operation mode"
    annotation(Dialog(tab="Cooling coil", group = "Dehumidification mode controller parameters"));

  parameter Real kCoiCooRegOpe(
  final unit="1") = 1
    "Gain of conPIDRegOpe controller"
    annotation(Dialog(tab="Cooling coil", group = "Regular operation controller parameters"));


  parameter Real TiCoiCooRegOpe(
  final unit="s")=60
    "Time constant of integrator block for conPIDRegOpe controller"
    annotation(Dialog(tab="Cooling coil", group = "Regular operation controller parameters"));


  parameter Real TdCoiCooRegOpe(
  final unit="s")=0.1
    "Time constant of derivative block for conPIDRegOpe controller"
    annotation(Dialog(tab="Cooling coil", group = "Regular operation controller parameters"));


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


  parameter CDL.Types.SimpleController controllerTypeHeaRec=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI controller type for energy recovery wheel heating recovery loop"
  annotation(Dialog(tab = "Energy recovery wheel",group="Heating recovery controller parameters"));


  parameter Real kHeaRec(
  final unit = "1") = 0.5
  "PID heating recovery loop gain value."
  annotation(Dialog(tab="Energy recovery wheel", group = "Heating recovery controller parameters"));


  parameter Real TiHeaRec(
  final unit = "s") = 60
  "PID  heating recovery loop time constant of integrator."
  annotation(Dialog(tab="Energy recovery wheel", group = "Heating recovery controller parameters"));

  parameter Real TdHeaRec(
  final unit = "s") = 0.1
  "PID heating recovery loop time constant of derivative block"
  annotation(Dialog(tab="Energy recovery wheel", group = "Heating recovery controller parameters"));

  parameter CDL.Types.SimpleController controllerTypeCooRec=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI controller type for energy recovery wheel cooling recovery loop"
  annotation(Dialog(tab="Energy recovery wheel", group = "Cooling recovery controller parameters"));

  parameter Real kCooRec(
  final unit = "1") = 0.5
  "PID cooling recovery loop gain value."
  annotation(Dialog(tab="Energy recovery wheel", group = "Cooling recovery controller parameters"));

  parameter Real TiCooRec(
  final unit = "s") = 60 "PID cooling recovery loop time constant of integrator."
  annotation(Dialog(tab="Energy recovery wheel", group = "Cooling recovery controller parameters"));

  parameter Real TdCooRec(
  final unit = "s") = 0.1
  "PID cooling recovery loop time constant of derivative block"
  annotation(Dialog(tab="Energy recovery wheel", group = "Cooling recovery controller parameters"));

   parameter Real dPSetBui(
  final unit = "Pa",
  final quantity = "PressureDifference") = 15
  "Building static pressure difference set point"
  annotation(Dialog(tab="Setpoints", group = "Exhuast fan parameters"));

  parameter Real kExhFan(
  final unit = "1") = 0.5
  "PID heating loop gain value."
  annotation(Dialog(tab="Exhaust fan", group = "Controller parameters"));

  parameter Real TiExhFan(
  final unit = "s") = 60
  "PID loop time constant of integrator."
  annotation(Dialog(tab="Exhaust fan", group = "Controller parameters"));

  parameter Real TdExhFan(
  final unit= "s") = 0.1 "Time constant of derivative block"
  annotation(Dialog(tab="Exhaust fan", group = "Controller parameters"));

  parameter CDL.Types.SimpleController controllerTypeExhFan=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PID controller type for exhaust fan"
    annotation(Dialog(tab="Exhaust fan", group = "Controller parameters"));

  parameter CDL.Types.SimpleController controllerTypeCoiHea=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
   "PID controller type for heating coil"
   annotation(Dialog(tab="Heating coil", group = "Controller parameters"));

  parameter Real kCoiHea(
   final unit= "1") = 0.5
  "Heating coil SAT PI gain value k."
  annotation(Dialog(tab="Heating coil", group = "Controller parameters"));

  parameter Real TiCoiHea(
   final unit= "s") = 60
  "Heating coil SAT PI time constant value Ti."
  annotation(Dialog(tab="Heating coil", group = "Controller parameters"));

  parameter Real TdCoiHea(
  final unit= "s") = 0.1 "Time constant of derivative block"
  annotation(Dialog(tab="Heating coil", group = "Controller parameters"));

     parameter Boolean is_vav = true
  "True: System has zone terminals with variable damper position. False: System has zone terminals with constant damper position."
  annotation(Dialog(tab="Supply Fan", group = "Differential setpoint controller parameters"));

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

  parameter Real kdPSetPoi(
   final unit= "1") = 0.5
  "Damper position setpoint PI gain value k."
  annotation(Dialog(tab="Supply Fan", group = "Differential setpoint controller parameters"));

  parameter Real TidPSetPoi(
   final unit= "s") = 60
  "Damper position setpoint PI time constant value Ti."
  annotation(Dialog(tab="Supply Fan", group = "Differential setpoint controller parameters"));

  parameter Real TddPSetPoi(
   final unit= "s") = 0.1 "Time constant of derivative block for conPIDdSetPoi"
   annotation(Dialog(tab="Supply Fan", group = "Differential setpoint controller parameters"));

  parameter CDL.Types.SimpleController controllerTypedPSetPoi=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
  "PI contoller type for differntial setpoint controller"
  annotation(Dialog(tab="Supply Fan", group = "Differential setpoint controller parameters"));

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
   annotation(Dialog(tab="Supply Fan", group = "Fan speed controller parameters "));

  parameter Real TdFanSpe(
   final unit= "s") = 60
   "Time constant of derivative block for conPIDFanSpe"
   annotation(Dialog(tab="Supply Fan", group = "Fan speed controller parameters"));

  parameter Real TiFanSpe(
   final unit= "s") = 0.000025
  "Fan speed set point SAT PI time constant value Ti."
  annotation(Dialog(tab="Supply Fan", group = "Fan speed controller parameters"));

  parameter CDL.Types.SimpleController controllerTypeFanSpe=Buildings.Controls.OBC.CDL.Types.SimpleController.PI
    "PI controller type for supply fan speed"
    annotation(Dialog(tab="Supply Fan", group = "Fan speed controller parameters"));

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
    annotation(Placement(transformation(extent={{-140,150},{-100,190}}),
      iconTransformation(extent={{-140,200},{-100,240}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uDamMaxOpe
    "Most open damper position from all terminal units served." annotation (
      Placement(transformation(extent={{-140,120},{-100,160}}),
        iconTransformation(extent={{-140,160},{-100,200}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanSupPro
    "True when supply fan is proven on." annotation (Placement(transformation(
          extent={{-140,90},{-100,130}}),
                                        iconTransformation(extent={{-140,120},{-100,
            160}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dPAirDucSta(final unit="Pa",
      final quantity="PressureDifference")
    "Down duct static pressure measurement." annotation (Placement(
        transformation(extent={{-140,60},{-100,100}}), iconTransformation(
          extent={{-140,80},{-100,120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirRet(
    final min=0,
    final max=100,
    final displayUnit="rh") "Return air relative humidity sensor." annotation (
      Placement(transformation(extent={{-140,0},{-100,40}}),
        iconTransformation(extent={{-140,40},{-100,80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSup(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Supply air temperature sensor."
    annotation (Placement(transformation(extent={{-140,-40},{-100,0}}),
        iconTransformation(extent={{-140,-40},{-100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirDisCoiCoo(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Cooling coil discharge air temperature sensor." annotation (Placement(
        transformation(extent={{-140,-130},{-100,-90}}),  iconTransformation(
          extent={{-140,-160},{-100,-120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput phiAirEneRecWhe(
    final min=0,
    final max=100,
    final displayUnit="rh") "ERW relative humidity sensor" annotation (
      Placement(transformation(extent={{-140,-160},{-100,-120}}),
        iconTransformation(extent={{-140,-200},{-100,-160}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirSupEneWhe(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "ERW dry bulb temperature sensor." annotation (Placement(transformation(
          extent={{-140,-190},{-100,-150}}), iconTransformation(extent={{-140,-240},
            {-100,-200}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirOut(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Outside air temperature"
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}}),
        iconTransformation(extent={{-140,-120},{-100,-80}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirRet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Return air temperature sensor."
    annotation (Placement(transformation(extent={{-140,-70},{-100,-30}}),
        iconTransformation(extent={{-140,-80},{-100,-40}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uFanExhPro
    "True when exhaust fan is proven on." annotation (Placement(transformation(
          extent={{-140,-220},{-100,-180}}), iconTransformation(extent={{-140,-280},
            {-100,-240}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dPAirStaBui(final unit="Pa",
      final quantity="PressureDifference") "Building static pressure"
    annotation (Placement(transformation(extent={{-140,-250},{-100,-210}}),
        iconTransformation(extent={{-140,-320},{-100,-280}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TAirHig(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature")
    "Highest space temperature reported from all terminal units." annotation (
      Placement(transformation(extent={{-140,30},{-100,70}}),
        iconTransformation(extent={{-140,0},{-100,40}})));

// ---outputs---
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yFanSup
    "Command supply fan to start when true." annotation (Placement(
        transformation(extent={{100,100},{140,140}}),
                                                    iconTransformation(extent={{100,140},
            {140,180}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yFanSupSpe
    "Supply fan speed command" annotation (Placement(transformation(extent={{100,70},
            {140,110}}),        iconTransformation(extent={{100,100},{140,140}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoiCoo
    "Cooling coil control signal" annotation (Placement(transformation(extent={{100,40},
            {140,80}}),         iconTransformation(extent={{100,60},{140,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCoiHea
    "Reheat coil valve command." annotation (Placement(transformation(extent={{100,10},
            {140,50}}),          iconTransformation(extent={{100,20},{140,60}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yBypDam
    "Bypass damper command; true when commanded full open." annotation (
      Placement(transformation(extent={{100,-20},{140,20}}),
        iconTransformation(extent={{100,-20},{140,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yEneRecWheEna
    "Command to start the energy recovery wheel." annotation (Placement(
        transformation(extent={{100,-50},{140,-10}}), iconTransformation(extent={{100,-60},
            {140,-20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yEneRecWheSpe
    "Energy recovery wheel speed command." annotation (Placement(transformation(
          extent={{100,-80},{140,-40}}),  iconTransformation(extent={{100,-100},
            {140,-60}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yExhFanSta
    "Command exhaust fan to start when true." annotation (Placement(
        transformation(extent={{100,-120},{140,-80}}),  iconTransformation(
          extent={{100,-140},{140,-100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput yExhFanSpe
    "Exhaust fan speed command" annotation (Placement(transformation(extent={{100,
            -150},{140,-110}}),     iconTransformation(extent={{100,-180},{140,-140}})));

  Buildings.Controls.OBC.FDE.DOAS.Subsequences.SupplyFanController SFcon(
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
    annotation(Placement(transformation(extent={{-10,140},{10,160}})));

  Buildings.Controls.OBC.FDE.DOAS.Subsequences.EnergyWheel ERWcon(
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
    annotation(Placement(transformation(extent={{60,-60},{80,-40}})));

  Buildings.Controls.OBC.FDE.DOAS.Subsequences.CoolingCoil Cooling(
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
     annotation(Placement(transformation(extent={{52,50},{72,70}})));

  Buildings.Controls.OBC.FDE.DOAS.Subsequences.HeatingCoil Heating(
    controllerTypeCoiHea=controllerTypeCoiHea,
    kCoiHea=kCoiHea,
    TiCoiHea=TiCoiHea,
    TdCoiHea=TdCoiHea)
    "This block commands the heating coil."
    annotation(Placement(transformation(extent={{50,0},{70,20}})));

  Buildings.Controls.OBC.FDE.DOAS.Subsequences.DehumidificationMode DehumMod(
    dehumSet=dehumSet,
    timThrDehDis=timThrDehDis,
    timDelDehEna=timDelDehEna,
    timThrDehEna=timThrDehEna)
    "This block calculates when dehumidification mode is active."
    annotation(Placement(transformation(extent={{-44,30},{-24,50}})));

  Subsequences.ExhaustFan EFcon1(
    dPSetBui=dPSetBui,
    kExhFan=kExhFan,
    TiExhFan=TiExhFan,
    TdExhFan=TdExhFan,
    controllerTypeExhFan=controllerTypeExhFan)
    annotation (Placement(transformation(extent={{30,-130},{50,-110}})));
  Subsequences.EconomizerMode EconMod1(dTEcoThr=dTEcoThr)
    annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
  Subsequences.SupplyTemperatureSetpoint TSupSetpt(
    TSupLowSet=TSupLowSet,
    TSupHigSet=TSupHigSet,
    THigZon=THigZon,
    TLowZon=TLowZon,
    TSupCooOff=TSupCooOff,
    TSupHeaOff=TSupHeaOff)
    annotation (Placement(transformation(extent={{-10,8},{10,28}})));
  CDL.Interfaces.RealInput TZonHeaSet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Zone heating setpoint"
    annotation (Placement(transformation(extent={{-140,180},{-100,220}}),
        iconTransformation(extent={{-140,240},{-100,280}})));
  CDL.Interfaces.RealInput TZonCooSet(
    final unit="K",
    final displayUnit="degC",
    final quantity="ThermodynamicTemperature") "Zone cooling setpoint"
    annotation (Placement(transformation(extent={{-140,210},{-100,250}}),
        iconTransformation(extent={{-140,280},{-100,320}})));
equation
  connect(SFcon.Occ,Occ)  annotation (
    Line(points={{-12,158},{-12,156},{-94,156},{-94,170},{-120,170}},
                                                                 color = {255, 0, 255}));
  connect(SFcon.uDamMaxOpe, uDamMaxOpe) annotation (Line(points={{-12,154},{-12,
          152},{-82,152},{-82,140},{-120,140}},
                                         color={0,0,127}));
  connect(SFcon.dPAirDucSta, dPAirDucSta) annotation (Line(points={{-12,144},{-90,
          144},{-90,80},{-120,80}},      color={0,0,127}));
  connect(SFcon.yFanSup, yFanSup) annotation (Line(points={{12,156},{90,156},{90,
          120},{120,120}},
                     color={255,0,255}));
  connect(SFcon.yFanSupSpe, yFanSupSpe) annotation (Line(points={{12,146},{80,146},
          {80,90},{120,90}},       color={0,0,127}));
//connect(SFcon.supFanProof, DehumMod.supFanProof) annotation (
// Line(points={{-62,68},{-56,68},{-56,25.2},{-50.2,25.2}},color={255,0,255}));
//connect(SFcon.supFanProof, Cooling.supFanProof) annotation (
// Line(points={{-62,68},{-56,68},{-56,36.4},{55.8,36.4}},color={255,0,255}));
  connect(DehumMod.yDehMod, Cooling.uDeh) annotation (Line(points={{-22,40},{36,
          40},{36,60},{50,60}},            color={255,0,255}));
  connect(DehumMod.phiAirRet, phiAirRet) annotation (Line(points={{-46,40},{-62,
          40},{-62,28},{-92,28},{-92,20},{-120,20}},
                                     color={0,0,127}));
  connect(Cooling.TAirSup, TAirSup) annotation (Line(points={{50,66},{-84,66},{-84,
          -20},{-120,-20}},            color={0,0,127}));
  connect(Cooling.TAirDis, TAirDisCoiCoo) annotation (Line(points={{50,58},{-86,
          58},{-86,-110},{-120,-110}},        color={0,0,127}));
  connect(Cooling.phiAirEneRecWhe, phiAirEneRecWhe) annotation (Line(points={{50,56},
          {-88,56},{-88,-140},{-120,-140}},              color={0,0,127}));
  connect(Cooling.TAirEneRecWhe, TAirSupEneWhe) annotation (Line(points={{50,53.8},
          {40,53.8},{40,36},{-14,36},{-14,-54},{-16,-54},{-16,-170},{-120,-170}},
                                                    color={0,0,127}));
  connect(Cooling.yCoiCoo, yCoiCoo) annotation (Line(points={{74,60},{120,60}},
                             color={0,0,127}));
//connect(SFcon.supFanProof, Heating.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-8},{55.8,-8}},color={255,0,255}));
  connect(TAirSup, Heating.TAirSup) annotation (Line(points={{-120,-20},{16,-20},
          {16,4},{48,4}},        color={0,0,127}));
  connect(Heating.yCoiHea, yCoiHea) annotation (Line(points={{72,10},{88,10},{88,
          30},{120,30}},     color={0,0,127}));
  connect(ERWcon.yEneRecWheEna, yEneRecWheEna)
    annotation (Line(points={{82,-50},{96,-50},{96,-30},{120,-30}},
                                                    color={255,0,255}));
  connect(ERWcon.yEneRecWheSpe, yEneRecWheSpe) annotation (Line(points={{82,-56},
          {92,-56},{92,-60},{120,-60}},      color={0,0,127}));
// connect(SFcon.supFanProof, ERWcon.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-46.2},{55.8,-46.2}},color={255,0,255}));
  connect(ERWcon.yBypDam, yBypDam) annotation (Line(points={{82,-44},{90,-44},{90,
          0},{120,0}},         color={255,0,255}));
//connect(SFcon.supFanProof, EconMod.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-25},{23.8,-25}}, color={255,0,255}));
  connect(TAirOut, ERWcon.TAirOut) annotation (Line(points={{-120,-80},{-22,-80},
          {-22,-52},{58,-52}},       color={0,0,127}));
  connect(TAirSupEneWhe, ERWcon.TAirSupEneWhe) annotation (Line(points={{-120,-170},
          {-16,-170},{-16,-54},{58,-54}},       color={0,0,127}));
// connect(SFcon.supFanProof, EFcon.supFanProof) annotation (
//Line(points={{-62,68},{-56,68},{-56,-90},{56,-90}}, color={255,0,255}));
  connect(uFanSupPro, SFcon.uFanSupPro) annotation (Line(points={{-120,110},{-96,
          110},{-96,140},{-24,140},{-24,148},{-12,148}},
                                      color={255,0,255}));
  connect(uFanSupPro, DehumMod.uFanSupPro) annotation (Line(points={{-120,110},{
          -96,110},{-96,140},{-58,140},{-58,46},{-46,46}},
                                   color={255,0,255}));
  connect(uFanSupPro, Cooling.uFanSupPro) annotation (Line(points={{-120,110},{-96,
          110},{-96,60},{-58,60},{-58,112},{48,112},{48,68},{50,68}},
                                       color={255,0,255}));
  connect(uFanSupPro, Heating.uFanSupPro) annotation (Line(points={{-120,110},{-96,
          110},{-96,60},{-58,60},{-58,112},{48,112},{48,22},{42,22},{42,10},{48,
          10}},                      color={255,0,255}));
  connect(uFanSupPro, ERWcon.uFanSupPro) annotation (Line(points={{-120,110},{24,
          110},{24,-42},{58,-42}},      color={255,0,255}));
  connect(ERWcon.TAirRet, TAirRet) annotation (Line(points={{58,-48},{-88,-48},{
          -88,-50},{-120,-50}},    color={0,0,127}));
  connect(EFcon1.yExhFanSta, yExhFanSta) annotation (Line(points={{52,-114},{94,
          -114},{94,-100},{120,-100}}, color={255,0,255}));
  connect(EFcon1.yExhFanSpe, yExhFanSpe) annotation (Line(points={{52,-126},{80,
          -126},{80,-130},{120,-130}},            color={0,0,127}));
  connect(uFanExhPro, EFcon1.uFanExhPro) annotation (Line(points={{-120,-200},{22,
          -200},{22,-120},{28,-120}},                          color={255,0,255}));
  connect(dPAirStaBui, EFcon1.dPAirStaBui) annotation (Line(points={{-120,-230},
          {-90,-230},{-90,-172},{20,-172},{20,-126.4},{28,-126.4}}, color={0,0,
          127}));
  connect(DehumMod.yDehMod, TSupSetpt.uDehMod) annotation (Line(points={{-22,40},
          {-12,40},{-12,24}},           color={255,0,255}));
  connect(TAirHig, TSupSetpt.TAirHig) annotation (Line(points={{-120,50},{-78,50},
          {-78,16},{-12,16}},    color={0,0,127}));
  connect(TSupSetpt.ySupHeaSet, Heating.TAirSupSetHea) annotation (Line(points={{12,14},
          {29.9,14},{29.9,14},{48,14}},                color={0,0,127}));
  connect(TAirOut, EconMod1.TAirOut) annotation (Line(points={{-120,-80},{-22,-80},
          {-22,-30},{27.8,-30}},                             color={0,0,127}));
  connect(TSupSetpt.ySupCooSet, EconMod1.TAirSupSetCoo) annotation (Line(points={{12,22},
          {22,22},{22,-16},{20,-16},{20,-37},{27.8,-37}},
                                                  color={0,0,127}));
  connect(uFanSupPro, EconMod1.uFanSupPro) annotation (Line(points={{-120,110},{
          -46,110},{-46,-23},{27.8,-23}},color={255,0,255}));
  connect(uFanSupPro, EFcon1.uFanSupPro) annotation (Line(points={{-120,110},{-96,
          110},{-96,60},{-58,60},{-58,-114},{28,-114}},
                                         color={255,0,255}));
  connect(EconMod1.yEcoMod, ERWcon.uEcoMod) annotation (Line(points={{52.2,-30},
          {54,-30},{54,-44},{58,-44}},                         color={255,0,255}));
  connect(TSupSetpt.ySupCooSet, Cooling.TAirSupSetCoo) annotation (Line(points={{12,22},
          {28,22},{28,64},{50,64}},                  color={0,0,127}));
  connect(TSupSetpt.ySupSet, ERWcon.TAirSupSetEneWhe) annotation (Line(points={{12,18.2},
          {12,18},{20,18},{20,-16},{22,-16},{22,-38},{20,-38},{20,-58},{58,-58}},
                                                      color={0,0,127}));
  connect(TZonHeaSet, TSupSetpt.TZonHeaSet) annotation (Line(points={{-120,200},
          {-98,200},{-98,12},{-12,12}},
                                     color={0,0,127}));
  connect(TZonCooSet, TSupSetpt.TZonCooSet) annotation (Line(points={{-120,230},
          {-88,230},{-88,54},{-80,54},{-80,20},{-12,20}},
                                     color={0,0,127}));
  annotation (
    defaultComponentName = "DOAScon",
    Icon(coordinateSystem(preserveAspectRatio = false, extent={{-100,-320},{100,
            320}}),                                                                          graphics={
        Rectangle(
          extent={{-100,320},{100,-320}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),                                                                Text(textColor = {28, 108, 200}, extent={{-100,
              400},{100,320}},                                                                                                                                             textString = "%name", textStyle = {TextStyle.Bold}),
                                                                      Rectangle(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-92,130},{-50,104}}),    Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-76,132},{-30,86}}),     Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-58,114},{-48,104}}),    Rectangle(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{50,-108},{94,-134}}),      Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{32,-106},{78,-152}}),      Ellipse(lineColor = {162, 29, 33}, fillColor = {255, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{50,-124},{60,-134}}),      Line(points={{52,8},{2,8}}),          Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-10,66},{32,-60}}),      Rectangle(lineColor = {170, 255, 255}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-4,66},{12,-60}}),      Ellipse(lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-24,66},{18,-60}}),      Line(points={{-4,8},{-54,8}}),          Rectangle(lineColor = {28, 108, 200}, fillColor = {85, 255, 255},
            fillPattern=
FillPattern.Solid, extent={{-4,-108},{4,-150}}),      Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern=
FillPattern.Solid, extent={{-2,-144},{2,-148}}),      Ellipse(lineColor = {28, 108, 200}, fillColor = {0, 0, 255},
            fillPattern=
FillPattern.Solid, extent={{-2,-112},{2,-116}}),      Rectangle(lineColor = {162, 29, 33}, fillColor = {238, 46, 47},
            fillPattern=
FillPattern.Solid, extent={{8,-108},{16,-150}}),      Ellipse(fillColor = {127, 0, 0},
            fillPattern=
FillPattern.Solid, extent={{10,-144},{14,-148}}),      Ellipse(fillColor = {162, 29, 33},
            fillPattern=
FillPattern.Solid, extent={{10,-112},{14,-116}})}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent={{-100,-240},{100,
            240}})),
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
end Controller;
