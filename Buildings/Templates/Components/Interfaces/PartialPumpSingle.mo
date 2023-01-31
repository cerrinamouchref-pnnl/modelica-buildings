within Buildings.Templates.Components.Interfaces;
partial model PartialPumpSingle "Interface class for single pump"
  extends Buildings.Templates.Components.Interfaces.PartialPump;
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
    redeclare replaceable package Medium=Buildings.Media.Water,
    final m_flow_nominal(min=0)=dat.m_flow_nominal)
    annotation(__Linkage(enable=false));

  parameter Buildings.Templates.Components.Types.PumpSingleSpeedControl typCtrSpe
    "Type of pump speed control"
    annotation (Evaluate=true, Dialog(group="Configuration",
    enable=typ<>Buildings.Templates.Components.Types.Pump.None));

  parameter Buildings.Templates.Components.Data.PumpSingle dat(
    final typ=typ)
    "Design and operating parameters"
    annotation(__Linkage(enable=false));

  final parameter Modelica.Units.SI.PressureDifference dp_nominal(
    final min=0,
    displayUnit="Pa")=dat.dp_nominal
    "Pump head at design conditions"
    annotation (Dialog(group="Nominal condition",
      enable=typ<>Buildings.Templates.Components.Types.Pump.None));

  annotation (Documentation(info="<html>
<p>
This partial class provides a standard interface for 
single pump models.
</p>
</html>"));
end PartialPumpSingle;
