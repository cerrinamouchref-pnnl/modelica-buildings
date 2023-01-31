within Buildings.Templates.Components.Interfaces;
model PartialPump "Base class for all pump models"
  parameter Buildings.Templates.Components.Types.Pump typ
    "Equipment type"
    annotation (Evaluate=true, Dialog(group="Configuration", enable=false));

  parameter Boolean have_valChe=true
    "Set to true to include a check valve in pump line"
    annotation (Evaluate=true, Dialog(group="Configuration",
    enable=typ<>Buildings.Templates.Components.Types.Pump.None),
    __Linkage(enable=false));

  parameter Boolean addPowerToMedium=false
    "Set to false to avoid any power (=heat and flow work) being added to medium (may give simpler equations)"
    annotation(__Linkage(enable=false));

  parameter Modelica.Units.SI.Time tau=1
    "Time constant of fluid volume for nominal flow, used if energy or mass balance is dynamic"
    annotation (Dialog(
      tab="Dynamics",
      group="Nominal condition",
      enable=energyDynamics<>Modelica.Fluid.Types.Dynamics.SteadyState),
      __Linkage(enable=false));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=
    Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Conservation equations"),
      __Linkage(enable=false));
  parameter Boolean allowFlowReversal = true
    "= false to simplify equations, assuming, but not enforcing, no flow reversal"
    annotation(Dialog(tab="Assumptions"), Evaluate=true, __Linkage(enable=false));

  parameter Integer text_rotation = 0
    "Text rotation angle in icon layer"
    annotation(Dialog(tab="Graphics", enable=false));
  parameter Boolean text_flip = false
    "True to flip text horizontally in icon layer"
    annotation(Dialog(tab="Graphics", enable=false));

  Buildings.Templates.Components.Interfaces.Bus bus
    "Control bus"
    annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={0,100}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,100})));

  annotation (Icon(graphics={
        Text(
          extent={{-149,-112},{151,-152}},
          textColor={0,0,255},
          textString="%name")}), Documentation(info="<html>
<p>
This partial class provides a standard interface for pump models.
</p>
</html>"));
end PartialPump;
