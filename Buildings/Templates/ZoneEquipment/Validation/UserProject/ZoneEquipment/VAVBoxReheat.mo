within Buildings.Templates.ZoneEquipment.Validation.UserProject.ZoneEquipment;
model VAVBoxReheat "Configuration of VAV terminal unit with reheat"
  extends Buildings.Templates.ZoneEquipment.VAVBoxReheat(
    redeclare replaceable Buildings.Templates.ZoneEquipment.Components.Controls.OpenLoop ctl
      "Open loop control");
  annotation (
    defaultComponentName="ter",
    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This is a configuration model with the same default options as
<a href=\"modelica://Buildings.Templates.ZoneEquipment.VAVBoxReheat\">
Buildings.Templates.ZoneEquipment.VAVBoxReheat</a>.
</p>
</html>"));
end VAVBoxReheat;
