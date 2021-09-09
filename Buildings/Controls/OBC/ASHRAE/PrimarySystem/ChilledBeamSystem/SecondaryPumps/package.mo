within Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChilledBeamSystem;
package SecondaryPumps "Sequences for chilled water pump control"


annotation (
preferredView="info", Documentation(info="<html>
<p>
This package contains control sequences for chilled water pumps.
</p>
</html>"),
    Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100.0,-100.0},{100.0,100.0}},
          radius=25.0),
                 Ellipse(
        extent={{-66,66},{68,-68}},
        lineColor={0,0,0},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid), Polygon(
        points={{0,66},{0,-68},{68,0},{0,66}},
        lineColor={0,0,0},
        fillColor={255,0,0},
        fillPattern=FillPattern.Solid)}));
end SecondaryPumps;