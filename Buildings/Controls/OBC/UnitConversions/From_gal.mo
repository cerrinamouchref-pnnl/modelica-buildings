within Buildings.Controls.OBC.UnitConversions;
block From_gal "Block that converts volume from gallon to cubic meter"

  Buildings.Controls.OBC.CDL.Interfaces.RealInput u(
    final quantity = "Volume")
    "Volume in gallon"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealOutput y(
    final unit = "m3",
    final quantity = "Volume")
    "Volume in cubic meter"
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));

protected
  constant Real k = 0.003785412 "Multiplier";

  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter conv(
    final k = k) "Unit converter"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

equation
  connect(u, conv.u)
    annotation (Line(points={{-120,0},{-12,0}},color={0,0,127}));
  connect(conv.y, y)
    annotation (Line(points={{12,0},{120,0}},color={0,0,127}));
  annotation (
      defaultComponentName = "from_gal",
    Icon(graphics={
        Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{20,58}}, color={28,108,200}),
        Text(
          textColor={0,0,255},
          extent={{-150,110},{150,150}},
          textString="%name"),
        Text(
          extent={{-80,50},{0,10}},
          textColor={0,0,127},
          textString="gal"),
        Text(
          extent={{10,-70},{90,-30}},
          textColor={0,0,127},
          textString="m3"),
        Polygon(
        points={{90,0},{30,20},{30,-20},{90,0}},
        lineColor={191,0,0},
        fillColor={191,0,0},
        fillPattern=FillPattern.Solid),
        Line(points={{-90,0},{30,0}}, color={191,0,0})}),
        Documentation(info="<html>
<p>
Converts volume given in gallon [gal] to cubic meter [m3].
</p>
</html>", revisions="<html>
<ul>
<li>
November 29, 2021, by Michael Wetter:<br/>
Regenerated files with <code>Text</code> annotation using now the <code>textColor</code> attribute
rather than the deprecated <code>lineColor</code> attribute.
</li>
<li>
July 05, 2018, by Milica Grahovac:<br/>
Generated with <code>Buildings/Resources/src/Controls/OBC/UnitConversions/unit_converters.py</code>.<br/>
First implementation.
</li>
</ul>
</html>"));
end From_gal;
