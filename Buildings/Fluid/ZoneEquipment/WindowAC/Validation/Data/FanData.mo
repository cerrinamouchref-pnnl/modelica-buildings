within Buildings.Fluid.ZoneEquipment.WindowAC.Validation.Data;
record FanData "Fan data for the validation model"
  extends Buildings.Fluid.Movers.Data.Generic(
    speed_rpm_nominal=2900,
    use_powerCharacteristic=true,
    power(V_flow={0,0.056578,0.113156,0.169734,0.226312,0.28289,
        0.339468,0.396046,0.452624,0.509202,0.56578}, P={0,3.314,4.313,5.403,6.775,
          8.619,11.125,14.484,18.886,24.521,31.581}),
    pressure(V_flow={0.056578,0.113156,0.169734,0.226312,0.28289,
        0.339468,0.396046,0.452624,0.509202,0.56578}, dp={7500,1875,833.33,468.75,300,208.33,153.06,117.1875, 92.59, 75}),
    motorCooledByFluid=true);
  annotation (
defaultComponentPrefixes="parameter",
defaultComponentName="per",
Documentation(info="<html>
<p>
Fan performance data record based on EnergyPlus example file available in the 
Buildings library
(modelica-buildings/Buildings/Resources/Data/Fluid/ZoneEquipment/FanCoilAutoSize_ConstantFlowVariableFan.idf).
<br>
The volume flowrate values are derived by splitting the nominal flowrate from the 
EnergyPlus sizing report into ten datapoints. The power values are calculated for 
each flowrate datapoint with the power curve coefficients from EnergyPlus. 
</p>
</html>",   revisions="<html>
<ul>
<li>
September 06, 2022, by Karthik Devaprasad:
<br/>
Initial version
</li>
</ul>
</html>"));
end FanData;