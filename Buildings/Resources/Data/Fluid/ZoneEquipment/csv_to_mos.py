#!/usr/bin/env python3
import sys
sys.path.append('../../../Scripts/EnergyPlus')
import energyplus_csv_to_mos as e

if __name__ == '__main__':
  dat_fil = "FanCoil.dat"
  output_list =[
   "ZONE1FANCOIL:Fan Coil Heating Rate [W](TimeStep)",
   "ZONE1FANCOIL:Fan Coil Total Cooling Rate [W](TimeStep)",
   "ZONE1FANCOIL:Fan Coil Fan Electricity Rate [W](TimeStep)",
   "ZONE1FANCOILAIROUTLETNODE:System Node Temperature [C](TimeStep)",
   "ZONE1FANCOILAIRINLETNODE:System Node Temperature [C](TimeStep)",
   "ZONE1FANCOILAIRINLETNODE:System Node Mass Flow Rate [kg/s](TimeStep)",
   "ZONE1FANCOILCHWINLETNODE:System Node Temperature [C](TimeStep)",
   "ZONE1FANCOILCHWINLETNODE:System Node Mass Flow Rate [kg/s](TimeStep)",
   "ZONE1FANCOILHWINLETNODE:System Node Temperature [C](TimeStep)",
   "ZONE1FANCOILHWINLETNODE:System Node Mass Flow Rate [kg/s](TimeStep)"
  ]

  e.energyplus_csv_to_mos(
    output_list = output_list,
    dat_file_name=dat_fil,
    step_size=60,
    final_time=31536000)
