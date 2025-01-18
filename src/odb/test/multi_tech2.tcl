# set_debug_level ODB "read_def" 1
set_debug_level ODB "gpdk" 1
set_debug_level ODB "dumpPDK" 1

# Nangate45
read_liberty Nangate45/Nangate45_typ.lib
read_lef -tech_name ng45_tech -tech -lib Nangate45/Nangate45_tech.lef
read_lef -tech_name ng45_tech -lib Nangate45/Nangate45_stdcell.lef

# SKY130
read_liberty sky130hd/sky130hd_tt.lib
read_lef -tech_name sky_tech -tech -lib sky130hd/sky130hd.tlef
read_lef -tech_name sky_tech -lib sky130hd/sky130hd_std_cell.lef

read_def -tech ng45_tech data/gcd/floorplan2.def

ord::dump_pdk
ord::map_to_specific_pdk