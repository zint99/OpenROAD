# test for macros connected to different nets
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_macros/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VDD -pin_pattern {^VDDPE$}
add_global_connection -net VDD -pin_pattern {^VDDCE$}
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
add_global_connection -net VSS -pin_pattern {^VSSE$}

add_global_connection -net VDD1 \
  -inst_pattern "dcache.data.data_arrays_0.data_arrays_0_ext.mem" \
  -pin_pattern {^VDD$} -power
add_global_connection -net VDD2 \
  -inst_pattern "frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem" \
  -pin_pattern {^VDD$} -power

set_voltage_domain -power VDD -ground VSS -secondary_power "VDD1 VDD2"

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1
add_pdn_ring -grid "Core" -layers {metal7 metal8} -widths 2.0 -spacings 2.0 -core_offsets 2.0
add_pdn_stripe -layer metal4 -width 0.48 -spacing 4.0 -pitch 49.0 -offset 2.0
add_pdn_stripe -layer metal7 -width 1.4 -pitch 40.0 -offset 2.0 -extend_to_core_ring

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal4 metal7}
add_pdn_connect -layers {metal7 metal8}

define_pdn_grid -macro -name "sram1" \
  -instances "dcache.data.data_arrays_0.data_arrays_0_ext.mem"
add_pdn_stripe -layer metal5 -width 0.93 -pitch 10.0 -offset 2.0 -extend_to_core_ring
add_pdn_stripe -layer metal6 -width 0.93 -pitch 10.0 -offset 2.0 -extend_to_core_ring

add_pdn_connect -layers {metal4 metal5}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal6 metal7}

define_pdn_grid -macro -name "sram2" \
  -instances "frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem"
add_pdn_stripe -layer metal5 -width 0.93 -pitch 10.0 -offset 2.0 -extend_to_core_ring
add_pdn_stripe -layer metal6 -width 0.93 -pitch 10.0 -offset 2.0 -extend_to_core_ring

add_pdn_connect -layers {metal4 metal5}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal6 metal7}

pdngen

set def_file [make_result_file macros_different_nets.def]
write_def $def_file
diff_files macros_different_nets.defok $def_file
