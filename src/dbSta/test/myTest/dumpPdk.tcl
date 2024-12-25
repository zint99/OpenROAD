# 
source "helpers.tcl"

set_debug_level ODB dumpPDK 1

read_lef liberty1.lef
read_liberty liberty1.lib
read_verilog hier1.v
link_design top
ord::dump_pdk

