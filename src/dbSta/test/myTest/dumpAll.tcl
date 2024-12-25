# dump all about data model
source "../helpers.tcl"
# SET UP
set LEF_FILE "../liberty1.lef"
set LIB_FILE "../liberty1.lib"
set VERILOG "../hier1.v"
set DESIGN "top"

set_debug_level ODB dumpPDK 1
set_debug_level ODB dumpDb 1
set_debug_level ODB dumpDbNetwork 1
set_debug_level ODB dumpVerilogNetwork 1 
set_debug_level ODB dumpRegion 1 

read_lef $LEF_FILE
read_liberty $LIB_FILE
read_verilog $VERILOG
link_design $DESIGN

ord::dump_db
ord::dump_pdk
ord::dump_db_verilog_network
ord::dump_db_network
ord::dump_region_group

exit