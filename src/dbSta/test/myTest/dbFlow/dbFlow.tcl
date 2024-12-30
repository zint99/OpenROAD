# dump all about data model
source "../../helpers.tcl"
# SET UP
set LEF_FILE "../../liberty1.lef"
set LIB_FILE "../../liberty1.lib"
set VERILOG "../../hier1.v"
set DESIGN "top"
set db_file "../dumpNetwork/flat.odb"

# set_debug_level ODB dumpDb 1
set_debug_level ODB dumpDbNetwork 1
set_debug_level ODB dumpVerilogNetwork 1 

puts "@@read_db"
set new_db [odb::dbDatabase_create]
odb::read_db $new_db $db_file

# ord::dump_db
ord::dump_db_verilog_network
ord::dump_db_network