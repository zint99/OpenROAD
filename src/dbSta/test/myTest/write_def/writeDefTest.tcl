set curPath [pwd]

cd ../../

source "helpers.tcl"
read_lef liberty1.lef
read_liberty liberty1.lib
read_verilog hier1.v
link_design top

set def_file [make_result_file ${curPath}/read_verilog2.def]
write_def $def_file