# report_timing_histogram
source "helpers.tcl"
read_lef example1.lef
read_def example1.def
read_liberty example1_slow.lib

create_clock -name clk -period 10 {clk1 clk2 clk3}
set_input_delay -clock clk 0 {in1 in2}
set_output_delay -clock clk 0 out
report_timing_histogram -num_bins 5

puts "Setup"
report_timing_histogram -setup

puts "Hold"
report_timing_histogram -hold
