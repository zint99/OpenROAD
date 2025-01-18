def parse_lib_file_with_ports(lib_path):
    cells = {}
    current_cell = None
    in_pin_section = False

    with open(lib_path, "r") as file:
        for line in file:
            stripped_line = line.strip()
            if stripped_line.startswith("pin ("):
                print(f"stripped_line {stripped_line}")
            # cell
            if stripped_line.startswith("cell ("):
                cell_name = stripped_line.split("(")[1].split(")")[0].strip()
                current_cell = cell_name
                cells[current_cell] = {"ports": []}
                print(f"current_cell {current_cell}")
            # ports
            elif current_cell and stripped_line.startswith("pin ("):
                print(f"cell {current_cell}, stripped_line {stripped_line}")
                port_name = stripped_line.split("(")[1].split(")")[0].strip()
                cells[current_cell]["ports"].append(port_name)
            # cell
            elif current_cell and stripped_line == "}":
                current_cell = None

    return cells


nangate_lib_path = "../Nangate45/Nangate45_typ.lib"
sky130_lib_path = "../sky130hd/sky130hd_tt.lib"

nangate_cells = parse_lib_file_with_ports(nangate_lib_path)
sky130_cells = parse_lib_file_with_ports(sky130_lib_path)

nangate_summary = (
    len(nangate_cells),
    sum(len(c["ports"]) for c in nangate_cells.values()),
)
sky130_summary = (
    len(sky130_cells),
    sum(len(c["ports"]) for c in sky130_cells.values()),
)

print(nangate_summary, sky130_summary)
