"""
In:
    1. Source file
    2. Mapping rules file

Out:
    1. Revised file
"""

# here put the import lib
import argparse
import logging
import json

# logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)
formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("cellMapper.log", "w")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)


class MappingRuler:
    """
    A class to load and manage mapping rules between N45 and SKY130 standard cells.
    """

    def __init__(self, mapping_file):
        """
        Initialize the MappingRuler with a mapping file.

        :param mapping_file: Path to the JSON file containing mapping rules.
        """
        self.mapping_file = mapping_file
        self.mapping_dict = self._load_mapping_rules()

    def _load_mapping_rules(self):
        """
        Load mapping rules from the JSON file and build the mapping dictionary.

        :return: A dictionary containing the mapping rules.
        """
        try:
            with open(self.mapping_file, "r") as mapping_file:
                mapping_data = json.load(mapping_file)

            mapping_dict = {}
            for item in mapping_data:
                n45_cell = item["N45"]  # Source: 'AND2_X1'
                sky130_cell = item["SKY130"]  # Target: 'sky130_fd_sc_hd__and2_1'

                # Parse port mapping
                port_mapping = {}
                if item["Port"] and isinstance(item["Port"], str):
                    try:
                        port_mapping = {
                            src_port: dst_port
                            for mapping in item["Port"].split(", ")
                            for src_port, dst_port in [mapping.split("->")]
                        }
                    except Exception as e:
                        logger.warning(
                            f"Failed to parse port mapping for {n45_cell}: {e}. "
                            f"Using empty port mapping."
                        )

                # Store rules into mapping_dict
                mapping_dict[n45_cell] = {
                    "sky130_cell": sky130_cell,
                    "port_mapping": port_mapping,
                }
                logger.debug(
                    f"Loaded mapping rule for {n45_cell}: "
                    f"cell={sky130_cell}, "
                    f"ports={port_mapping if port_mapping else 'No port mapping'}"
                )

            logger.debug("Complete mapping dictionary: %s", mapping_dict)
            return mapping_dict

        except FileNotFoundError:
            logger.error(f"Mapping file not found: {self.mapping_file}")
            raise
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON format in file: {self.mapping_file}")
            raise
        except KeyError as e:
            logger.error(f"Missing required key in JSON file: {e}")
            raise

    def get_mapping_dict(self):
        """
        Get the loaded mapping dictionary.

        :return: The mapping dictionary.
        """
        return self.mapping_dict

    def get_cell_mapping(self, n45_cell):
        """
        Get the mapping rules for a specific N45 cell.

        :param n45_cell: The N45 cell name (e.g., 'AND2_X1').
        :return: A dictionary containing the SKY130 cell and port mapping, or None if not found.
        """
        return self.mapping_dict.get(n45_cell)


def main():
    # S1.Build the mapping rules
    try:
        mapping_ruler = MappingRuler(MAPPING_JSON)
        mapping_dict = mapping_ruler.get_mapping_dict()
        logger.info("Successfully loaded mapping rules")

    except Exception as e:
        logger.error(f"An error occurred while loading mapping rules: {e}")
        return

    # S2.Revise gate level netlist by the mapping rules
    try:
        # S2.1 Open and read the gate level netlist
        with open(VERILOG_FILE_PATH, "r") as f:
            netlist = f.read()

        # S2.2 Replace all cell types and corresponding ports
        for n45_cell, mapping in mapping_dict.items():
            sky130_cell = mapping["sky130_cell"]
            port_mapping = mapping["port_mapping"]

            # Find all instances of current cell type
            # e.g.: AND2_X1 _403_ (
            # Split by cell type and get all instances
            instance_list = []

            # Add spaces around cell type to ensure exact match
            # e.g.: " BUF_X1 " won't match "CLKBUF_X1"
            parts = netlist.split(f" {n45_cell} ")

            # Skip first part (before first cell_type)
            for part in parts[1:]:
                # Find the first '(' after cell_type
                paren_index = part.find("(")
                if paren_index != -1:
                    # Extract instance name between cell_type and (
                    instance_name = part[:paren_index].strip()
                    instance_list.append(instance_name)
                    logger.debug(f"Found instance {instance_name} of type {n45_cell}")

            # Replace cell type with spaces to ensure exact match
            logger.debug(f"Replace {n45_cell} to {sky130_cell}")
            netlist = netlist.replace(f" {n45_cell} ", f" {sky130_cell} ")

            # Replace port names only for instances of current cell type
            logger.debug(f"Port mapping for {n45_cell}: {port_mapping}")
            for instance in instance_list:
                # Find the instance definition block
                instance_start = netlist.find(f"{instance} (")
                if instance_start != -1:
                    # Find the end of this instance definition
                    instance_end = netlist.find(");", instance_start)
                    if instance_end != -1:
                        # Get the instance definition block
                        instance_block = netlist[instance_start : instance_end + 2]
                        updated_block = instance_block

                        # Create a list of (old_port, new_port) tuples sorted by port name length
                        # Process longer port names first to avoid partial matches
                        port_replacements = sorted(
                            [
                                (f".{src}", f".{dst}")
                                for src, dst in port_mapping.items()
                            ],
                            key=lambda x: len(x[0]),
                            reverse=True,
                        )

                        # Create temporary markers for each port to *avoid overwriting*
                        temp_markers = {}
                        for i, (old_port, _) in enumerate(port_replacements):
                            temp_marker = f"__TEMP_PORT_{i}__"
                            temp_markers[temp_marker] = old_port
                            updated_block = updated_block.replace(
                                f"{old_port}(", f"{temp_marker}("
                            )

                        # Replace temporary markers with final port names
                        for (_, new_port), temp_marker in zip(
                            port_replacements, temp_markers.keys()
                        ):
                            updated_block = updated_block.replace(
                                f"{temp_marker}(", f"{new_port}("
                            )
                            logger.debug(
                                f"Replace port {temp_markers[temp_marker][1:]} to {new_port[1:]} for instance {instance}"
                            )

                        # Update the netlist
                        netlist = netlist.replace(instance_block, updated_block)

        # S2.3 Save the revised netlist
        output_path = VERILOG_FILE_PATH.replace(".v", "_sky130.v")
        with open(output_path, "w") as f:
            f.write(netlist)

        logger.info(f"Successfully wrote revised netlist to {output_path}")

    except Exception as e:
        logger.error(f"An error occurred while processing netlist: {e}")
        return

    # S3.Check if the mapping is right
    try:
        # Read the original and mapped netlist files
        with open(VERILOG_FILE_PATH, "r") as f:
            original_netlist = f.read()
        with open(output_path, "r") as f:
            mapped_netlist = f.read()

        # Check cell type mapping
        for n45_cell, mapping in mapping_dict.items():
            sky130_cell = mapping["sky130_cell"]

            # Count occurrences in original netlist (with spaces to ensure exact match)
            orig_count = original_netlist.count(f" {n45_cell} ")
            # Count occurrences in mapped netlist
            mapped_count = mapped_netlist.count(f" {sky130_cell} ")

            if orig_count != mapped_count:
                logger.error(
                    f"Cell type mapping mismatch for {n45_cell}: "
                    f"Original count={orig_count}, Mapped count={mapped_count}"
                )
            else:
                logger.info(
                    f"Cell type mapping verified for {n45_cell} -> {sky130_cell}: "
                    f"Found {orig_count} instances"
                )

        # Check port mapping for each instance
        for n45_cell, mapping in mapping_dict.items():
            port_mapping = mapping["port_mapping"]

            # Find all instances of the original cell type
            instance_list = []
            parts = original_netlist.split(f" {n45_cell} ")
            for part in parts[1:]:
                paren_index = part.find("(")
                if paren_index != -1:
                    instance_name = part[:paren_index].strip()
                    if instance_name.startswith("_") and instance_name.endswith("_"):
                        instance_list.append(instance_name)

            # Check each instance's ports
            for instance in instance_list:
                # Find instance blocks in both netlists
                orig_start = original_netlist.find(f"{instance} (")
                mapped_start = mapped_netlist.find(f"{instance} (")

                if orig_start != -1 and mapped_start != -1:
                    orig_end = original_netlist.find(");", orig_start)
                    mapped_end = mapped_netlist.find(");", mapped_start)

                    if orig_end != -1 and mapped_end != -1:
                        orig_block = original_netlist[orig_start:orig_end]
                        mapped_block = mapped_netlist[mapped_start:mapped_end]

                        # Check each port mapping
                        for src_port, dst_port in port_mapping.items():
                            src_count = orig_block.count(f".{src_port}(")
                            dst_count = mapped_block.count(f".{dst_port}(")

                            if src_count != dst_count:
                                logger.error(
                                    f"Port mapping mismatch for instance {instance}: "
                                    f"Port {src_port}->{dst_port}, "
                                    f"Original count={src_count}, Mapped count={dst_count}"
                                )
                            else:
                                logger.debug(
                                    f"Port mapping verified for instance {instance}: "
                                    f"{src_port}->{dst_port}"
                                )

        logger.info("Mapping verification completed")

    except Exception as e:
        logger.error(f"An error occurred during mapping verification: {e}")
        return


if __name__ == "__main__":
    # argparse
    parser = argparse.ArgumentParser(
        description="Map the cells in the Nangate45/gcd netlist to the cells defined in SKY130."
    )

    parser.add_argument(
        "--mapping_file",
        help="The json file contain mapping rules",
        required=True,
        type=str,
    )
    parser.add_argument(
        "--netlist",
        help="The original netlist file which you wanna map",
        required=True,
        type=str,
    )

    args = parser.parse_args()
    VERILOG_FILE_PATH = args.netlist
    MAPPING_JSON = args.mapping_file

    main()
