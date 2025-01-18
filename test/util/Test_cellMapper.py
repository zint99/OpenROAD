from cellMapper import MappingRuler


def main():
    MAPPING_JSON = "mappingRules.json"
    try:
        mapping_ruler = MappingRuler(MAPPING_JSON)
        mapping_dict = mapping_ruler.get_mapping_dict()
        print(f"Successfully loaded mapping rules\nmapping_dict: {mapping_dict}")

    except Exception as e:
        print(f"An error occurred while loading mapping rules: {e}")
        return


if __name__ == "__main__":
    main()
# end main
