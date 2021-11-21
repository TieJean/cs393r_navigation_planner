import yaml
import os

# rostopic echo /localization | tee test.yaml
# time rosbag play --immediate test.bag --topics /localization

BAG_YAML_DIR = "../bag/GDC1/"

def parse_bag_yaml():
    ret = []
    for filename in os.listdir(BAG_YAML_DIR):
        if not filename.endswith(".yaml"):
            continue
        with open(os.path.join(BAG_YAML_DIR, filename), "r") as stream:
            try:
                dictionaries = yaml.safe_load_all(stream)
                for dictionary in dictionaries:
                    ret.append(dictionary)
            except yaml.YAMLError as exc:
                print(exc)
    return ret

# if __name__ == '__main__':
#     dictionaries = parse_bag_yaml()
#     print(len(dictionaries))
    # for dictionary in dictionaries:
    #     print(dictionary)