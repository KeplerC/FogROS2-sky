import argparse
import os
import yaml

class SkyOptimization():
    def __init__(self,yaml_file):
        print("Initialize sky optimization")
        cpu_start_string = '    candidates:\n'
        cwd = os.getcwd()
        yaml_file = cwd + '/' + yaml_file
        yaml_file_contents = None
        with open(yaml_file, 'r') as file:
            yaml_file_contents = file.readlines()
        cpu_index = yaml_file_contents.index(cpu_start_string) + 1
        cpu_list = []
        yaml_parsed = False
        while(not yaml_parsed):
            try:
                cpu_string = yaml_file_contents[cpu_index]
                print(cpu_string)
                cpu_substring = cpu_string[cpu_string.find("cpus:")+5:cpu_string.find("}")]
                cpu_num = int(cpu_substring)
                cpu_list.append(cpu_num)
                cpu_index = cpu_index + 1
            except:
                print("ITS OVER")
                yaml_parsed = True
        cpu_list.sort()
        cpu_min = cpu_list[0]
        cpu_max = cpu_list[len(cpu_list) - 1]
        cpu_mid = cpu_list[int(len(cpu_list) / 2)]
        print(cpu_min)
        print(cpu_mid)
        print(cpu_max)
        #parsed through_yaml = False
        #print(yaml_file_contents[cpu_start_index])
        #os.system("cat " + yaml_file_file)

def main():
    print("Hello World!")
    parser = argparse.ArgumentParser()
    parser.add_argument('-y', '--yaml_file', required=True)
    args = parser.parse_args()
    sky_optimization = SkyOptimization(**vars(args))
    # Start with reading the benchmark.yaml_file file
    # Get the max, min, and middle CPU values
    # Run benchmarking these 3 values
    # Find linear function for seconds per step
    # Find these exact instances in the ~/.sky/catalogs/v5/aws/vms.csv
    # Figure out cost per CPU by looking at those instances (we could just pull from terminal, but that may not scale well)
    # Setup optimization

if __name__ == "__main__":
    main()
