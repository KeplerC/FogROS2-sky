import argparse
import os
import yaml
import csv

class SkyOptimization():
    def __init__(self,yaml_file,debug):
        self.debug_ = debug
        if self.debug_:
            print("Initialize sky optimization")
        self.yaml_file_ = yaml_file
        self.relevant_cpu_list_ = []
        self.cpu_benchmark_yaml_list_ = ['benchmark_cpu_min.yaml','benchmark_cpu_mid.yaml','benchmark_cpu_max.yaml']
        self.yaml_file_contents_ = None
        self.cpu_start_index_ = -1
        self.cpu_candiate_index_ = -1
        self.cpu_str_offset_ = 5
        self.findCpuValues()
        self.createBenchmarks()
        #min_yaml_contents = yaml_file_contents
        # f = open("demofile3.txt", "w")
        # file_contents = ""
        # for line in yaml_file_contents:
        #     file_contents += line
        # f.write(file_contents)
        # f.close()

        #parsed through_yaml = False
        #print(yaml_file_contents[cpu_start_index])
        #os.system("cat " + yaml_file_file)

    def findCpuValues(self):
        if self.debug_:
            print("Finding relevant CPU values for fitting")
        cpu_start_string = '    candidates:\n'
        cwd = os.getcwd()
        self.yaml_file_ = cwd + '/' + self.yaml_file_
        self.yaml_file_contents_ = None
        with open(self.yaml_file_, 'r') as file:
            self.yaml_file_contents_ = file.readlines()
        cpu_index = self.yaml_file_contents_.index(cpu_start_string) + 1
        self.cpu_start_index_ = cpu_index
        cpu_list = []
        yaml_parsed = False
        self.cpu_str_offset_ = 5
        while(not yaml_parsed):
            try:
                cpu_string = self.yaml_file_contents_[cpu_index]
                cpu_substring = cpu_string[cpu_string.find("cpus:")+self.cpu_str_offset_:cpu_string.find("}")]
                cpu_num = int(cpu_substring)
                cpu_list.append(cpu_num)
                cpu_index = cpu_index + 1
            except:
                yaml_parsed = True
        cpu_list.sort()
        cpu_min = cpu_list[0]
        cpu_max = cpu_list[len(cpu_list) - 1]
        cpu_mid = cpu_list[int(len(cpu_list) / 2)]
        self.relevant_cpu_list_.append(cpu_min)
        self.relevant_cpu_list_.append(cpu_mid)
        self.relevant_cpu_list_.append(cpu_max)
        if self.debug_:
            print("CPU Min: " + str(cpu_min))
            print("CPU Mid: " + str(cpu_mid))
            print("CPU Max: " + str(cpu_max))

    def createBenchmarks(self):
        if self.debug_:
            print("Creating benchmarks")
        for i in range(len(self.cpu_benchmark_yaml_list_)):
            cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
            cpu_substring = cpu_string[cpu_string.find("cpus:")+self.cpu_str_offset_:cpu_string.find("}")]
            new_cpu_string = cpu_string.replace(cpu_substring,str(self.relevant_cpu_list_[i]))
            self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
            f = open(self.cpu_benchmark_yaml_list_[i], "w")
            file_contents = ""
            for line in self.yaml_file_contents_:
                file_contents += line
            f.write(file_contents)
            f.close()
        

def main():
    print("Hello World!")
    parser = argparse.ArgumentParser()
    parser.add_argument('-y', '--yaml_file', required=True)
    parser.add_argument('-d','--debug', action='store_true')
    parser.add_argument
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
