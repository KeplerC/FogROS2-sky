import argparse
import os
import yaml
import csv
import threading
import subprocess
import time

# Run aws configure, and sky check beforehand
class SkyOptimization():
    def __init__(self,yaml_file,debug):
        self.debug_ = debug
        if self.debug_:
            print("Initialize sky optimization")
        self.SECONDS_PER_MINUTE = 60
        self.yaml_file_ = yaml_file
        self.relevant_cpu_list_ = []
        self.benchmark_time_results_ = {}
        self.cpu_benchmark_yaml_list_ = ['benchmark_cpu_min.yaml','benchmark_cpu_mid.yaml','benchmark_cpu_max.yaml']
        self.benchmark_name_list_ = ['cpu-min','cpu-mid','cpu-max']
        self.benchmark_threads_ = []
        self.yaml_file_contents_ = None
        self.cpu_start_index_ = -1
        self.cpu_candiate_index_ = -1
        self.cpu_str_offset_ = 5
        self.seconds_per_step_index_ = 8
        self.findCpuValues()
        self.createBenchmarks()
        self.runSkyBenchmarks()
        print(self.benchmark_time_results_)
        
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
            cpu_replacement_substring = " " + str(self.relevant_cpu_list_[i])
            new_cpu_string = cpu_string.replace(cpu_substring,cpu_replacement_substring)
            self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
            f = open(self.cpu_benchmark_yaml_list_[i], "w")
            file_contents = ""
            for line in self.yaml_file_contents_:
                file_contents += line
            f.write(file_contents)
            f.close()

    def runSkyBenchmarks(self):
        if self.debug_:
            print("Run Sky Benchmarks")
        thread_list = []
        for i in range(len(self.relevant_cpu_list_)):
            t = threading.Thread(target=self.skyBenchmark,args=(self.cpu_benchmark_yaml_list_[i],self.benchmark_name_list_[i],self.relevant_cpu_list_[i]))
            t.start()
            thread_list.append(t)
        for i in range(len(thread_list)):
            thread_list[i].join()
        if self.debug_:
            print("Finished running sky benchmarks")
        # t1 = threading.Thread(target=self.skyBenchmark,args=(self.cpu_benchmark_yaml_list_[0],self.benchmark_name_list_[0],self.relevant_cpu_list_[0]))
        # t1.start()
        # t2 = threading.Thread(target=self.skyBenchmark,args=(self.cpu_benchmark_yaml_list_[1],self.benchmark_name_list_[1],self.relevant_cpu_list_[1]))
        # t2.start()
        # t1.join()
        # t2.join()
        #for i in range(len(self.cpu_benchmark_yaml_list_)):
    
    def skyBenchmark(self,yaml_file,benchmark_name,hardware_count):
        if self.debug_:
            print("Sky benchmark: " + yaml_file)
        benchmark_setup_command = 'bash benchmark.sh ' + yaml_file + ' ' + benchmark_name
        benchmark_setup=subprocess.Popen(benchmark_setup_command, shell=True, stdout=subprocess.PIPE, )
        #output = ""
        while True:
            line = benchmark_setup.stdout.readline()
            #if line == '' and process.poll() is not None:
            #    break
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                line_output = "Hardware count " + str(hardware_count) + ": " + line 
                print(line_output)
                # if(line == "To teardown the clusters:"):
                #     print("OVER")
                #     break
                
                #output += str(line.strip()) + "\n"
                #print(line.strip())  # Optional: Print the output in real-time
            else:
                break
        if self.debug_:
            print("Hardware count " + str(hardware_count) + ": " +"Cluster is setup")
        
        time_remaining = 5
        while(time_remaining > 0):
            print("Hardware count " + str(hardware_count) + ": " + "Benchmarking. " + str(time_remaining) + " minutes remaining")
            time.sleep(self.SECONDS_PER_MINUTE)
            time_remaining -= 1
        if self.debug_:
            print("Hardware count " + str(hardware_count) + ": " +"Check status")
        benchmark_log_command = 'sky bench show ' + benchmark_name
        benchmark_log=subprocess.Popen(benchmark_log_command, shell=True, stdout=subprocess.PIPE, )
        seconds_per_step_line = ""
        while True:
            line = benchmark_log.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                line_output = "Hardware count " + str(hardware_count) + ": " + line 
                print(line_output)
                if "sky-bench" in line:
                    seconds_per_step_line = line
            else:
                break
        seconds_per_step_array = seconds_per_step_line.split()
        seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_]
        self.benchmark_time_results_[hardware_count] = float(seconds_per_step)
        print("Hardware count " + str(hardware_count) + ": " +"Terminating cluster")
        benchmark_down_command = 'sky bench down ' + benchmark_name + '< benchmark_input.txt'
        benchmark_down = subprocess.Popen(benchmark_down_command, shell=True, stdout=subprocess.PIPE, )
        while True:
            line = benchmark_down.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                line_output = "Hardware count " + str(hardware_count) + ": " + line 
                print(line_output)
            else:
                break
        
        benchmark_delete_command = 'sky bench delete ' + benchmark_name + '< benchmark_input.txt'
        benchmark_delete = subprocess.Popen(benchmark_delete_command, shell=True, stdout=subprocess.PIPE, )
        while True:
            line = benchmark_delete.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                line_output = "Hardware count " + str(hardware_count) + ": " + line 
                print(line_output)
            else:
                break
        # benchmark_delete_command = 'sky bench delete ' + benchmark_name + '< benchmark_input.txt'
        # benchmark_delete = subprocess.Popen(benchmark_delete_command, shell=True, stdout=subprocess.PIPE, )
        # while True:
        #     line = benchmark_delete.stdout.readline()
        #     if line:
        #         line_byte = line.strip()
        #         line = line_byte.decode('utf-8')
        #         print(line)
        #     else:
        #         break
        # proc=subprocess.Popen('bash basic.sh', shell=True, stdout=subprocess.PIPE, )
        # output=proc.communicate()[0]
        # print("IN HERE 1")
        # print(output)
        # print("IN HERE 2")

        
        
        

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
