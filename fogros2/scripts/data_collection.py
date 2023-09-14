import argparse
import os
import yaml
import csv
import threading
import subprocess
import time
import numpy as np
from enum import Enum
from scipy.optimize import NonlinearConstraint, minimize, Bounds
import math
from itertools import product
from scipy.optimize import curve_fit
from models import Model, GPUModel
import queue


class ModelType(Enum):
    LINEAR = 1
    POWER = 2
    EXPONENTIAL = 3
    LOGARITHMIC = 4
    QUADRATIC = 5
    HYPERBOLIC = 6


class OptimizationFunctionType(Enum):
    TIME = 1
    COST = 2


# Run aws configure, and sky check beforehand
class SkyOptimization:
    def __init__(self, yaml_file, gpu_yaml_file,steps, max_cost, max_time, debug,gpu_type,count):
        self.debug_ = debug
        self.steps_ = steps
        self.max_cost_ = max_cost
        self.max_time_ = max_time
        self.gpu_type_ = gpu_type
        self.count_ = count
        if self.debug_:
            print("Initialize sky optimization")
        if self.steps_ <= 0:
            print("Please enter a positive step number")
            print("Program now exiting")
            exit()
        if self.max_cost_ <= 0:
            print("Please enter a positive cost")
            print("Program now exiting")
            exit()
        if self.max_time_ <= 0:
            print("Please enter a positive time")
            print("Program now exiting")
            exit()
        self.SECONDS_PER_MINUTE = 60
        self.MINUTES_PER_HOUR = 60
        self.SECONDS_PER_HOUR = self.SECONDS_PER_MINUTE * self.MINUTES_PER_HOUR
        self.ZERO_EPSILON = 1e-10
        self.yaml_file_ = yaml_file
        self.gpu_yaml_file_ = gpu_yaml_file
        self.is_slam_ = "slam" in self.yaml_file_
        self.is_motion_planning_ = "mp" in self.yaml_file_
        self.optimization_function_type_length_ = 25
        self.max_cpu_ = 0
        self.max_memory_ = 0
        self.relevant_cpu_list_ = []
        self.relevant_hardware_list_ = []
        self.benchmark_time_results_ = []
        self.benchmark_yaml_list_ = []
        self.cpu_benchmark_yaml_list_ = [
            "benchmark_cpu_min",
            "benchmark_cpu_mid",
            "benchmark_cpu_max",
        ]
        self.benchmark_range_ = 3
        self.benchmark_name_list_ = ["cpu-min", "cpu-mid", "cpu-max"]
        self.benchmark_threads_ = []
        self.benchmark_cost_substring_1_ = "CLUSTER"
        self.benchmark_cost_substring_2_ = "CLOUD"
        self.benchmark_time_substring_1_ = "CLUSTER"
        self.benchmark_time_substring_2_ = "RESOURCES"
        self.min_gpu_substring_1_ = "CLOUD"
        self.min_gpu_substring_2_ = "ACCELERATORS"
        self.mem_to_cpu_ratio_for_gpu_ = None
        self.num_steps_and_duration_ = []
        self.average_setup_time_ = 0
        self.got_cost_info_ = False
        self.benchmark_cost_substring_cpu_index_ = 8
        self.benchmark_cost_substring_cost_index_ = 11
        self.benchmark_cost_results_ = []
        self.yaml_file_contents_ = None
        self.cpu_start_index_ = -1
        self.cpu_candiate_index_ = -1
        self.cpu_str_offset_ = 5
        self.seconds_per_step_index_ = 8
        self.num_steps_index_ = 7
        self.duration_min_index_ = 4
        self.duration_sec_index_ = 5
        if(self.gpu_type_ != ""):
            self.seconds_per_step_index_ += 2
            self.duration_min_index_ += 2
            self.duration_sec_index_ += 2
        self.time_constraint_coefficients_ = None
        self.time_constraint_model_ = None
        self.solutions_in_constraints_ = []
        self.solutions_outside_constraints_ = []
        self.data_file_ = "CLUSTER                RESOURCES                        STATUS   DURATION  SPENT($)  #STEPS  SEC/STEP  $/STEP    EST(hr)  EST($)"
        self.data_file_name_ = "benchmark_data_" + self.gpu_type_ + ".txt"
        self.result_queue_ = []
        self.possible_cpus_ = [2,4,8,16,32,64]
        self.optimal_cost_instance_ = "Sample"
        self.optimal_time_instance_ = "Sample"
        self.t4_map_ = {
            (4,16): "g4dn.xlarge",
            (8,32): "g4dn.2xlarge",
            (16,64): "g4dn.4xlarge"
        }
        self.no_gpu_map_ = {
            (2,4): "c6i.large",
            (2,8): "m6i.large",
            (2,16): "r6i.large",
            (4,8): "c6i.xlarge",
            (4,16): "m6i.xlarge",
            (4,32): "r6i.xlarge",
            (8,16): "c6i.2xlarge",
            (8,32): "m6i.2xlarge",
            (8,64): "r6i.2xlarge",
            (16,32): "c6i.4xlarge",
            (16,64): "m6i.4xlarge",
            (16,128): "r6i.4xlarge",
            (32,64): "c6i.8xlarge",
            (32,128): "m6i.8xlarge",
            (32,256): "r6i.8xlarge",
            (64,128): "c6i.16xlarge",
            (64,256): "m6i.16xlarge",
            (64,512): "r6i.16xlarge",
        }
        #self.displayResults()
        # self.bounds_ = self.createBounds()
        # self.constraints_ = self.createConstraints()
        # self.objective_function_ = self.createObjectiveFunction()
        # res = minimize(self.objective_function_, (1), method='SLSQP', bounds=self.bounds_,constraints=self.constraints_)
        # print(res)
        # if(not res.success):
        #     print("Time constraint was too harsh. Given that you have max " + str(self.relevant_cpu_list_[2]) + " CPUs. The fastest time you can do is " + str(self.timeConstraintFn(self.relevant_cpu_list_[2])) + " seconds.")
        # # print(linear_coefficients)

        # min_yaml_contents = yaml_file_contents
        # f = open("demofile3.txt", "w")
        # file_contents = ""
        # for line in yaml_file_contents:
        #     file_contents += line
        # f.write(file_contents)
        # f.close()

        # parsed through_yaml = False
        # print(yaml_file_contents[cpu_start_index])
        # os.system("cat " + yaml_file_file)

    def fullOptimization(self):
        print(self.gpu_type_)
        if(self.gpu_type_ == ""):
            self.findCpuValues()
        else:
            self.findGpuValues()
        self.createBenchmarks(self.gpu_type_)
        self.runSkyBenchmarks(self.count_)
        print("Time benchmark results")
        print(self.benchmark_time_results_)
        print("Cost benchmark results")
        print(self.benchmark_cost_results_)
        print("DONEZO DONEZO")
        #exit()
        self.time_model_info_ = self.regressionSolver(
            self.benchmark_time_results_, "Time"
        )
        self.cost_model_info_ = self.regressionSolver(
            self.benchmark_cost_results_, "Cost"
        )
        #self.findSetupTime()
        self.timeModel_ = self.time_model_info_[0]
        self.time_model_coefficients_ = self.time_model_info_[1]
        print(self.timeModel_)
        print(self.time_model_coefficients_)
        self.costModel_ = self.cost_model_info_[0]
        self.cost_model_coefficients_ = self.cost_model_info_[1]
        print(self.costModel_)
        print(self.cost_model_coefficients_)
        self.result_queue_.append(str(self.timeModel_))
        self.result_queue_.append(str(self.time_model_coefficients_))
        self.result_queue_.append(str(self.costModel_))
        self.result_queue_.append(str(self.cost_model_coefficients_))
        #self.timeModel_ = self.makeModel(self.time_model_info_)
        #self.costModel_ = self.makeModel(self.cost_model_info_)

        #if self.debug_:
        #    self.displayRelevantOptimizationInfo()
        self.cost_optimization_results_ = self.solveOptimization(
            OptimizationFunctionType.COST, OptimizationFunctionType.TIME
        )
        self.time_optimization_results_ = self.solveOptimization(
            OptimizationFunctionType.TIME, OptimizationFunctionType.COST
        )
        print("COST INSTANCE")
        self.findInstance(self.cost_optimization_results_,"cost")
        
        self.findInstance(self.time_optimization_results_,"time")
        self.result_queue_.append(str(self.cost_optimization_results_))
        self.result_queue_.append(str(self.optimal_cost_instance_))
        self.result_queue_.append(str(self.time_optimization_results_))
        self.result_queue_.append(str(self.optimal_time_instance_))
        for line in self.result_queue_:
            self.data_file_ += '\n' + line
        with open(self.data_file_name_, "w") as file:
            file.write(self.data_file_)
        return [self.cost_optimization_results_,self.time_optimization_results_]

    def findInstance(self,optimization_results,optimization_type):
        print("Start find instance")
        print(optimization_results)
        optimal_cpu_count =  optimization_results[0][0]
        optimal_memory_count = None
        if(self.gpu_type_ == ""):
            optimal_memory_count = optimization_results[0][1]
        print("Optimal CPU Count")
        print(optimal_cpu_count)
        print("Optimal Memory Count")
        print(optimal_memory_count)
        rounded_cpu_count = self.binarySearchRounding(optimal_cpu_count)
        rounded_memory_count = None
        if self.gpu_type_== "":
            potential_memory_amounts = [rounded_cpu_count*2,rounded_cpu_count*4,rounded_cpu_count*8]
            min_diff = 1000
            for potential_memory_amount in potential_memory_amounts:
                if(abs(potential_memory_amount - optimal_memory_count) < min_diff):
                    min_diff = abs(potential_memory_amount - optimal_memory_count)
                    rounded_memory_count = potential_memory_amount
        else:
            rounded_memory_count = self.mem_to_cpu_ratio_for_gpu_ * rounded_cpu_count
        optimal_hardware = (rounded_cpu_count,rounded_memory_count)
        optimal_instance = None
        if self.gpu_type_ == "":
            optimal_instance = self.no_gpu_map_[optimal_hardware]
        elif self.gpu_type_ == "T4":
            optimal_instance = self.t4_map_[optimal_hardware]
        else:
            print("Need to implement " + self.gpu_type_ + " Instance Rounding!!!")
            exit()
        if(optimization_type == "cost"):
            self.optimal_cost_instance_ = optimal_instance
        elif(optimization_type == "time"):
            self.optimal_time_instance_ = optimal_instance
        else:
            print(optimization_type + "is not a valid optimization type")
    
    def binarySearchRounding(self,optimal_cpu_count):
        low = 0
        high = len(self.possible_cpus_) - 1        
        while low <= high:
            mid = (low + high) // 2
            if(self.possible_cpus_[mid] == optimal_cpu_count):
                return self.possible_cpus_[mid]
            if(self.possible_cpus_[mid] < optimal_cpu_count):
                low = mid + 1
            else:
                high = mid - 1
        if low == 0:
            return self.possible_cpus_[0]
        if (high == len(self.possible_cpus_) - 1):
            return self.possible_cpus_[-1]
        closest_left = self.possible_cpus_[high]
        closest_right = self.possible_cpus_[low]
        if(abs(closest_left - optimal_cpu_count) <= abs(closest_right - optimal_cpu_count)):
            return closest_left
        else:
            return closest_right


    def findGpuValues(self):
        if self.debug_:
            print("Finding relevant GPU values for fitting")
        cpu_start_string = "    candidates:\n"
        print("IN HERE")
        print(os.getcwd())
        cwd = os.getcwd()
        self.gpu_yaml_file_ = cwd + "/" + self.gpu_yaml_file_
        print(self.gpu_yaml_file_)
        self.yaml_file_contents_ = None
        with open(self.gpu_yaml_file_, "r") as file:
            self.yaml_file_contents_ = file.readlines()
        print(self.yaml_file_contents_)
        cpu_index = self.yaml_file_contents_.index(cpu_start_string) + 1
        self.cpu_start_index_ = cpu_index
        cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
        cpu_substring = cpu_string[
            cpu_string.find("cpus:") + self.cpu_str_offset_ : cpu_string.find("}")
        ]
        cpu_replacement_substring = " 1+, memory: 1+, accelerators: " + self.gpu_type_
        new_cpu_string = cpu_string.replace(
                    cpu_substring, cpu_replacement_substring
                )
        self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
        filename = "min_" + self.gpu_type_ + ".yaml"
        f = open(filename, "w")
        file_contents = ""
        for line in self.yaml_file_contents_:
            file_contents += line
        f.write(file_contents)
        f.close()

        gpu_benchmark_setup_command = (
            "bash gpu_benchmark.sh " + filename + " gpucheck"
        )
        benchmark_setup = subprocess.Popen(
            gpu_benchmark_setup_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        # output = ""
        gpu_headers = None
        gpu_values = None
        found_line = False
        gpu_line_gap = 3
        while True:
            line = benchmark_setup.stdout.readline()
            # if line == '' and process.poll() is not None:
            #    break
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                #print(line)
                if self.min_gpu_substring_1_ in line and self.min_gpu_substring_2_ in line:
                    gpu_headers = line.split()
                    print(gpu_headers)
                    found_line = True
                if found_line:
                    gpu_line_gap -= 1
                if gpu_line_gap == 0:
                    gpu_values = line.split()
                    print(gpu_values)
                    min_cpu_index = gpu_headers.index('vCPUs') - 1
                    min_mem_index = gpu_headers.index('Mem(GB)') - 1
                    cpu_min = float(gpu_values[min_cpu_index])
                    mem_min = float(gpu_values[min_mem_index])
                    self.mem_to_cpu_ratio_for_gpu_ = mem_min / cpu_min
                    cpu_max = 16
                    log_cpu_min = math.log2(cpu_min)
                    log_cpu_max = math.log2(cpu_max)
                    log_cpu_mid = int((log_cpu_min + log_cpu_max) / 2)
                    cpu_mid = 2 ** log_cpu_mid
                    self.relevant_cpu_list_.append(cpu_min)
                    self.relevant_cpu_list_.append(cpu_mid)
                    self.relevant_cpu_list_.append(cpu_max)
                    
            else:
                break



    def findCpuValues(self):
        if self.debug_:
            print("Finding relevant CPU values for fitting")
        cpu_start_string = "    candidates:\n"
        print("CPU HERE")
        print(os.getcwd())
        cwd = os.getcwd()
        self.yaml_file_ = cwd + "/" + self.yaml_file_
        print(self.yaml_file_)
        self.yaml_file_contents_ = None
        with open(self.yaml_file_, "r") as file:
            self.yaml_file_contents_ = file.readlines()
        print(self.yaml_file_contents_)
        cpu_index = self.yaml_file_contents_.index(cpu_start_string) + 1
        self.cpu_start_index_ = cpu_index
        cpu_list = []
        yaml_parsed = False
        self.cpu_str_offset_ = 5
        while not yaml_parsed:
            try:
                cpu_string = self.yaml_file_contents_[cpu_index]
                cpu_substring = cpu_string[
                    cpu_string.find("cpus:")
                    + self.cpu_str_offset_ : cpu_string.find("}")
                ]
                cpu_num = int(cpu_substring)
                cpu_list.append(cpu_num)
                cpu_index = cpu_index + 1
            except:
                yaml_parsed = True
        cpu_list.sort()
        cpu_min = cpu_list[0]
        cpu_max = cpu_list[len(cpu_list) - 1]
        cpu_mid = cpu_list[int(len(cpu_list) / 2)]
        self.max_cpu_ = cpu_max
        self.max_memory_ = cpu_max * 8
        self.relevant_cpu_list_.append(cpu_min)
        self.relevant_cpu_list_.append(cpu_mid)
        self.relevant_cpu_list_.append(cpu_max)
        if self.debug_:
            print("CPU Min: " + str(cpu_min))
            print("CPU Mid: " + str(cpu_mid))
            print("CPU Max: " + str(cpu_max))

    def createBenchmarks(self,gpu_type):
        if self.debug_:
            print("Creating benchmarks")
        if(gpu_type == ""):
            for i in range(self.benchmark_range_):
                cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
                cpu_substring = cpu_string[
                    cpu_string.find("cpus:") + self.cpu_str_offset_ : cpu_string.find("}")
                ]
                for j in range(self.benchmark_range_):
                    memory = self.relevant_cpu_list_[i] * (2 ** (j + 1))
                    cpu_replacement_substring = (
                        " " + str(self.relevant_cpu_list_[i]) + ", memory: " + str(memory)
                    )
                    new_cpu_string = cpu_string.replace(
                        cpu_substring, cpu_replacement_substring
                    )
                    self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
                    filename = "cpu_"+str(i) + "_mem" + str(j) + ".yaml"
                    self.benchmark_yaml_list_.append(filename)
                    self.relevant_hardware_list_.append((self.relevant_cpu_list_[i],memory))
                    f = open(filename, "w")
                    file_contents = ""
                    for line in self.yaml_file_contents_:
                        file_contents += line
                    f.write(file_contents)
                    f.close()
        else:
            for cpu in self.relevant_cpu_list_:
                memory = self.mem_to_cpu_ratio_for_gpu_ * cpu
                cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
                cpu_substring = cpu_string[
                    cpu_string.find("cpus:") + self.cpu_str_offset_ : cpu_string.find("}")
                ]
                print(cpu_substring)
                cpu_replacement_substring = (
                        " " + str(int(cpu)) + ", memory: " + str(int(memory)) +", accelerators: " + gpu_type
                    )
                print(cpu_replacement_substring)
                new_cpu_string = cpu_string.replace(
                    cpu_substring, cpu_replacement_substring
                )
                self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
                filename = "cpu_"+str(int(cpu)) + "_mem" + str(int(memory)) + "_accel_" + gpu_type + ".yaml"
                self.benchmark_yaml_list_.append(filename)
                if self.gpu_type_ == "":
                    self.relevant_hardware_list_.append((cpu,memory))
                else:
                    self.relevant_hardware_list_.append(cpu)
                f = open(filename, "w")
                file_contents = ""
                for line in self.yaml_file_contents_:
                    file_contents += line
                f.write(file_contents)
                f.close()
                

    def runSkyBenchmarks(self,count):
        if self.debug_:
            print("Run Sky Benchmarks")
        thread_list = []
        for i in range(len(self.relevant_hardware_list_)):
            benchmark_name = "benchmark"+str(count * 9 + i)
            print(benchmark_name)
            t = threading.Thread(
                target=self.skyBenchmark,
                args=(
                    self.benchmark_yaml_list_[i],
                    benchmark_name,
                    self.relevant_hardware_list_[i],
                ),
            )
            t.start()
            thread_list.append(t)
        # t1 = threading.Thread(
        #     target=self.skyBenchmark,
        #         args=(
        #             self.benchmark_yaml_list_[0],
        #             "benchmark10",
        #             self.relevant_hardware_list_[0],
        #         ),
        # )
        # t1.start()
        # thread_list.append(t1)

        # t2 = threading.Thread(
        #     target=self.skyBenchmark,
        #         args=(
        #             self.benchmark_yaml_list_[1],
        #             "benchmark11",
        #             self.relevant_hardware_list_[1],
        #         ),
        # )
        # t2.start()
        # thread_list.append(t2)

        # t3 = threading.Thread(
        #     target=self.skyBenchmark,
        #         args=(
        #             self.benchmark_yaml_list_[2],
        #             "benchmark12",
        #             self.relevant_hardware_list_[2],
        #         ),
        # )
        # t3.start()
        # thread_list.append(t3)
        for i in range(len(thread_list)):
            thread_list[i].join()
            #TODO: 1. Fix GPU bug by making sure we don't kill it early. Also, send the equations in the text file too. And also save optimization results (put it all in text file)
            #TODO: Figure out mapping between various hardware to instances
        
        # if self.debug_:
        #     print("Finished running sky benchmarks. Will now terminate clusters.")

        # benchmark_down_command = "sky down --all < benchmark_input.txt"
        # benchmark_down = subprocess.Popen(
        #     benchmark_down_command,
        #     shell=True,
        #     stdout=subprocess.PIPE,
        # )
        # while True:
        #     line = benchmark_down.stdout.readline()
        #     if line:
        #         line_byte = line.strip()
        #         line = line_byte.decode("utf-8")
        #         print(line)
        #     else:
        #         break

        # benchmark_delete_command = "sky bench delete --all < benchmark_input.txt"
        # benchmark_delete = subprocess.Popen(
        #     benchmark_delete_command,
        #     shell=True,
        #     stdout=subprocess.PIPE,
        # )
        
        # while True:
        #     line = benchmark_delete.stdout.readline()
        #     if line:
        #         line_byte = line.strip()
        #         line = line_byte.decode("utf-8")
        #         print(line)
        #     else:
        #         break
        if self.debug_:
            print("Successfully terminated clusters.")
            # Retrieve the result from the queue
        result = self.result_queue_
        
        # t1 = threading.Thread(target=self.skyBenchmark,args=(self.cpu_benchmark_yaml_list_[0],self.benchmark_name_list_[0],self.relevant_cpu_list_[0]))
        # t1.start()
        # t2 = threading.Thread(target=self.skyBenchmark,args=(self.cpu_benchmark_yaml_list_[1],self.benchmark_name_list_[1],self.relevant_cpu_list_[1]))
        # t2.start()
        # t1.join()
        # t2.join()
        # for i in range(len(self.cpu_benchmark_yaml_list_)):

    def skyBenchmark(self, yaml_file, benchmark_name, hardware_count):
        if self.debug_:
            print("Sky benchmark: " + yaml_file)
        benchmark_setup_command = (
            "bash benchmark.sh " + yaml_file + " " + benchmark_name
        )
        benchmark_setup = subprocess.Popen(
            benchmark_setup_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        # output = ""
        num_lines_to_cost_data = 2
        while True:
            line = benchmark_setup.stdout.readline()
            # if line == '' and process.poll() is not None:
            #    break
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                line_output = "Hardware count " + str(hardware_count) + ": " + line
                print(line_output)
                # import pdb
                # pdb.set_trace()
                if(self.gpu_type_ == ""):
                    if self.got_cost_info_:
                        if num_lines_to_cost_data == 0:
                            num_lines_to_cost_data = 2
                            self.got_cost_info_ = False
                            line_array = line.split()
                            print("DEBUG COST RESULTS")
                            print(line_array)
                            print(self.benchmark_cost_substring_cost_index_)
                            self.benchmark_cost_results_.append((hardware_count,float(
                                        line_array[
                                            self.benchmark_cost_substring_cost_index_
                                        ]
                                    )
                                    / self.SECONDS_PER_HOUR))
                        else:
                            num_lines_to_cost_data -= 1
                    if (
                        self.benchmark_cost_substring_1_ in line
                        and self.benchmark_cost_substring_2_ in line
                    ):
                        self.got_cost_info_ = True
                        num_lines_to_cost_data -= 1

                # if(line == "To teardown the clusters:"):
                #     print("OVER")
                #     break

                # output += str(line.strip()) + "\n"
                # print(line.strip())  # Optional: Print the output in real-time
            else:
                break
        if self.debug_:
            print("Hardware count " + str(hardware_count) + ": " + "Cluster is setup")

        time_remaining = 5
        if(self.is_slam_):
            time_remaining = 1
        while time_remaining > 0:
            print(
                "Hardware count "
                + str(hardware_count)
                + ": "
                + "Benchmarking. "
                + str(time_remaining)
                + " minutes remaining"
            )
            time.sleep(self.SECONDS_PER_MINUTE)
            time_remaining -= 1
        if self.debug_:
            print("Hardware count " + str(hardware_count) + ": " + "Check status")
        benchmark_log_command = "sky bench show " + benchmark_name
        benchmark_log = subprocess.Popen(
            benchmark_log_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        seconds_per_step_line = ""
        while True:
            line = benchmark_log.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                line_output = "Hardware count " + str(hardware_count) + ": " + line
                print(line_output)
                if "sky-bench" in line:
                    seconds_per_step_line = line
            else:
                break
        print("HOLD THE LINE")
        print(seconds_per_step_line)
        self.result_queue_.append(seconds_per_step_line)
        seconds_per_step_array = seconds_per_step_line.split()
        if self.debug_:
            print(seconds_per_step_array)
        # Weird exception where if duration is exactly 5 minutes and 0 seconds, everything after is shifted by 1
        seconds_per_step = None
        dollars_per_second = None
        duration = 0
        num_steps = 0
        print("STARTING APPEND THING")
        print(seconds_per_step_array)
        print(self.seconds_per_step_index_)
        print(self.seconds_per_step_index_ - 3)
        print(seconds_per_step_array[self.seconds_per_step_index_ - 3])
        if "s" not in seconds_per_step_array[self.seconds_per_step_index_ - 3]:
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_ - 1]
            print(seconds_per_step)
            
            # print(self.duration_min_index_)
            # print(seconds_per_step_array[self.duration_min_index_][:-1])
            
            # duration = (
            #     int(seconds_per_step_array[self.duration_min_index_][:-1])
            #     * self.SECONDS_PER_MINUTE
            # )
            # print(duration)
            # print(self.seconds_per_step_index_)
            num_steps = int(seconds_per_step_array[self.seconds_per_step_index_ - 2])
            print(num_steps)
            if(self.gpu_type_ != ""):
                dollars_per_step = seconds_per_step_array[self.seconds_per_step_index_]
                print(dollars_per_step)
                dollars_per_second = float(dollars_per_step) / float(seconds_per_step)
                print(dollars_per_second)
        else:
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_]
            print(seconds_per_step)
            print(self.duration_min_index_)
            print(seconds_per_step_array[self.duration_min_index_][:-1])
            print(seconds_per_step_array[self.duration_sec_index_][:-1])
            # duration = int(
            #     seconds_per_step_array[self.duration_min_index_][:-1]
            # ) * self.SECONDS_PER_MINUTE + int(
            #     seconds_per_step_array[self.duration_sec_index_][:-1]
            # )
            
            # print(duration)
            print(self.seconds_per_step_index_)
            num_steps = int(seconds_per_step_array[self.seconds_per_step_index_ - 1])
            print(num_steps)
            # import pdb
            # pdb.set_trace()
            if(self.gpu_type_ != ""):
                dollars_per_step = seconds_per_step_array[self.seconds_per_step_index_ + 1]
                print(dollars_per_step)
                dollars_per_second = float(dollars_per_step) / float(seconds_per_step)
                print(dollars_per_second)
        self.benchmark_time_results_.append((hardware_count, float(seconds_per_step)))
        if(self.gpu_type_ != ""):
            self.benchmark_cost_results_.append((hardware_count, dollars_per_second))
        #self.num_steps_and_duration_.append(
        #    (num_steps, duration, hardware_count, float(seconds_per_step))
        #)
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

    def findSetupTime(self):
        if self.debug_:
            print("Finding the average setup time to get everything on the cloud")
            print(self.num_steps_and_duration_)
        setup_times = []
        for item in self.num_steps_and_duration_:
            (num_steps, duration, hardware_count, seconds_per_step) = item
            setup_time = duration - (seconds_per_step * num_steps)
            setup_times.append(setup_time)
        self.average_setup_time_ = sum(setup_times) / len(setup_times)
        if self.debug_:
            print("Setup times: " + str(setup_times))
            print("Average setup time: " + str(self.average_setup_time_))

    def regressionSolver(self, data, purpose):
        if self.debug_:
            print(
                "Running least squares regression to solve for "
                + purpose
                + " per hardware"
            )
        data_np = np.array(data)
        # Linear regression
        x = data_np[:, 0]  # np.array(list(self.benchmark_time_results_.keys()))
        y = data_np[:, 1]  # np.array(list(self.benchmark_time_results_.values()))
        x_np = None
        y_np = None
        print("X type: " + str(type(x[0])))
        print("Y type: " + str(type(y[0])))
        if(type(x[0]) is tuple):
            x_np = np.array([list(item) for item in x])
            
        else:
            x_np = np.array(x)
            
        if(type(y[0]) is tuple):
            y_np = np.array([list(item) for item in y])
        else:
            y_np = np.array(y)

        if self.debug_:
            print("Input data: " + str(x_np))
            print("Output data: " + str(y_np))
        if(self.gpu_type_ != ""):
            potential_model_object = GPUModel(x_np,y_np)
            potential_model = potential_model_object.getModel()
            if self.debug_:
                print("Model for " + str(purpose) + ": " + str(potential_model))
            return potential_model
        else:
            potential_model_object = Model(x_np,y_np)
            potential_model = potential_model_object.getModel()
            if self.debug_:
                print("Model for " + str(purpose) + ": " + str(potential_model))
            return potential_model
        # log_x = np.log(x)
        # log_y = np.log(y)
        # regressions = []
        # linear_regression = self.linearRegression(x, y, ModelType.LINEAR)
        # regressions.append(linear_regression)
        # power_regression = self.linearRegression(log_x, log_y, ModelType.POWER)
        # regressions.append(power_regression)
        # exponential_regression = self.linearRegression(x, log_y, ModelType.EXPONENTIAL)
        # regressions.append(exponential_regression)
        # logarithmic_regression = self.linearRegression(log_x, y, ModelType.LOGARITHMIC)
        # regressions.append(logarithmic_regression)
        # hyperbolic_regression = self.hyperbolicRegression(x, y)
        # regressions.append(hyperbolic_regression)
        # if(purpose == 'Time'):
        #    quadratic_regression = self.quadraticRegression(x,y)
        #    regressions.append(quadratic_regression)
        max_rsquared = 0
        coefficients = None
        model = None
        for regression in regressions:
            (rsquared, linear_coefficients, constraint_type) = regression
            if abs(rsquared) > max_rsquared:
                max_rsquared = abs(rsquared)
                coefficients = linear_coefficients
                model = constraint_type
        if self.debug_:
            print(purpose + " Model R^2: " + str(max_rsquared))
            print(purpose + " Model Coefficients: " + str(coefficients))
            print(purpose + " Model Type: " + str(model))
            # print("Linear regression: " + str(linear_regression))
            # print("Power regression: " + str(power_regression))
            # print("Exponential regression: " + str(exponential_regression))
            # print("Logarithmic regression: " + str(logarithmic_regression))
        return (coefficients, model)
        # A = np.vstack([x, np.ones(len(x))]).T
        # linear_coefficients = np.linalg.lstsq(A, y, rcond=None)[0]
        # y_pred = linear_coefficients[0] * x + linear_coefficients[1]
        # rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        # if self.debug_:
        #     print("R squared value: " + str(rsquared))

    # def solveCostPerHardwareModel(self):
    #     if self.debug_:
    #         print("Running least squares regression to solve for cost per hardware")
    #     cost_data = np.array()

    def hyperbolic_model(self, x, A, B):
        return A + B / x

    def hyperbolicRegression(self, x, y, constraint_type=ModelType.HYPERBOLIC):
        coefficients, _ = curve_fit(self.hyperbolic_model, x, y)
        y_pred = self.hyperbolic_model(x, coefficients[0], coefficients[1])
        rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        return (rsquared, coefficients, constraint_type)

    def linearRegression(self, x, y, constraint_type):
        A = np.vstack([x, np.ones(len(x))]).T
        linear_coefficients = np.linalg.lstsq(A, y, rcond=None)[0]
        y_pred = linear_coefficients[0] * x + linear_coefficients[1]
        rsquared = None
        if (
            constraint_type == ModelType.POWER
            or constraint_type == ModelType.EXPONENTIAL
        ):
            y = np.exp(y)
            y_pred = np.exp(y_pred)
        rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        return (rsquared, linear_coefficients, constraint_type)

    def quadraticRegression(self, x, y, constraint_type=ModelType.QUADRATIC):
        A = np.vstack([x**2, x, np.ones(len(x))]).T
        print("Quad debugging")
        print(A)
        print(y)
        linear_coefficients = np.linalg.lstsq(A, y, rcond=None)[0]
        print(linear_coefficients)
        y_pred = (
            linear_coefficients[0] * (x**2)
            + linear_coefficients[1] * x
            + linear_coefficients[2]
        )
        rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        return (rsquared, linear_coefficients, constraint_type)

    def createBounds(self):
        if self.debug_:
            print("Creating bounds for optimization problem")
        low_cpu = self.relevant_cpu_list_[0]
        high_cpu = self.relevant_cpu_list_[2]
        low_mem = self.relevant_cpu_list_[0] * 2
        high_mem = self.max_memory_
        if(self.gpu_type_!= ""):
            low_mem = self.relevant_cpu_list_[0] * self.mem_to_cpu_ratio_for_gpu_
            high_mem = self.relevant_cpu_list_[2] * self.mem_to_cpu_ratio_for_gpu_
        if self.debug_:
            print("Low CPU bound: " + str(low_cpu))
            print("High CPU bound: " + str(high_cpu))
            print("Low Memory bound: " + str(low_mem))
            print("High Memory bound: " + str(high_mem))
        if(self.gpu_type_ == ""):
            return Bounds([low_cpu, low_mem], [high_cpu, high_mem]) #[(low_cpu, high_cpu)]
        else:
            return Bounds([low_cpu],[high_cpu])

    def displayRelevantOptimizationInfo(self):
        # for cpu_count in self.relevant_cpu_list_:
        #     print("CPU Count: " + str(cpu_count))
        #     print("Time (Sec/Step): " + str(self.timeModel_(cpu_count)))
        #     print("Cost ($/Sec): " + str(self.costModel_(cpu_count)))
        #     print("$/Hr: " + str(self.costModel_(cpu_count) * self.SECONDS_PER_HOUR))
        #     print("Total Cost: " + str(self.costFunction()(cpu_count)))
        #     print("Total Time: " + str(self.timeFunction()(cpu_count)))
        
        for relevant_hardware in self.relevant_hardware_list_:
            relevant_hardware = np.array(relevant_hardware)
            print("Hardware Count: " + str(relevant_hardware))
            # print("Hardware type: " + str(type(relevant_hardware)))
            # print("Hardware shape: " + str(relevant_hardware.shape))
            # print("Time model coefficients: " + str(self.time_model_coefficients_))
            # print("Type coefficients: " + str(type(self.time_model_coefficients_)))
            # print("Time shape: "+ str(self.time_model_coefficients_.shape))
            # print(relevant_hardware[0])
            # print(relevant_hardware[1])
            # print(self.time_model_coefficients_[0])
            # print(self.time_model_coefficients_[1])
            # print(self.time_model_coefficients_[2])
            # print(self.time_model_coefficients_[3])
            
            print("Time (Sec/Step): " + str(self.timeModel_(relevant_hardware,self.time_model_coefficients_,predict=True,single=False)))
            print("Cost ($/Sec): " + str(self.costModel_(relevant_hardware,self.cost_model_coefficients_,predict=True,single=False)))

    def createConstraints(self, constraint_function_type):
        if self.debug_:
            print("Creating constraints for optimization problem")
        constraints = []
        mem_cpu_ratio_lower_bound = 2
        mem_cpu_ratio_upper_bound = 8
        memory_constraint = None
        if(self.gpu_type_ == ""):
            memory_constraint = NonlinearConstraint(self.memoryFunction(),mem_cpu_ratio_lower_bound,mem_cpu_ratio_upper_bound)
            constraints.append(memory_constraint)
        
        match (constraint_function_type):
            case OptimizationFunctionType.COST:
                cost_lower_bound = 0
                cost_upper_bound = self.max_cost_
                cost_constraint = NonlinearConstraint(
                    self.costFunction(), cost_lower_bound, cost_upper_bound
                )
                constraints.append(cost_constraint)
                return constraints
            case OptimizationFunctionType.TIME:
                time_lower_bound = 0
                time_upper_bound = self.max_time_
                time_constraint = NonlinearConstraint(
                    self.timeFunction(), time_lower_bound, time_upper_bound
                )
                constraints.append(time_constraint)
                return constraints

        # cost_lower_bound = 0
        # cost_upper_bound = self.max_cost_

        # time_lower_bound = 0
        # time_upper_bound = self.max_time_

        # self.timeModel_ = self.makeModel(self.time_model_info_)
        # self.costModel_ = self.makeModel(self.cost_model_info_)
        # self.costConstraintFn = self.costFunction()
        # self.timeConstraintFn = self.timeFunction()

        # if (self.debug_):
        #     print("CPU COUNT 2")
        #     print("Time (Sec/Step): " + str(self.timeModel_(2)))
        #     print("Cost ($/Sec): " + str(self.costModel_(2)))
        #     print("$/Hr: " + str(self.costModel_(2) * self.SECONDS_PER_HOUR))
        #     print("Total Cost: " + str(self.costConstraintFn(2)))
        #     print("Total Time: " + str(self.timeConstraintFn(2)))

        #     print("CPU COUNT 16")
        #     print("Time (Sec/Step): " + str(self.timeModel_(16)))
        #     print("Cost ($/Sec): " + str(self.costModel_(16)))
        #     print("$/Hr: " + str(self.costModel_(16) * self.SECONDS_PER_HOUR))
        #     print("Total Cost: " + str(self.costConstraintFn(16)))
        #     print("Total Time: " + str(self.timeConstraintFn(16)))

        #     print("CPU COUNT 64")
        #     print("Time (Sec/Step): " + str(self.timeModel_(64)))
        #     print("Cost ($/Sec): " + str(self.costModel_(64)))
        #     print("$/Hr: " + str(self.costModel_(64) * self.SECONDS_PER_HOUR))
        #     print("Total Cost: " + str(self.costConstraintFn(64)))
        #     print("Total Time: " + str(self.timeConstraintFn(64)))
        # cost_constraint = NonlinearConstraint(self.costConstraintFn,cost_lower_bound,cost_upper_bound)
        # time_constraint = NonlinearConstraint(self.timeConstraintFn,time_lower_bound,time_upper_bound)
        # constraints.append(time_constraint)
        # return constraints

    def getTimeOptimization(self):
        return self.time_optimization_results_
    
    def getCostOptimization(self):
        return self.cost_optimization_results_
    
    def createObjectiveFunction(self, constraint_function_type):
        if self.debug_:
            print("Creating objective function for optimization problem")
        match (constraint_function_type):
            case OptimizationFunctionType.COST:
                return self.costFunction()
            case OptimizationFunctionType.TIME:
                return self.timeFunction()

    def memoryFunction(self):
        def memFn(x):
            return x[1] / x[0]
        return memFn

    def costFunction(self):
        def costFn(x):
            seconds_per_step = self.timeModel_(x,self.time_model_coefficients_,predict = False,single = True)
            total_time_in_seconds = (
                seconds_per_step * self.steps_
            )# + self.average_setup_time_
            dollars_per_second = self.costModel_(x,self.cost_model_coefficients_,predict=False,single=True)
            cost = dollars_per_second * total_time_in_seconds
            return cost

        return costFn

    def timeFunction(self):
        def timeFn(x):
            seconds_per_step = self.timeModel_(x,self.time_model_coefficients_,predict = False,single = True)
            total_time_in_seconds = (
                seconds_per_step * self.steps_
            ) + self.average_setup_time_
            return total_time_in_seconds

        return timeFn

    def makeModel(self, model_info):
        match model_info[1]:
            case ModelType.LINEAR:

                def model(x):
                    x = np.array(x)
                    y = model_info[0][0] * x + model_info[0][1]
                    return y

                return model
            case ModelType.POWER:

                def model(x):
                    x = np.array(x)
                    log_y = model_info[0][0] * np.log(x) + model_info[0][1]
                    y = np.exp(log_y)
                    return y

                return model
            case ModelType.EXPONENTIAL:

                def model(x):
                    x = np.array(x)
                    log_y = model_info[0][0] * x + model_info[0][1]
                    y = np.exp(log_y)
                    return y

                return model
            case ModelType.LOGARITHMIC:

                def model(x):
                    x = np.array(x)
                    y = model_info[0][0] * np.log(x) + model_info[0][1]
                    return y

                return model
            case ModelType.QUADRATIC:

                def model(x):
                    x = np.array(x)
                    y = (
                        model_info[0][0] * (x**2)
                        + model_info[0][1] * x
                        + model_info[0][2]
                    )
                    return y

            case ModelType.HYPERBOLIC:

                def model(x):
                    x = np.array(x)
                    y = self.hyperbolic_model(x, model_info[0][0], model_info[0][1])
                    return y

                return model

    def solveOptimization(self, objective_function_type, constraint_function_type):
        if objective_function_type == constraint_function_type:
            print("Constraint function can't be objective function")
            print("Program now exiting")
            exit()
        bounds = self.createBounds()
        constraints = self.createConstraints(constraint_function_type)
        objective_function = self.createObjectiveFunction(objective_function_type)
        initial_guess = np.array([1,1])
        if(self.gpu_type_ != ""):
            initial_guess = np.array([self.relevant_cpu_list_[0]])
        optimization_method = "SLSQP"
        optimization_result = minimize(
            objective_function,
            initial_guess,
            method=optimization_method,
            bounds=bounds,
            constraints=constraints,
            tol=1e-5,
            options={'maxiter': 10000}
        )
        match objective_function_type:
            case OptimizationFunctionType.COST:
                print("I GOT THE COST OPTIMIZATION RESULT")
            case OptimizationFunctionType.TIME:
                print("I GOT THE TIME OPTIMIZATION RESULT")
        print(optimization_result)
        print("Time: " + str(self.timeFunction()(optimization_result.x)))
        print("Cost: " + str(self.costFunction()(optimization_result.x)))
        return optimization_result.x,self.timeFunction()(optimization_result.x),self.costFunction()(optimization_result.x)
        hardware_result = None
        if not optimization_result.success:
            hardware_result = optimization_result.x
            print(str(objective_function_type)[25:] + " Optimization Failed")
            match objective_function_type:
                case OptimizationFunctionType.COST:
                    print(
                        "Time constraint upper bound is too low. Given that you have max "
                        + str(self.relevant_cpu_list_[2])
                        + " CPUs. The fastest time you can do is "
                        + str(self.timeFunction()(self.relevant_cpu_list_[2]))
                        + " seconds."
                    )
                case OptimizationFunctionType.TIME:
                    print(
                        "Cost constraint upper bound is too low. To gauge the lowest cost, look at the cost optimization under an infinite time constraint"
                    )
        else:
            hardware_result = self.hardwareIntegerResults(
                optimization_result.x, objective_function_type
            )
        # match objective_function_type:
        #     case OptimizationFunctionType.COST:
        #         for i in range(len(hardware_result)):
        #             hardware_result[i] = math.floor(hardware_result[i])
        #     case OptimizationFunctionType.TIME:
        #         for i in range(len(hardware_result)):
        #             hardware_result[i] = math.ceil(hardware_result[i])
        if (
            self.costFunction()(hardware_result) <= self.max_cost_
            and self.timeFunction()(hardware_result) <= self.max_time_
        ):
            self.solutions_in_constraints_.append(
                (objective_function_type, hardware_result)
            )
        else:
            self.solutions_outside_constraints_.append(
                (objective_function_type, hardware_result)
            )
        if self.debug_:
            print("Optimization results: " + str(optimization_result))
        # match objective_function_type:
        #     case OptimizationFunctionType.COST:
        #     bounds = self.createBounds()
        #     constraints = self.createConstraints(OptimizationFunctionType.TIME)
        #     obc= self.createObjectiveFunction()
        #     res = minimize(self.objective_function_, (1), method='SLSQP', bounds=self.bounds_,constraints=self.constraints_)
        #     print(res)
        #     if(not res.success):

        #     # print(linear_coefficients)
        #         return "COST"
        #     case OptimizationFunctionType.TIME:
        #         return "Time not implemented yet"

    def hardwareIntegerResults(self, optimization_result, objective_function_type):
        floor_and_ceiling = [
            (math.floor(num), math.ceil(num)) for num in optimization_result
        ]
        permutations = list(product(*floor_and_ceiling))
        successful_permutations = []
        for permutation in permutations:
            if (
                self.costFunction()(permutation) <= self.max_cost_
                and self.timeFunction()(permutation) <= self.max_time_
            ):
                successful_permutations.append(permutation)
        if self.debug_:
            print("Before sort")
            print(successful_permutations)
        sorted_successful_permutations = successful_permutations
        if objective_function_type == OptimizationFunctionType.COST:
            sorted_successful_permutations = sorted(
                successful_permutations, key=self.costFunction()
            )
        elif objective_function_type == OptimizationFunctionType.TIME:
            sorted_successful_permutations = sorted(
                successful_permutations, key=self.timeFunction()
            )
        if self.debug_:
            print("After sort")
            print(sorted_successful_permutations)
        return sorted_successful_permutations[0]

    def displayResults(self):
        success_count = len(self.solutions_in_constraints_)
        fail_count = len(self.solutions_outside_constraints_)
        print(
            "Program yielded "
            + str(success_count)
            + " successful optimization(s) and "
            + str(fail_count)
            + " failed optimization(s).\n"
        )
        for solution in self.solutions_in_constraints_:
            print(
                "Successful "
                + str(solution[0])[self.optimization_function_type_length_ :].lower()
                + " optimization results"
            )
            print("------------------------------------")
            print("Optimal hardware: " + str(solution[1][0]) + " CPUs")
            print("Total Cost: $" + str(self.costFunction()(solution[1])[0]))
            print(
                "Total Time: " + str(self.timeFunction()(solution[1])[0]) + " seconds"
            )
            print("\n")
        for solution in self.solutions_outside_constraints_:
            print(
                "Failed "
                + str(solution[0])[self.optimization_function_type_length_ :].lower()
                + " optimization results"
            )
            print("------------------------------------")
            print("Optimal hardware: " + str(solution[1][0]) + " CPUs")
            print("Total Cost: $" + str(self.costFunction()(solution[1])[0]))
            print(
                "Total Time: " + str(self.timeFunction()(solution[1])[0]) + " seconds"
            )

def terminateClusters():
    benchmark_down_command = "sky down --all < benchmark_input.txt"
    benchmark_down = subprocess.Popen(
        benchmark_down_command,
        shell=True,
        stdout=subprocess.PIPE,
    )
    while True:
        line = benchmark_down.stdout.readline()
        if line:
            line_byte = line.strip()
            line = line_byte.decode("utf-8")
            print(line)
        else:
            break

    benchmark_delete_command = "sky bench delete --all < benchmark_input.txt"
    benchmark_delete = subprocess.Popen(
        benchmark_delete_command,
        shell=True,
        stdout=subprocess.PIPE,
    )
    
    while True:
        line = benchmark_delete.stdout.readline()
        if line:
            line_byte = line.strip()
            line = line_byte.decode("utf-8")
            print(line)
        else:
            break
    print("Successfully terminated clusters.")

def main():
    print("Hello World!")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-y",
        "--yaml_file",
        required=True,
        help="Path to yaml file from current directory (not the directory that the script is in)",
    )
    parser.add_argument(
        "-gy",
        "--gpu_yaml_file",
        required=True,
        help="Path to yaml file from current directory (not the directory that the script is in)",
    )
    parser.add_argument(
        "-s",
        "--steps",
        required=True,
        type=int,
        help="Total number of steps you want the algorithm to take (Dexnet by default)",
    )
    parser.add_argument(
        "-c",
        "--max_cost",
        required=True,
        type=float,
        help="Maximum financial cost you want to spend on cloud computing (in $)",
    )
    parser.add_argument(
        "-t",
        "--max_time",
        required=True,
        type=float,
        help="Maximum mount of time you want the algorithm to spend on the cloud (in seconds)",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="Type -d if you want to run debug mode",
    )
    args = parser.parse_args()
    gpu_list = ['T4']
    sky_optimization_list = []
    thread_list = []
    count = 0
    args.gpu_type = ""
    args.count = count
    sky_optimization_no_gpu = SkyOptimization(**vars(args))
    sky_optimization_no_gpu.fullOptimization()
    sky_optimization_list.append(sky_optimization_no_gpu)
    t1 = threading.Thread(
                target=sky_optimization_no_gpu.fullOptimization,
            )
    t1.start()
    thread_list.append(t1)

    for gpu_type in gpu_list:
        args.gpu_type = gpu_type
        args.count += 1
        print("ARG COUNT: " + str(args.count))
        print(args.gpu_type)
        sky_optimization_gpu = SkyOptimization(**vars(args))
        sky_optimization_list.append(sky_optimization_gpu)
        t = threading.Thread(target=sky_optimization_gpu.fullOptimization)
        t.start()
        thread_list.append(t)

    for thread in thread_list:
        thread.join()
    terminateClusters()
    print("Got eeem at the end")
    for sky_optimization in sky_optimization_list:
        print(sky_optimization.gpu_type_)
        print(sky_optimization.cost_optimization_results_)
        print(sky_optimization.optimal_cost_instance_)
        print(sky_optimization.time_optimization_results_)
        print(sky_optimization.optimal_time_instance_)
    print("Cooking")
    # Start with reading the benchmark.yaml_file file
    # Get the max, min, and middle CPU values
    # Run benchmarking these 3 values
    # Find linear function for seconds per step
    # Find these exact instances in the ~/.sky/catalogs/v5/aws/vms.csv
    # Figure out cost per CPU by looking at those instances (we could just pull from terminal, but that may not scale well)
    # Setup optimization


if __name__ == "__main__":
    main()
