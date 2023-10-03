import argparse
import os
import json
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


## @class OptimizationFunctionType
#  @brief Specifies whether the optimization problem will be solving for time or cost
#
class OptimizationFunctionType(Enum):
    ## @brief Time Optimization
    #
    TIME = 1

    ## @brief Cost Optimization
    #
    COST = 2

    ## @brief Objective Cost Optimization (Motion Planning)
    #
    OBJECTIVE_COST = 3


# TODO: Run aws configure, and sky check beforehand
## @class SkyOptimization
# @brief Sky Optimizer responsible for performing the cost and time optimizations based on problem type (Dexnet, VSLAM, Motion Planning) and GPU type (No GPU, T4, etc)
#
class SkyOptimization:
    ## @brief Sky Optimization Constructor
    # @param yaml_file YAML file specifying Sky benchmarking setup for no GPU
    # @param gpu_yaml_file YAML file specifying Sky benchmarking setup for given GPU type (Tested only on T4)
    # @param steps Number of steps the Sky benchmarking should run
    # @param max_cost User provided cost constraint
    # @param max_time User provided time constraint
    # @param max_objective_cost User provided objective cost constraint
    # @param debug Debug Mode (Lots of print statements)
    # @param gpu_type The GPU type used in the Sky benchmarking (only tested on No GPU, T4)
    # @param count Index in list of SkyOptimization Problems being run (helps coordinate the multithreading)
    #
    def __init__(
        self,
        yaml_file,
        gpu_yaml_file,
        steps,
        max_cost,
        max_time,
        max_objective_cost,
        debug,
        gpu_type,
        count,
    ):
        ## @brief Debug Mode (Lots of print statements)
        #
        self.debug_ = debug

        ## @brief Number of steps the Sky benchmarking should run
        #
        self.steps_ = steps

        ## @brief User provided cost constraint
        #
        self.max_cost_ = max_cost

        ## @brief User provided time constraint
        #
        self.max_time_ = max_time

        ## @brief User provided objective cost constraint
        #
        self.max_objective_cost_ = max_objective_cost

        ## @brief The GPU type used in the Sky benchmarking (only tested on No GPU, T4)
        #
        self.gpu_type_ = gpu_type

        ## @brief Index in list of SkyOptimization Problems being run (helps coordinate the multithreading)
        #
        self.count_ = count

        if self.debug_:
            print("Initialize sky optimization")
        # Only attempts optimization if realistic constraints are provided
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
        if self.max_objective_cost_ <= 0:
            print("Please enter a positive objective cost")
            print("Program now exiting")
            exit()

        ## @brief Number of seconds in a minute
        #
        self.SECONDS_PER_MINUTE = 60

        ## @brief Number of minutes in an hour
        #
        self.MINUTES_PER_HOUR = 60

        ## @brief Number of seconds in an hour
        #
        self.SECONDS_PER_HOUR = self.SECONDS_PER_MINUTE * self.MINUTES_PER_HOUR

        ## @brief Positive zero approximation
        #
        self.ZERO_EPSILON = 1e-10

        ## @brief YAML file specifying Sky benchmarking setup for no GPU
        #
        self.yaml_file_ = yaml_file

        ## @brief YAML file specifying Sky benchmarking setup for given GPU type (Tested only on T4)
        #
        self.gpu_yaml_file_ = gpu_yaml_file

        ## @brief Boolean indicating if the task is V-SLAM
        #
        self.is_slam_ = "slam" in self.yaml_file_

        ## @brief Boolean indicating if the task is motion planning
        #
        self.is_motion_planning_ = "mp" in self.yaml_file_

        ## @brief Boolean indicating if the task is Dexnet
        #
        self.is_dexnet_ = not self.is_slam_ and not self.is_motion_planning_

        ## @brief Index in solution string specifying optimization function type
        #
        self.optimization_function_type_length_ = 25

        ## @brief Initialzation of maximum CPU constraint
        #
        self.max_cpu_ = 0

        ## @brief Initialization of maximum memory constraint
        #
        self.max_memory_ = 0

        ## @brief List of realistic CPU setups for the cloud (similar to relevant hardware list without memory, rememant of prototype version that just looked for optimal CPU)
        #
        self.relevant_cpu_list_ = []

        ## @brief List of realistic CPU and memory setups for the cloud
        #
        self.relevant_hardware_list_ = []

        ## @brief List storing results from time benchmarking
        #
        self.benchmark_time_results_ = []

        ## @brief List of YAML files generated that will be used for Sky benchmarking
        #
        self.benchmark_yaml_list_ = []

        ## @brief Number of benchmarks performed per feature (i.e we only do benchmarking for minimum CPU amount, maximum CPU amount, and middle CPU amount)
        #
        self.benchmark_range_ = 3

        ## @brief List of different threads running Sky benchmarking
        #
        self.benchmark_threads_ = []

        ## @brief First identifier substring for cost benchmarking
        #
        self.benchmark_cost_substring_1_ = "CLUSTER"

        ## @brief Second identifier substring for cost benchmarking
        #
        self.benchmark_cost_substring_2_ = "CLOUD"

        ## @brief First identifier substring for time benchmarking
        #
        self.benchmark_time_substring_1_ = "CLUSTER"

        ## @brief Second identifier substring for time benchmarking
        #
        self.benchmark_time_substring_2_ = "RESOURCES"

        ## @brief First identifier substring for GPU identification
        #
        self.min_gpu_substring_1_ = "CLOUD"

        ## @brief Second identifier substring for GPU identification
        #
        self.min_gpu_substring_2_ = "ACCELERATORS"

        ## @brief Initialization for memory:CPU ratio (used only when GPU type isn't None)
        #
        self.mem_to_cpu_ratio_for_gpu_ = None

        ## @brief List containing step and startup time results for benchmarking (remmanat of prototype version where we tried to account for startup time.
        #
        self.num_steps_and_duration_ = []

        ## @brief Average setup time for cloud instance (remmenant of prototype version: see self.num_steps_and_duration_ for explanation)
        #
        #
        self.average_setup_time_ = 0

        ## @brief Boolean indicating if we have receieved cost information from benchmarking
        #
        self.got_cost_info_ = False

        ## @brief Indiex where CPU information is located in benchmark cost string
        #
        self.benchmark_cost_substring_cpu_index_ = 8

        ## @brief Index where Cost information is located in benchmark cost string
        #
        self.benchmark_cost_substring_cost_index_ = 11

        ## @brief List containng cost results from Sky benchmarking
        #
        self.benchmark_cost_results_ = []

        ## @brief  List containing objective cost results for Sky benchmarking (only relevant for Motion Planning)
        #
        self.benchmark_objective_cost_results_ = []

        ## @brief String containing YAML file contents for Sky benchmark YAML
        #
        self.yaml_file_contents_ = None

        ## @brief Index containing information about where CPU information starts in YAML file
        #
        self.cpu_start_index_ = -1

        ## @brief Index containing information about where CPU candidate information starts in YAML file
        #
        self.cpu_candiate_index_ = -1

        ## @brief Offset for index where we insert CPU information into the YAML file
        #
        self.cpu_str_offset_ = 5

        ## @brief Index containing seconds per step information during Sky benchmarking
        #
        self.seconds_per_step_index_ = 8

        ## @brief Index containing number of steps information during Sky benchmarking
        #
        self.num_steps_index_ = 7

        ## @brief Index containing minute duration during Sky benchmarking
        #
        self.duration_min_index_ = 4

        ## @brief Index containing second duration during Sky benchmarking
        #
        self.duration_sec_index_ = 5

        # Account for index offset when GPU is not None
        if self.gpu_type_ != "":
            self.seconds_per_step_index_ += 2
            self.duration_min_index_ += 2
            self.duration_sec_index_ += 2

        ## @brief List of solutions that are within constraints
        #
        self.solutions_in_constraints_ = []

        ## @brief List of solutions that are outside constraints
        #
        self.solutions_outside_constraints_ = []

        ## @brief String before relevant benchmarking data (Let's the program know which line to collect benchmarking data from)
        #
        self.data_file_ = "CLUSTER                RESOURCES                        STATUS   DURATION  SPENT($)  #STEPS  SEC/STEP  $/STEP    EST(hr)  EST($)"

        ## @brief Data file we will save benchmark data results
        #
        self.data_file_name_ = "benchmark_data_" + self.gpu_type_ + ".txt"

        ## @brief List containing info that will be outputted to data file
        #
        self.result_queue_ = []

        ## @brief List containing possible CPUs (current cloud setup could not do any other CPU amounts)
        #
        self.possible_cpus_ = [2, 4, 8, 16, 32, 64]

        ## @brief String stating the optimal cloud instance for cost optimization
        #
        self.optimal_cost_instance_ = "Sample"

        ## @brief String stating the optimal cloud instance for time optimization
        #
        self.optimal_time_instance_ = "Sample"

        ## @brief String stating the optimal cloud instance for objective cost optimization
        #
        self.optimal_objective_cost_instance_ = "Sample"

        ## @brief Map between hardware tuples (CPU, Memory) and T4 GPU cloud instances for AWS (Future Work: Will generate these maps online by parsing SkyPilot database CSV file)
        #
        self.t4_map_ = {
            (4, 16): "g4dn.xlarge",
            (8, 32): "g4dn.2xlarge",
            (16, 64): "g4dn.4xlarge",
        }

        ## @brief Map between hardware tuples (CPU, Memory) and no GPU cloud instance (Future Work: Will generate these maps online by parsing SkyPilot database CSV file)
        #
        self.no_gpu_map_ = {
            (2, 4): "c6i.large",
            (2, 8): "m6i.large",
            (2, 16): "r6i.large",
            (4, 8): "c6i.xlarge",
            (4, 16): "m6i.xlarge",
            (4, 32): "r6i.xlarge",
            (8, 16): "c6i.2xlarge",
            (8, 32): "m6i.2xlarge",
            (8, 64): "r6i.2xlarge",
            (16, 32): "c6i.4xlarge",
            (16, 64): "m6i.4xlarge",
            (16, 128): "r6i.4xlarge",
            (32, 64): "c6i.8xlarge",
            (32, 128): "m6i.8xlarge",
            (32, 256): "r6i.8xlarge",
            (64, 128): "c6i.16xlarge",
            (64, 256): "m6i.16xlarge",
            (64, 512): "r6i.16xlarge",
        }

        ## @brief Time Regression Output containing model info and constraints
        #
        self.time_model_info_ = None

        ## @brief Cost Regression Output containing model info and constraints
        #
        self.cost_model_info_ = None

        ## @brief Objective Cost Regression Output containing model info and constraints (only motion planning)
        #
        self.objective_cost_model_info_ = None

        ## @brief Time Regression Model
        #
        self.timeModel_ = None

        ## @brief Coefficients for Time Regression Model
        #
        self.time_model_coefficients_ = None

        ## @brief Cost Regression Model
        #
        self.costModel_ = None

        ## @brief Coefficients for Cost Regression Model
        #
        self.cost_model_coefficients_ = None

        ## @brief Objective Cost Regression Model
        #
        self.objectiveCostModel_ = None

        ## @brief Coefficients for Objective Cost Regression Model
        #
        self.objective_cost_model_coefficients_ = None

        ## @brief Scipy Output from Cost Optimization
        #
        self.cost_optimization_results_ = None

        ## @brief Scipy Output from Time Optimization
        #
        self.time_optimization_results_ = None

        ## @brief Scipy Output from Objective Cost Optimization
        #
        self.objective_cost_optimization_results_ = None

    ## @brief Full Sky Optimization Function
    # @return List containing Time and Cost Optimizations
    #
    def fullOptimization(self):
        # Setup Sky benchmarks
        if self.gpu_type_ == "":
            self.findCpuValues()
        else:
            self.findGpuValues()
        self.createBenchmarks(self.gpu_type_)
        self.runSkyBenchmarks(self.count_)

        if self.debug_:
            print("Time benchmark results")
            print(self.benchmark_time_results_)
            print("Cost benchmark results")
            print(self.benchmark_cost_results_)
            print("Objective cost benchmark results")
            print(self.benchmark_objective_cost_results_)

        # Solve for models
        self.time_model_info_ = self.regressionSolver(
            self.benchmark_time_results_, "Time"
        )
        self.cost_model_info_ = self.regressionSolver(
            self.benchmark_cost_results_, "Cost"
        )
        if self.is_motion_planning_:
            self.objective_cost_model_info_ = self.regressionSolver(
                self.benchmark_objective_cost_results_, "Objective Cost"
            )

        self.timeModel_ = self.time_model_info_[0]
        self.time_model_coefficients_ = self.time_model_info_[1]
        if self.debug_:
            print("Time Model")
            print(self.timeModel_)
            print("Time Model Coefficients")
            print(self.time_model_coefficients_)
        self.costModel_ = self.cost_model_info_[0]
        self.cost_model_coefficients_ = self.cost_model_info_[1]
        if self.debug_:
            print("Cost Model")
            print(self.costModel_)
            print("Cost Model Coefficients")
            print(self.cost_model_coefficients_)
        if self.is_motion_planning_:
            self.objectiveCostModel_ = self.objective_cost_model_info_[0]
            self.objective_cost_model_coefficients_ = self.objective_cost_model_info_[1]
            if self.debug_:
                print("Objective Cost Model")
                print(self.objectiveCostModel_)
                print("Objective Cost Model Coefficients")
            print(self.objective_cost_model_coefficients_)
        self.result_queue_.append(str(self.timeModel_))
        self.result_queue_.append(str(self.time_model_coefficients_))
        self.result_queue_.append(str(self.costModel_))
        self.result_queue_.append(str(self.cost_model_coefficients_))
        self.result_queue_.append(str(self.objectiveCostModel_))
        self.result_queue_.append(str(self.objective_cost_model_coefficients_))
        # Perform optimization
        if self.is_motion_planning_:
            self.cost_optimization_results_ = self.solveOptimization(
                OptimizationFunctionType.COST, OptimizationFunctionType.OBJECTIVE_COST
            )
            self.objective_cost_optimization_results_ = self.solveOptimization(
                OptimizationFunctionType.OBJECTIVE_COST, OptimizationFunctionType.COST
            )
        else:
            self.cost_optimization_results_ = self.solveOptimization(
                OptimizationFunctionType.COST, OptimizationFunctionType.TIME
            )
            self.time_optimization_results_ = self.solveOptimization(
                OptimizationFunctionType.TIME, OptimizationFunctionType.COST
            )
        self.findInstance(self.cost_optimization_results_, OptimizationFunctionType.COST)
        self.result_queue_.append(str(self.cost_optimization_results_))
        self.result_queue_.append(str(self.optimal_cost_instance_))
        if self.is_motion_planning_:
            self.findInstance(self.objective_cost_optimization_results_,OptimizationFunctionType.OBJECTIVE_COST)
            self.result_queue_.append(str(self.objective_cost_optimization_results_))
            self.result_queue_.append(str(self.optimal_objective_cost_instance_))
        else:
            self.findInstance(self.time_optimization_results_, OptimizationFunctionType.TIME)
            self.result_queue_.append(str(self.time_optimization_results_))
            self.result_queue_.append(str(self.optimal_time_instance_))

        # Write data to file
        for line in self.result_queue_:
            self.data_file_ += "\n" + line
        with open(self.data_file_name_, "w") as file:
            file.write(self.data_file_)
        if(self.is_motion_planning_):
            return [self.cost_optimization_results_, self.objective_cost_optimization_results_]
        else:
            return [self.cost_optimization_results_, self.time_optimization_results_]

    ## @brief Parses Sky benchmarking to get objective cost for given machine instance
    # @param machine_name Name of benchmarking machine instance
    # @return Objective cost from instance
    #
    def getObjectiveCost(self, machine_name):  # sky-bench-apartment-full-0
        # benchmark_name = "-".join(machine_name.strip("sky-bench-").split("-")[:-1])
        benchmark_name = machine_name[10:]
        # expand_path =
        path = (
            os.path.expanduser("~")
            + "/.sky/benchmarks/"
            + benchmark_name
            + "/"
            + machine_name
            + "/"
        )  # + "/summary.json"
        # find the directory under path that start with sky-callback
        objective_cost = None
        for callback in os.listdir(path):
            if callback.startswith("sky-callback"):
                with open(path + callback + "/summary.json", "r") as f:
                    objective_cost = str(json.loads(f.read())["objective_cost"])
        if self.debug_:
            print("Objective cost method: " + str(objective_cost))
        return float(objective_cost)

    ## @brief Given results of optimization, this will find the closest feasible hardware setup and its corresponding AWS Instance (Future Work: Build out to other cloud providers)
    # @param optimization_results Scipy Output from optimization
    # @param optimization_type Optimization Type (Cost or Time)
    #
    def findInstance(self, optimization_results, optimization_type):
        if self.debug_:
            print("Start find instance")
            print("Optimization Results")
            print(optimization_results)

        # Extract optimization results
        optimal_cpu_count = optimization_results[1][0]
        optimal_memory_count = None
        if self.gpu_type_ == "":
            optimal_memory_count = optimization_results[1][1]

        if self.debug_:
            print("Optimal CPU Count")
            print(optimal_cpu_count)
            print("Optimal Memory Count")
            print(optimal_memory_count)

        # Find nearest feasible hardware
        rounded_cpu_count = self.binarySearchRounding(optimal_cpu_count)
        rounded_memory_count = None
        if self.gpu_type_ == "":
            potential_memory_amounts = [
                rounded_cpu_count * 2,
                rounded_cpu_count * 4,
                rounded_cpu_count * 8,
            ]
            min_diff = 1000
            for potential_memory_amount in potential_memory_amounts:
                if abs(potential_memory_amount - optimal_memory_count) < min_diff:
                    min_diff = abs(potential_memory_amount - optimal_memory_count)
                    rounded_memory_count = potential_memory_amount
        else:
            rounded_memory_count = self.mem_to_cpu_ratio_for_gpu_ * rounded_cpu_count
        optimal_hardware = (rounded_cpu_count, rounded_memory_count)
        optimal_instance = None

        # Map to AWS Instance (Needs to be expanded for more cloud cases)
        if self.gpu_type_ == "":
            optimal_instance = self.no_gpu_map_[optimal_hardware]
        elif self.gpu_type_ == "T4":
            optimal_instance = self.t4_map_[optimal_hardware]
        else:
            print("Need to implement " + self.gpu_type_ + " Instance Rounding!!!")
            exit()
        if optimization_type == OptimizationFunctionType.COST:
            self.optimal_cost_instance_ = optimal_instance
        elif optimization_type == OptimizationFunctionType.TIME:
            self.optimal_time_instance_ = optimal_instance
        elif optimization_type == OptimizationFunctionType.OBJECTIVE_COST:
            self.optimal_objective_cost_instance_ = optimal_instance
        else:
            print(optimization_type + "is not a valid optimization type")

    ## @brief Rounds CPU output of optimization to nearest feasible CPU amount allowed in cloud instance
    # @param optimal_cpu_count CPU output from optimization
    # @return Feasible CPU amount closest to the optimal CPU amount
    #
    def binarySearchRounding(self, optimal_cpu_count):
        # Binary search algo implementation
        low = 0
        high = len(self.possible_cpus_) - 1
        while low <= high:
            mid = (low + high) // 2
            if self.possible_cpus_[mid] == optimal_cpu_count:
                return self.possible_cpus_[mid]
            if self.possible_cpus_[mid] < optimal_cpu_count:
                low = mid + 1
            else:
                high = mid - 1
        if low == 0:
            return self.possible_cpus_[0]
        if high == len(self.possible_cpus_) - 1:
            return self.possible_cpus_[-1]
        closest_left = self.possible_cpus_[high]
        closest_right = self.possible_cpus_[low]
        if abs(closest_left - optimal_cpu_count) <= abs(
            closest_right - optimal_cpu_count
        ):
            return closest_left
        else:
            return closest_right

    ## @brief Parses provided GPU benchmark file to determine possible hardware setups for given GPU
    #
    def findGpuValues(self):
        if self.debug_:
            print("Finding relevant GPU values for fitting")
        cpu_start_string = "    candidates:\n"
        if self.debug_:
            print("Current directory")
            print(os.getcwd())

        # Find GPU file
        cwd = os.getcwd()
        self.gpu_yaml_file_ = cwd + "/" + self.gpu_yaml_file_
        if self.debug_:
            print("GPU File Path")
            print(self.gpu_yaml_file_)
        self.yaml_file_contents_ = None
        try:
            with open(self.gpu_yaml_file_, "r") as file:
                self.yaml_file_contents_ = file.readlines()
        except:
            print("Invalid file path for GPU benchmark YAML")
            exit()

        if self.debug_:
            print("YAML File Contents")
            print(self.yaml_file_contents_)

        # Configures Benchmark GPU Yaml to use minimum possible CPU and memory to establish minimum
        cpu_index = self.yaml_file_contents_.index(cpu_start_string) + 1
        self.cpu_start_index_ = cpu_index
        cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
        cpu_substring = cpu_string[
            cpu_string.find("cpus:") + self.cpu_str_offset_ : cpu_string.find("}")
        ]
        cpu_replacement_substring = " 1+, memory: 1+, accelerators: " + self.gpu_type_
        new_cpu_string = cpu_string.replace(cpu_substring, cpu_replacement_substring)
        self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string

        # Writes minimum GPU Yaml file
        filename = "min_" + self.gpu_type_ + ".yaml"
        f = open(filename, "w")
        file_contents = ""
        for line in self.yaml_file_contents_:
            file_contents += line
        f.write(file_contents)
        f.close()

        # Runs Sky benchmarking on minimum GPU Yaml
        # Purpose: We don't actually want to execute the benchmarking, but in the setup, it will tell us the minimum CPU and memory amount (Not just 1+)
        gpu_benchmark_setup_command = "bash gpu_benchmark.sh " + filename + " gpucheck"
        benchmark_setup = subprocess.Popen(
            gpu_benchmark_setup_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        gpu_headers = None
        gpu_values = None
        found_line = False
        gpu_line_gap = 3
        while True:
            line = benchmark_setup.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                if (
                    self.min_gpu_substring_1_ in line
                    and self.min_gpu_substring_2_ in line
                ):
                    gpu_headers = line.split()
                    print(gpu_headers)
                    found_line = True
                if found_line:
                    gpu_line_gap -= 1
                if gpu_line_gap == 0:
                    gpu_values = line.split()
                    if self.debug_:
                        print("String List from Sky benchmarking")
                        print(gpu_values)

                    # Finds minimum, middle, and maximum CPU values
                    min_cpu_index = gpu_headers.index("vCPUs") - 1
                    min_mem_index = gpu_headers.index("Mem(GB)") - 1
                    cpu_min = float(gpu_values[min_cpu_index])
                    mem_min = float(gpu_values[min_mem_index])
                    self.mem_to_cpu_ratio_for_gpu_ = mem_min / cpu_min

                    # We are limited to a max of 16 CPUs if we are using GPU for our current AWS setup
                    cpu_max = 16
                    log_cpu_min = math.log2(cpu_min)
                    log_cpu_max = math.log2(cpu_max)
                    log_cpu_mid = int((log_cpu_min + log_cpu_max) / 2)
                    cpu_mid = 2**log_cpu_mid
                    self.relevant_cpu_list_.append(cpu_min)
                    self.relevant_cpu_list_.append(cpu_mid)
                    self.relevant_cpu_list_.append(cpu_max)

            else:
                break

    ## @brief Parses provided CPU benchmark file to determine edge case hardware setups
    #
    def findCpuValues(self):
        if self.debug_:
            print("Finding relevant CPU values for fitting")
        cpu_start_string = "    candidates:\n"
        if self.debug_:
            print("Current directory")
            print(os.getcwd())

        # Parses CPU benchmarking file
        cwd = os.getcwd()
        self.yaml_file_ = cwd + "/" + self.yaml_file_
        if self.debug_:
            print("Directory of Yaml File")
            print(self.yaml_file_)

        self.yaml_file_contents_ = None
        try:
            with open(self.yaml_file_, "r") as file:
                self.yaml_file_contents_ = file.readlines()
        except:
            print("Invalid file path for CPU benchmark YAML")
            exit()
        print(self.yaml_file_contents_)
        cpu_index = self.yaml_file_contents_.index(cpu_start_string) + 1
        self.cpu_start_index_ = cpu_index
        cpu_list = []
        yaml_parsed = False
        self.cpu_str_offset_ = 5

        # Obtains possible CPU values listed in Yaml
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

        # Finds minimum, middle, and maximum CPU counts
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

    ## @brief Creates Yaml files for each benchmarking instance that will be run
    # @param gpu_type The GPU type of this optimization instance (only tested on T4 and No GPU)
    def createBenchmarks(self, gpu_type):
        if self.debug_:
            print("Creating benchmarks")
        if gpu_type == "":
            # When we use a GPU, the memory to CPU ratio is not fixed, so we setup instances that vary CPU and memory with each other
            for i in range(self.benchmark_range_):
                cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
                cpu_substring = cpu_string[
                    cpu_string.find("cpus:")
                    + self.cpu_str_offset_ : cpu_string.find("}")
                ]
                for j in range(self.benchmark_range_):
                    memory = self.relevant_cpu_list_[i] * (2 ** (j + 1))
                    cpu_replacement_substring = (
                        " "
                        + str(self.relevant_cpu_list_[i])
                        + ", memory: "
                        + str(memory)
                    )
                    new_cpu_string = cpu_string.replace(
                        cpu_substring, cpu_replacement_substring
                    )
                    self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
                    filename = "cpu_" + str(i) + "_mem" + str(j) + ".yaml"
                    self.benchmark_yaml_list_.append(filename)
                    self.relevant_hardware_list_.append(
                        (self.relevant_cpu_list_[i], memory)
                    )
                    f = open(filename, "w")
                    file_contents = ""
                    for line in self.yaml_file_contents_:
                        file_contents += line
                    f.write(file_contents)
                    f.close()
        else:
            # When we use a GPU, the memory to CPU ratio is fixed
            for cpu in self.relevant_cpu_list_:
                memory = self.mem_to_cpu_ratio_for_gpu_ * cpu
                cpu_string = self.yaml_file_contents_[self.cpu_start_index_]
                cpu_substring = cpu_string[
                    cpu_string.find("cpus:")
                    + self.cpu_str_offset_ : cpu_string.find("}")
                ]
                print(cpu_substring)
                cpu_replacement_substring = (
                    " "
                    + str(int(cpu))
                    + ", memory: "
                    + str(int(memory))
                    + ", accelerators: "
                    + gpu_type
                )
                print(cpu_replacement_substring)
                new_cpu_string = cpu_string.replace(
                    cpu_substring, cpu_replacement_substring
                )
                self.yaml_file_contents_[self.cpu_start_index_] = new_cpu_string
                filename = (
                    "cpu_"
                    + str(int(cpu))
                    + "_mem"
                    + str(int(memory))
                    + "_accel_"
                    + gpu_type
                    + ".yaml"
                )
                self.benchmark_yaml_list_.append(filename)
                if self.gpu_type_ == "":
                    self.relevant_hardware_list_.append((cpu, memory))
                else:
                    self.relevant_hardware_list_.append(cpu)
                f = open(filename, "w")
                file_contents = ""
                for line in self.yaml_file_contents_:
                    file_contents += line
                f.write(file_contents)
                f.close()

    ## @brief Multithreads (Python-style) sky benchmarking instances
    # @param count Global count relative to other SkyOptimization instances to make sure each Sky benchmarking instance name is unique
    #
    def runSkyBenchmarks(self, count):
        if self.debug_:
            print("Run Sky Benchmarks")
        thread_list = []
        if(self.is_slam_):
            for i in range(0,len(self.relevant_hardware_list_),3):
                for j in range(3):
                    slam_i = i + j
                    benchmark_name = "benchmark" + str(count * 9 + slam_i)
                    print(benchmark_name)
                    t = threading.Thread(
                        target=self.skyBenchmark,
                        args=(
                            self.benchmark_yaml_list_[slam_i],
                            benchmark_name,
                            self.relevant_hardware_list_[slam_i],
                        ),
                    )
                    t.start()
                    thread_list.append(t)
                for i in range(len(thread_list)):
                    thread_list[i].join()
                terminateClusters()
        else:
            for i in range(len(self.relevant_hardware_list_)):
                benchmark_name = "benchmark" + str(count * 9 + i)
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
            for i in range(len(thread_list)):
                thread_list[i].join()

        if self.debug_:
            print("Successfully received data clusters.")

    ## @brief Runs sky benchmarking instance from bash script
    # @param yaml_file Yaml configuration file for benchmarking instance
    # @param benchmark_name Name of Sky benchmarking instance
    # @param hardware_count Tuple containing (CPU, Memory)
    #
    def skyBenchmark(self, yaml_file, benchmark_name, hardware_count):
        if self.debug_:
            print("Sky benchmark: " + yaml_file)

        # Runs bash script and outputs content to terminal for parsing
        benchmark_setup_command = (
            "bash benchmark.sh " + yaml_file + " " + benchmark_name
        )
        if self.is_motion_planning_:
            benchmark_setup_command = (
                "bash benchmark_apartment.sh " + yaml_file + " " + benchmark_name
            )
        benchmark_setup = subprocess.Popen(
            benchmark_setup_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        num_lines_to_cost_data = 2
        while True:
            line = benchmark_setup.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                line_output = "Hardware count " + str(hardware_count) + ": " + line
                print(line_output)

                # For no GPU, we obtain cost from the initial $/hr outputted by the benchmarking
                # It is much harder to parse this for GPU instances, so we wait to parse $/step
                # Overall, it amounts to the same thing either way
                if self.gpu_type_ == "":
                    if self.got_cost_info_:
                        if num_lines_to_cost_data == 0:
                            num_lines_to_cost_data = 2
                            self.got_cost_info_ = False
                            line_array = line.split()
                            print("DEBUG COST RESULTS")
                            print(line_array)
                            print(self.benchmark_cost_substring_cost_index_)
                            self.benchmark_cost_results_.append(
                                (
                                    hardware_count,
                                    float(
                                        line_array[
                                            self.benchmark_cost_substring_cost_index_
                                        ]
                                    )
                                    / self.SECONDS_PER_HOUR,
                                )
                            )
                        else:
                            num_lines_to_cost_data -= 1
                    if (
                        self.benchmark_cost_substring_1_ in line
                        and self.benchmark_cost_substring_2_ in line
                    ):
                        self.got_cost_info_ = True
                        num_lines_to_cost_data -= 1
            else:
                break
        if self.debug_:
            print("Hardware count " + str(hardware_count) + ": " + "Cluster is setup")

        # Runs Benchmarking for 10 minutes unless it is SLAM, which default runs for 1 second
        time_remaining = 10
        if self.is_slam_:
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

        # Shows output of Sky benchmarking, we want to extract the line containing the seconds/step and $/step information
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
        if self.debug_:
            print("Relevant benchmarking line")
            print(seconds_per_step_line)
        self.result_queue_.append(seconds_per_step_line)
        seconds_per_step_array = seconds_per_step_line.split()
        # Odd exception where if duration is exactly 5 minutes and 0 seconds, everything after is shifted by 1
        seconds_per_step = None
        dollars_per_second = None
        duration = 0
        num_steps = 0
        if self.debug_:
            print("Seconds per line array parsing")
            print(seconds_per_step_array)
            print(self.seconds_per_step_index_)
            print(self.seconds_per_step_index_ - 3)
            print(seconds_per_step_array[self.seconds_per_step_index_ - 3])

        # Find seconds per step, number of steps, dollars per step, and dollars per second
        if "s" not in seconds_per_step_array[self.seconds_per_step_index_ - 3]:
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_ - 1]
            num_steps = int(seconds_per_step_array[self.seconds_per_step_index_ - 2])
            if self.debug_:
                print("Seconds per step")
                print(seconds_per_step)
                print("Number of steps")
                print(num_steps)
            if self.gpu_type_ != "":
                dollars_per_step = seconds_per_step_array[self.seconds_per_step_index_]
                dollars_per_second = float(dollars_per_step) / float(seconds_per_step)
                if self.debug_:
                    print("Dollars per step")
                    print(dollars_per_step)
                    print("Dollars per second")
                    print(dollars_per_second)
        else:
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_]
            num_steps = int(seconds_per_step_array[self.seconds_per_step_index_ - 1])
            if self.debug_:
                print("Seconds per step")
                print(seconds_per_step)
                print("Number of steps")
                print(num_steps)
            if self.gpu_type_ != "":
                dollars_per_step = seconds_per_step_array[
                    self.seconds_per_step_index_ + 1
                ]
                dollars_per_second = float(dollars_per_step) / float(seconds_per_step)
                if self.debug_:
                    print("Dollars per step")
                    print(dollars_per_step)
                    print("Dollars per second")
                    print(dollars_per_second)

        # Creates benchmarking maps for time, cost and objective cost
        self.benchmark_time_results_.append((hardware_count, float(seconds_per_step)))
        if self.is_motion_planning_:
            objective_cost = self.getObjectiveCost(seconds_per_step_array[0])
            self.benchmark_objective_cost_results_.append(
                (hardware_count, objective_cost)
            )
        if self.gpu_type_ != "":
            self.benchmark_cost_results_.append((hardware_count, dollars_per_second))

    # Previous version, we tried to model setup time as function of hardware, but they weren't related
    # It was more dependent on cloud server availablity
    # def findSetupTime(self):
    #     if self.debug_:
    #         print("Finding the average setup time to get everything on the cloud")
    #         print(self.num_steps_and_duration_)
    #     setup_times = []
    #     for item in self.num_steps_and_duration_:
    #         (num_steps, duration, hardware_count, seconds_per_step) = item
    #         setup_time = duration - (seconds_per_step * num_steps)
    #         setup_times.append(setup_time)
    #     self.average_setup_time_ = sum(setup_times) / len(setup_times)
    #     if self.debug_:
    #         print("Setup times: " + str(setup_times))
    #         print("Average setup time: " + str(self.average_setup_time_))

    ## @brief Finds best fitting model given benchmarking data
    # @param data Benchmarking data for either time, cost, or objective cost
    # @param purpose Time, Cost, or Objective Cost
    # @return Best fit model and its coefficients
    #
    def regressionSolver(self, data, purpose):
        if self.debug_:
            print(
                "Running least squares regression to solve for "
                + purpose
                + " per hardware"
            )
        if self.debug_:
            print("Data")
            print(data)
        data_np = np.array(data)
        if self.debug_:
            print("NumpyData")
            print(data_np)
        x = data_np[:, 0]
        y = data_np[:, 1]
        x_np = None
        y_np = None
        # Parsing to make sure all data is correct shape
        if type(x[0]) is tuple:
            x_np = np.array([list(item) for item in x])

        else:
            x_np = np.array(x)

        if type(y[0]) is tuple:
            y_np = np.array([list(item) for item in y])
        else:
            y_np = np.array(y)

        if self.debug_:
            print("Input data: " + str(x_np))
            print("Output data: " + str(y_np))
        if self.gpu_type_ != "":
            potential_model_object = GPUModel(x_np, y_np)
            potential_model = potential_model_object.getModel()
            if self.debug_:
                print("Model for " + str(purpose) + ": " + str(potential_model))
            return potential_model
        else:
            potential_model_object = Model(x_np, y_np)
            potential_model = potential_model_object.getModel()
            if self.debug_:
                print("Model for " + str(purpose) + ": " + str(potential_model))
            return potential_model

    ## @brief Bounds for CPU and Memory for optimization problem
    # @return SciPy Bounds object for optimization problem
    #
    def createBounds(self):
        if self.debug_:
            print("Creating bounds for optimization problem")
        low_cpu = self.relevant_cpu_list_[0]
        high_cpu = self.relevant_cpu_list_[2]
        low_mem = self.relevant_cpu_list_[0] * 2
        high_mem = self.max_memory_
        if self.gpu_type_ != "":
            low_mem = self.relevant_cpu_list_[0] * self.mem_to_cpu_ratio_for_gpu_
            high_mem = self.relevant_cpu_list_[2] * self.mem_to_cpu_ratio_for_gpu_
        if self.debug_:
            print("Low CPU bound: " + str(low_cpu))
            print("High CPU bound: " + str(high_cpu))
            print("Low Memory bound: " + str(low_mem))
            print("High Memory bound: " + str(high_mem))
        if self.gpu_type_ == "":
            return Bounds(
                [low_cpu, low_mem], [high_cpu, high_mem]
            )  # [(low_cpu, high_cpu)]
        else:
            return Bounds([low_cpu], [high_cpu])

    ## @brief Constraints for Optimization Problem (includes time/cost along with memory/CPU ratio)
    # @param constraint_function_type Constraint function type, which is opposite of the optimization problem (Time Optimization -> Cost Constraint and Cost Optimization -> Time Constraint)
    # @return SciPy Constraints object for optimization problem
    #
    def createConstraints(self, constraint_function_type):
        if self.debug_:
            print("Creating constraints for optimization problem")
        constraints = []
        mem_cpu_ratio_lower_bound = 2
        mem_cpu_ratio_upper_bound = 8
        memory_constraint = None
        if self.gpu_type_ == "":
            memory_constraint = NonlinearConstraint(
                self.memoryFunction(),
                mem_cpu_ratio_lower_bound,
                mem_cpu_ratio_upper_bound,
            )
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
            case OptimizationFunctionType.OBJECTIVE_COST:
                objective_cost_lower_bound = 0
                objective_cost_upper_bound = self.max_objective_cost_
                objective_cost_constraint = NonlinearConstraint(
                    self.objectiveCostFunction(),
                    objective_cost_lower_bound,
                    objective_cost_upper_bound,
                )
                constraints.append(objective_cost_constraint)
                return constraints

    ## @brief Gets Time Optimization Results
    # @return Time Optimization Results
    #
    def getTimeOptimization(self):
        return self.time_optimization_results_

    ## @brief Gets Cost Optimization Results
    # @return Cost Optimization Results
    #
    def getCostOptimization(self):
        return self.cost_optimization_results_

    ## @brief Optimization Loss Function
    # @param objective_function_type Time or Cost
    # @return Optimization Loss Function
    #
    def createObjectiveFunction(self, objective_function_type):
        if self.debug_:
            print("Creating objective function for optimization problem")
        match (objective_function_type):
            case OptimizationFunctionType.COST:
                return self.costFunction()
            case OptimizationFunctionType.TIME:
                return self.timeFunction()
            case OptimizationFunctionType.OBJECTIVE_COST:
                return self.objectiveCostFunction()

    ## @brief Function for memory/CPU ratio constraint
    # @return Memory:CPU Constraint Function
    #
    def memoryFunction(self):
        def memFn(x):
            return x[1] / x[0]

        return memFn

    ## @brief Cost Function = $/second * seconds/step * # steps
    # @return Function object for the Cost Function
    #
    def costFunction(self):
        def costFn(x):
            seconds_per_step = self.timeModel_(
                x, self.time_model_coefficients_, predict=False, single=True
            )
            total_time_in_seconds = (
                seconds_per_step * self.steps_
            )  # + self.average_setup_time_
            dollars_per_second = self.costModel_(
                x, self.cost_model_coefficients_, predict=False, single=True
            )
            cost = dollars_per_second * total_time_in_seconds
            return cost

        return costFn

    ## @brief Time Function = seconds/step * # steps
    # @return Function object for the Time Function
    #
    def timeFunction(self):
        def timeFn(x):
            seconds_per_step = self.timeModel_(
                x, self.time_model_coefficients_, predict=False, single=True
            )
            total_time_in_seconds = (
                seconds_per_step * self.steps_
            ) + self.average_setup_time_
            return total_time_in_seconds

        return timeFn

    ## @brief Objective Cost Function
    # @return Function object for the Objective Cost Function
    #
    def objectiveCostFunction(self):
        def objectiveCostFn(x):
            objective_cost = self.objectiveCostModel_(
                x, self.objective_cost_model_coefficients_, predict=False, single=True
            )
            return objective_cost

        return objectiveCostFn

    ## @brief Optimization Problem Setup and Solver
    # @param objective_function_type Time or Cost
    # @param constraint_function_type Opposite of objective_function_type
    # @return Optimization Result, Time Model Result, Cost Model Result
    #
    def solveOptimization(self, objective_function_type, constraint_function_type):
        if objective_function_type == constraint_function_type:
            print("Constraint function can't be objective function")
            print("Program now exiting")
            exit()

        # Optimization Problem Setup
        bounds = self.createBounds()
        constraints = self.createConstraints(constraint_function_type)
        objective_function = self.createObjectiveFunction(objective_function_type)
        initial_guess = np.array([1, 1])

        # Ensures initial guess is reasonable
        if self.gpu_type_ != "":
            initial_guess = np.array([self.relevant_cpu_list_[0]])

        # Solve Optimization Problem
        optimization_method = "SLSQP"
        optimization_result = minimize(
            objective_function,
            initial_guess,
            method=optimization_method,
            bounds=bounds,
            constraints=constraints,
            tol=1e-5,
            options={"maxiter": 10000},
        )
        match objective_function_type:
            case OptimizationFunctionType.COST:
                print("Obtained Cost Optimization Result")
            case OptimizationFunctionType.TIME:
                print("Obtained Time Optimization Result")
            case OptimizationFunctionType.OBJECTIVE_COST:
                print("Obtained Objective Cost Optimization Result")
        if self.debug_:
            print("Optimization Result")
            print(optimization_result)
            if self.is_motion_planning_:
                print(
                    "Objective Cost: "
                    + str(self.objectiveCostFunction()(optimization_result.x))
                )
            else:
                print("Time: " + str(self.timeFunction()(optimization_result.x)))
            print("Cost: " + str(self.costFunction()(optimization_result.x)))
        if self.is_motion_planning_:
            return (
                optimization_result.success,
                optimization_result.x,
                self.objectiveCostFunction()(optimization_result.x),
                self.costFunction()(optimization_result.x),
            )
        else:
            return (
                optimization_result.success,
                optimization_result.x,
                self.timeFunction()(optimization_result.x),
                self.costFunction()(optimization_result.x),
            )


## @brief Terminates all Sky benchmarking instances
#
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


## @brief Main Method containing user interface command parser
#
def main():
    # Terminates all clusters before starting benchmarking
    terminateClusters()
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
        "-oc",
        "--max_objective_cost",
        required=True,
        type=float,
        help="Maximum mount of time you want the algorithm to spend on the cloud (in seconds) (only relevant for motion planning)",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="Type -d if you want to run debug mode",
    )
    args = parser.parse_args()
    gpu_list = ["T4"]
    sky_optimization_list = []
    thread_list = []
    count = 0
    args.gpu_type = ""
    args.count = count

    # Initializes No GPU Sky Optimization
    sky_optimization_no_gpu = SkyOptimization(**vars(args))
    sky_optimization_list.append(sky_optimization_no_gpu)
    t1 = threading.Thread(
        target=sky_optimization_no_gpu.fullOptimization,
    )
    t1.start()
    thread_list.append(t1)

    # Loops through all GPU types and creates SkyOptimization instances for them
    if (
        not sky_optimization_no_gpu.is_slam_
        and not sky_optimization_no_gpu.is_motion_planning_
    ):
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
    # Prints results
    for sky_optimization in sky_optimization_list:
        print(sky_optimization.gpu_type_)
        if(sky_optimization.cost_optimization_results_[0] and sky_optimization.cost_optimization_results_[3] <= sky_optimization.max_cost_):
            if(sky_optimization.is_motion_planning_):
                print("Successful optimization")
                print("Optimized CPU: " + str(sky_optimization.cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Optimized Memory: " + str(sky_optimization.cost_optimization_results_[1][1]))
                else:
                    print("Optimized Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.cost_optimization_results_[1][0]))
                print("Projected objective cost: " + str(sky_optimization.cost_optimization_results_[2]))
                print("Projected cost: $" + str(sky_optimization.cost_optimization_results_[3]))
                print(sky_optimization.optimal_cost_instance_)
            else:
                print("Successful optimization")
                print("Optimized CPU: " + str(sky_optimization.cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Optimized Memory: " + str(sky_optimization.cost_optimization_results_[1][1]))
                else:
                    print("Optimized Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.cost_optimization_results_[1][0]))
                print("Projected time: " + str(sky_optimization.cost_optimization_results_[2]) + " seconds")
                print("Projected cost: $" + str(sky_optimization.cost_optimization_results_[3]))
                print(sky_optimization.optimal_cost_instance_)
        else:
            if(sky_optimization.is_motion_planning_):
                print("Failed optimization. Please relax constraints")
                print("Closest CPU: " + str(sky_optimization.cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Closest Memory: " + str(sky_optimization.cost_optimization_results_[1][1]))
                else:
                    print("Closest Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.cost_optimization_results_[1][0]))
                print("Projected objective cost: " + str(sky_optimization.cost_optimization_results_[2]))
                print("Projected cost: $" + str(sky_optimization.cost_optimization_results_[3]))
            else:
                print("Failed optimization. Please relax constraints")
                print("Closest CPU: " + str(sky_optimization.cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Closest Memory: " + str(sky_optimization.cost_optimization_results_[1][1]))
                else:
                    print("Closest Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.cost_optimization_results_[1][0]))
                print("Projected time: " + str(sky_optimization.cost_optimization_results_[2]) + " seconds")
                print("Projected cost: $" + str(sky_optimization.cost_optimization_results_[3]))
            
            print(sky_optimization.optimal_cost_instance_)
        if(sky_optimization.is_motion_planning_):
            if(sky_optimization.objective_cost_optimization_results_[0] and sky_optimization.objective_cost_optimization_results_[2] <= sky_optimization.max_objective_cost_):
                print("Successful optimization")
                print("Optimized CPU: " + str(sky_optimization.objective_cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Optimized Memory: " + str(sky_optimization.objective_cost_optimization_results_[1][1]))
                else:
                    print("Optimized Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.objective_cost_optimization_results_[1][0]))
                print("Projected objective cost: " + str(sky_optimization.objective_cost_optimization_results_[2]))
                print("Projected cost: $" + str(sky_optimization.objective_cost_optimization_results_[3]))
                print(sky_optimization.optimal_objective_cost_instance_)
            else:
                print("Failed optimization. Please relax constraints")
                print("Closest CPU: " + str(sky_optimization.objective_cost_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Closest Memory: " + str(sky_optimization.objective_cost_optimization_results_[1][1]))
                else:
                    print("Closest Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.objective_cost_optimization_results_[1][0]))
                print("Projected objective cost: " + str(sky_optimization.objective_cost_optimization_results_[2]))
                print("Projected cost: $" + str(sky_optimization.objective_cost_optimization_results_[3]))
                print(sky_optimization.optimal_objective_cost_instance_)
        else:
            if(sky_optimization.time_optimization_results_[0] and sky_optimization.time_optimization_results_[2] <= sky_optimization.max_time_):
                print("Successful optimization")
                print("Optimized CPU: " + str(sky_optimization.time_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Optimized Memory: " + str(sky_optimization.time_optimization_results_[1][1]))
                else:
                    print("Optimized Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.time_optimization_results_[1][0]))
                print("Projected time: " + str(sky_optimization.time_optimization_results_[2]) + " seconds")
                print("Projected cost: $" + str(sky_optimization.time_optimization_results_[3]))
                print(sky_optimization.optimal_time_instance_)
            else:
                print("Failed optimization. Please relax constraints")
                print("Closest CPU: " + str(sky_optimization.time_optimization_results_[1][0]))
                if(sky_optimization.gpu_type_ == ""):
                    print("Closest Memory: " + str(sky_optimization.time_optimization_results_[1][1]))
                else:
                    print("Closest Memory: " + str(sky_optimization.mem_to_cpu_ratio_for_gpu_ * sky_optimization.time_optimization_results_[1][0]))
                print("Projected time: " + str(sky_optimization.time_optimization_results_[2]) + " seconds")
                print("Projected cost: $" + str(sky_optimization.time_optimization_results_[3]))
                print(sky_optimization.optimal_time_instance_)


if __name__ == "__main__":
    main()
