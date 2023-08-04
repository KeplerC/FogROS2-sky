import argparse
import os
import yaml
import csv
import threading
import subprocess
import time
import numpy as np
from enum import Enum
from scipy.optimize import NonlinearConstraint, minimize

class ModelType(Enum):
    LINEAR = 1
    POWER = 2
    EXPONENTIAL = 3
    LOGARITHMIC = 4
    QUADRATIC = 5

class OptimizationFunctionType(Enum):
    TIME = 1
    COST = 2

# Run aws configure, and sky check beforehand
class SkyOptimization():
    def __init__(self,yaml_file,steps,max_cost,max_time,debug):
        self.debug_ = debug
        self.steps_ = steps
        self.max_cost_ = max_cost
        self.max_time_ = max_time
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
        self.timeModel = None
        self.costModel = None
        self.relevant_cpu_list_ = []
        self.benchmark_time_results_ = []
        self.cpu_benchmark_yaml_list_ = ['benchmark_cpu_min.yaml','benchmark_cpu_mid.yaml','benchmark_cpu_max.yaml']
        self.benchmark_name_list_ = ['cpu-min','cpu-mid','cpu-max']
        self.benchmark_threads_ = []
        self.benchmark_cost_substring_1_ = "CLUSTER"
        self.benchmark_cost_substring_2_ = "CLOUD"
        self.benchmark_time_substring_1_ = "CLUSTER"
        self.benchmark_time_substring_2_ = "RESOURCES"
        self.got_cost_info_ = False
        self.benchmark_cost_substring_cpu_index_ = 8
        self.benchmark_cost_substring_cost_index_ = 11
        self.benchmark_cost_results_ = []
        self.yaml_file_contents_ = None
        self.cpu_start_index_ = -1
        self.cpu_candiate_index_ = -1
        self.cpu_str_offset_ = 5
        self.seconds_per_step_index_ = 8
        self.time_constraint_coefficients_ = None
        self.time_constraint_model_ = None
        self.findCpuValues()
        self.createBenchmarks()
        self.runSkyBenchmarks()
        self.time_model_info_ = self.regressionSolver(self.benchmark_time_results_,'Time')
        self.cost_model_info_ = self.regressionSolver(self.benchmark_cost_results_,'Cost')
        self.bounds_ = self.createBounds()
        self.constraints_ = self.createConstraints()
        self.objective_function_ = self.createObjectiveFunction()
        res = minimize(self.objective_function_, (1), method='SLSQP', bounds=self.bounds_,constraints=self.constraints_)
        print(res)
        if(not res.success):
            print("Time constraint was too harsh. Given that you have max " + str(self.relevant_cpu_list_[2]) + " CPUs. The fastest time you can do is " + str(self.timeConstraintFn(self.relevant_cpu_list_[2])) + " seconds.") 
        # print(linear_coefficients)
        
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
            print("Finished running sky benchmarks. Will now terminate clusters.")
        
        benchmark_down_command = 'sky down --all < benchmark_input.txt'
        benchmark_down = subprocess.Popen(benchmark_down_command, shell=True, stdout=subprocess.PIPE, )
        while True:
            line = benchmark_down.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                print(line)
            else:
                break
        
        benchmark_delete_command = 'sky bench delete --all < benchmark_input.txt'
        benchmark_delete = subprocess.Popen(benchmark_delete_command, shell=True, stdout=subprocess.PIPE, )
        while True:
            line = benchmark_delete.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                print(line)
            else:
                break
        if self.debug_:
            print("Successfully terminated clusters.")
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
        num_lines_to_cost_data = 2
        while True:
            line = benchmark_setup.stdout.readline()
            #if line == '' and process.poll() is not None:
            #    break
            if line:
                line_byte = line.strip()
                line = line_byte.decode('utf-8')
                line_output = "Hardware count " + str(hardware_count) + ": " + line 
                print(line_output)
                if(self.got_cost_info_):
                    if(num_lines_to_cost_data == 0):
                        num_lines_to_cost_data = 2
                        self.got_cost_info_ = False
                        line_array = line.split()
                        self.benchmark_cost_results_.append((int(line_array[self.benchmark_cost_substring_cpu_index_]),float(line_array[self.benchmark_cost_substring_cost_index_]) / self.SECONDS_PER_HOUR))
                    else:
                        num_lines_to_cost_data -= 1
                if(self.benchmark_cost_substring_1_ in line and self.benchmark_cost_substring_2_ in line):
                    self.got_cost_info_ = True
                    num_lines_to_cost_data -= 1
                    
                    
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
        # Weird exception where if duration is exactly 5 minutes and 0 seconds, everything after is shifted by 1
        seconds_per_step = None
        if('s' not in seconds_per_step_array[self.seconds_per_step_index_ - 3]):
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_ - 1]
        else:
            seconds_per_step = seconds_per_step_array[self.seconds_per_step_index_]
        self.benchmark_time_results_.append((hardware_count,float(seconds_per_step)))
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

    def regressionSolver(self,data,purpose):
        if self.debug_:
            print("Running least squares regression to solve for " + purpose +" per hardware")
        data_np = np.array(data)
        # Linear regression
        x = data_np[:,0]#np.array(list(self.benchmark_time_results_.keys()))
        y = data_np[:,1]#np.array(list(self.benchmark_time_results_.values()))
        log_x = np.log(x)
        log_y = np.log(y)
        regressions = []
        linear_regression = self.linearRegression(x,y,ModelType.LINEAR)
        regressions.append(linear_regression)
        power_regression = self.linearRegression(log_x,log_y,ModelType.POWER)
        regressions.append(power_regression)
        exponential_regression = self.linearRegression(x,log_y,ModelType.EXPONENTIAL)
        regressions.append(exponential_regression)
        logarithmic_regression = self.linearRegression(log_x,y,ModelType.LOGARITHMIC)
        regressions.append(logarithmic_regression)
        #if(purpose == 'Time'):
        #    quadratic_regression = self.quadraticRegression(x,y)
        #    regressions.append(quadratic_regression)
        max_rsquared = 0
        coefficients = None
        model = None
        for regression in regressions:
            (rsquared,linear_coefficients,constraint_type) = regression
            if(abs(rsquared) > max_rsquared):
                max_rsquared = abs(rsquared)
                coefficients = linear_coefficients
                model = constraint_type
        if self.debug_:
            print(purpose + " Model R^2: " + str(max_rsquared))
            print(purpose + " Model Coefficients: " + str(coefficients))
            print(purpose + " Model Type: " + str(model))
            #print("Linear regression: " + str(linear_regression))
            #print("Power regression: " + str(power_regression))
            #print("Exponential regression: " + str(exponential_regression))
            #print("Logarithmic regression: " + str(logarithmic_regression))
        return (coefficients,model)
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

    def linearRegression(self,x,y,constraint_type):
        A = np.vstack([x, np.ones(len(x))]).T
        linear_coefficients = np.linalg.lstsq(A, y, rcond=None)[0]
        y_pred = linear_coefficients[0] * x + linear_coefficients[1]
        rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        return (rsquared,linear_coefficients,constraint_type)

    def quadraticRegression(self,x,y,constraint_type=ModelType.QUADRATIC):
        A = np.vstack([x**2,x, np.ones(len(x))]).T
        print("Quad debugging")
        print(A)
        print(y)
        linear_coefficients = np.linalg.lstsq(A, y, rcond=None)[0]
        print(linear_coefficients)
        y_pred = linear_coefficients[0] * (x ** 2) + linear_coefficients[1] * x + linear_coefficients[2]
        rsquared = 1.0 - np.sum((y - y_pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
        return (rsquared,linear_coefficients,constraint_type)

    def createBounds(self):
        if(self.debug_):
            print("Creating bounds for optimization problem")
        low_cpu = self.ZERO_EPSILON
        high_cpu = self.relevant_cpu_list_[2]
        if(self.debug_):
            print("Low CPU bound: " + str(low_cpu))
            print("High CPU bound: " + str(high_cpu))
        return [(low_cpu,high_cpu)]

    def createConstraints(self):
        if(self.debug_):
            print("Creating bounds for optimization problem")
        constraints = []
        cost_lower_bound = 0
        cost_upper_bound = self.max_cost_

        time_lower_bound = 0
        time_upper_bound = self.max_time_

        self.timeModel = self.makeModel(self.time_model_info_)
        self.costModel = self.makeModel(self.cost_model_info_)
        self.costConstraintFn = self.costFunction()
        self.timeConstraintFn = self.timeFunction()

        if (self.debug_):
            print("CPU COUNT 2")
            print("Time (Sec/Step): " + str(self.timeModel(2)))
            print("Cost ($/Sec): " + str(self.costModel(2)))
            print("$/Hr: " + str(self.costModel(2) * self.SECONDS_PER_HOUR))
            print("Total Cost: " + str(self.costConstraintFn(2)))
            print("Total Time: " + str(self.timeConstraintFn(2)))

            print("CPU COUNT 16")
            print("Time (Sec/Step): " + str(self.timeModel(16)))
            print("Cost ($/Sec): " + str(self.costModel(16)))
            print("$/Hr: " + str(self.costModel(16) * self.SECONDS_PER_HOUR))
            print("Total Cost: " + str(self.costConstraintFn(16)))
            print("Total Time: " + str(self.timeConstraintFn(16)))

            print("CPU COUNT 64")
            print("Time (Sec/Step): " + str(self.timeModel(64)))
            print("Cost ($/Sec): " + str(self.costModel(64)))
            print("$/Hr: " + str(self.costModel(64) * self.SECONDS_PER_HOUR))
            print("Total Cost: " + str(self.costConstraintFn(64)))
            print("Total Time: " + str(self.timeConstraintFn(64)))
        #cost_constraint = NonlinearConstraint(self.costConstraintFn,cost_lower_bound,cost_upper_bound)
        time_constraint = NonlinearConstraint(self.timeConstraintFn,time_lower_bound,time_upper_bound)
        constraints.append(time_constraint)
        return constraints

    def createObjectiveFunction(self):
        if(self.debug_):
            print("Creating objective function for optimization problem")
        return self.costFunction()

    def costFunction(self):
        def costFn(x):
            seconds_per_step = self.timeModel(x)
            dollars_per_second = self.costModel(x)
            return dollars_per_second * seconds_per_step * self.steps_
        return costFn

    def timeFunction(self):
        def timeFn(x):
            seconds_per_step = self.timeModel(x)
            return seconds_per_step * self.steps_
        return timeFn

    def makeModel(self,model_info):
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
                    y = model_info[0][0] * (x ** 2) + model_info[0][1] * x + model_info[0][2]
                    return y
                return model

    def solveOptimization(self,objective_function_type):
        match objective_function_type:
            case OptimizationFunctionType.COST:
                return "COST"
            case OptimizationFunctionType.TIME:
                return "Time not implemented yet"


def main():
    print("Hello World!")
    parser = argparse.ArgumentParser()
    parser.add_argument('-y', '--yaml_file', required=True)
    parser.add_argument('-s','--steps',required=True,type=int)
    parser.add_argument('-c','--max_cost',required=True,type=float)
    parser.add_argument('-t','--max_time',required=True,type=float)
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
