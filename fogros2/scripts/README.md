# Optimization Script

## Installation and Setup
Follow the installation steps on the [`Sky Setup Instructions`](https://github.com/KeplerC/FogROS2-sky#sky-setup-instructions) section of the FogROS2-Sky README. 

Before running the optimization script, configure AWS using your access key and secret access key.

```
aws configure
sky check
```

## Command Line Arguments
| Short Argument | Long Argument        | Required | Type   | Description                                                                                                                             |
| -------------- | -------------        | -------- | -----  | -----------                                                                                                                             |
| -y             | --yaml_file          | True     | String | Path to yaml file from current directory (not the directory that the script is in)                                                      |
| -gy            | --gpu_yaml_file      | True     | String | Path to gpu yaml file from current directory (not the directory that the script is in)                                                  |
| -s             | --steps              | True     | Int    | Total number of steps you want the algorithm to take (Dexnet by default)                                                                |
| -c             | --max_cost           | True     | Float  | Maximum financial cost you want to spend on cloud computing (in $)                                                                      |
| -t             | --max_time           | True     | Float  | Maximum amount of time you want the algorithm to spend on the cloud (in seconds)                                                        |
| -oc            | --max_objective_cost | True     | Float  | Maximum objective cost (used only for motion planning, TODO: Remove objective cost requirement for SLAM and Dexnet where it isn't used) |
| -d             | --debug              | False    | N/A    | Type -d if you want to run debug mode                                                                                                   |
In order to run SLAM, yaml and gpu yaml files need to contain slam in the filename. In order to run motion planning, yaml and gpu yaml files need to contain mp. Otherwise, Dexnet is run by default.
TODO: Add command line argument to specify if the program should run Dexnet, SLAM, or Motion Planning.

## Example Runs

### Dexnet (Light Constraints)
```
cd ~/FogROS2-sky/fogros2/scripts
python3 sky_optimization.py -y ../../benchmark.yaml -gy ../../benchmark_gpu.yaml -s 1000 -c 1 -t 3500 -oc 700 -d
```
#### Outputs
Successful optimization  
Optimized CPU: 6.583081308434977  
Optimized Memory: 13.166162616833319  
Projected time: 3500.000000746982 seconds  
Projected cost: $0.27147393261837877  
c6i.2xlarge  
Successful optimization  
Optimized CPU: 29.10002363774633  
Optimized Memory: 208.22697699205892  
Projected time: 2067.889985919936 seconds  
Projected cost: $1.0000000067110584  
r6i.8xlarge  
T4  
Successful optimization  
Optimized CPU: 4.0  
Optimized Memory: 16.0  
Projected time: 1841.2 seconds  
Projected cost: $0.26924140299114097  
g4dn.xlarge  
Successful optimization  
Optimized CPU: 4.0  
Optimized Memory: 16.0  
Projected time: 1841.2 seconds  
Projected cost: $0.26924140299114097  
g4dn.xlarge  

The first optimization used cost as an objective function and set time as a constraint. We can see that we selected CPU and memory such that cost was minimized while still adhering to the user's time constraint of 3500 seconds. The closest AWS instance to imitate this hardware setup is the c6i.2xlarge.  
The second optimization used time as an objective function and set cost as a constraint. We can see that we selected CPU and memory such that time was minimized while still adhering to the user's cost constraint of $5. The closest AWS instance to imitate this hardware setup is the r6i.8xlarge.  
Now, it is important to note the latter 2 optimizations including the T4 GPU also satisfy the constraints, and seem to work more efficiently. This can be highlighed in the next section with harsher constraints.
### Dexnet (Harsh Constraints)
```
cd ~/FogROS2-sky/fogros2/scripts
python3 sky_optimization.py -y ../../benchmark.yaml -gy ../../benchmark_gpu.yaml -s 1000 -c 0.5 -t 2000 -oc 700 -d
```
#### Outputs
Failed optimization. Please relax constraints  
Closest CPU: 55.174132878793564  
Closest Memory: 110.34826575767434  
Projected time: 1999.999999995834 seconds  
Projected cost: $1.2847671764297848  
c6i.16xlarge  
Failed optimization. Please relax constraints  
Closest CPU: 19.513887987497554  
Closest Memory: 39.027775974959205  
Projected time: 2197.1952866003394 seconds  
Projected cost: $0.5000000000002129  
c6i.4xlarge  
T4  
Successful optimization  
Optimized CPU: 4.0  
Optimized Memory: 16.0  
Projected time: 1835.8785714325759 seconds  
Projected cost: $0.2679159162503509  
g4dn.xlarge  
Successful optimization  
Optimized CPU: 4.0  
Optimized Memory: 16.0  
Projected time: 1835.8785714325759 seconds  
Projected cost: $0.2679159162503509  
g4dn.xlarge  

The first two optimizations attempt to find an instance to run Dexnet in under 2000 seconds and $0.50. With just CPU, this task is not possible under those constraints, but the latter 2 optimizations show cost and time optimizations when a T4 is included. We can see that additional CPU and memory don't help with computation since the optimal CPU and memory for the T4 GPU is the minimum amount. Furthermore, since CPU and memory are most efficient at the minimum, the GPU optimizations don't push up to the user's constraints.

### SLAM
```
cd ~/FogROS2-sky/fogros2/scripts
python3 sky_optimization.py -y ../../benchmark_slam.yaml -gy ../../benchmark_slam_gpu.yaml -s 1000000 -c 5 -t 55000 -oc 650 -d
```
#### Outputs
Successful optimization  
Optimized CPU: 4.586338333006091  
Optimized Memory: 16.89002258471161  
Projected time: 54999.9999999424284 seconds  
Projected cost: $3.444474922339687  
m6i.xlarge  
Successful optimization  
Optimized CPU: 7.441775519458142  
Optimized Memory: 59.53420428352985  
Projected time: 38322.24476746237 seconds  
Projected cost: $4.999999996850867  
r6i.2xlarge  

The first optimization used cost as an objective function and set time as a constraint. We can see that we selected CPU and memory such that cost was minimized while still adhering to the user's time constraint of 55000 seconds. The closest AWS instance to imitate this hardware setup is the m6i.xlarge.  
The second optimization used time as an objective function and set cost as a constraint. We can see that we selected CPU and memory such that time was minimized while still adhering to the user's cost constraint of $5. The closest AWS instance to imitate this hardware setup is the r6i.2xlarge.  

### Motion Planning
```
cd ~/FogROS2-sky/fogros2/scripts
python3 sky_optimization.py -y ../../benchmark_mp.yaml -gy ../../benchmark_mp_gpu.yaml -s 1000 -c 2 -t 3500 -oc 650 -d
```
#### Outputs
Successful optimization  
Optimized CPU: 14.868582555026965  
Optimized Memory: 29.73716511005242  
Projected objective cost: 650.0000000000771  
Projected cost: $1.4081357880019716  
c6i.4xlarge  
Successful optimization  
Optimized CPU: 23.744358343008674  
Optimized Memory: 47.488716242681406  
Projected objective cost: 637.8114539965433  
Projected cost: $2.000000117784925  
c6i.4xlarge  

The first optimization used cost as an objective function and set objective cost as a constraint. We can see that we selected CPU and memory such that cost was minimized while still adhering to the user's objective cost constraint of 650. The closest AWS instance to imitate this hardware setup is the c6i.4xlarge.  
The second optimization used objective cost as an objective function and set cost as a constraint. We can see that we selected CPU and memory such that objective cost was minimized while still adhering to the user's cost constraint of $2. The closest AWS instance to imitate this hardware setup is the c6i.4xlarge (same as the first optimization).  
It is also important to note that the memory:CPU ratio for both optimizations is 2, meaning that cloud instances with excessive memory is inefficient for this motion planning task.  
