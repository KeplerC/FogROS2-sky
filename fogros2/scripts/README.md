# Optimization Script

## Setup
Before running the optimization script, configure AWS using your access key and secret access key.

```
aws configure
sky check
```

## Command Line Arguments
| Short Argument | Long Argument | Required | Type   | Description                                                                        |
| -------------- | ------------- | -------- | -----  | -----------                                                                        |
| -y             | --yaml_file   | True     | String | Path to yaml file from current directory (not the directory that the script is in) |
| -s             | --steps       | True     | Int    | Total number of steps you want the algorithm to take (Dexnet by default)           |
| -c             | --max_cost    | True     | Float  | Maximum financial cost you want to spend on cloud computing (in $)                 |
| -t             | --max_time    | True     | Float  | Maximum mount of time you want the algorithm to spend on the cloud (in seconds)    |
| -d             | --debug       | False    | N/A    | Type -d if you want to run debug mode                                              |

## Sample Run
```
python3 test_optimization.py -y ./path/to/benchmark.yaml -s 1000 -c 1.30 -t 2000 -d
```
