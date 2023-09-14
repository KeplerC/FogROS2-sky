# import argparse

# hardware_to_instance_map = {
#                                 (2,4): "c6i.large",
#                                 (2,8): "m6i.large",
#                                 (2,16): "r6i.large",
#                                 (4,8): "c6i.xlarge",
#                                 (4,16): "m6i.xlarge",
#                                 (4,32): "r6i.xlarge",
#                                 (8,16): "c6i.2xlarge",
#                                 (8,32): "m6i.2xlarge",
#                                 (8,64): "r6i.2xlarge",
#                                 (16,32): "c6i.4xlarge",
#                                 (16,64): "m6i.4xlarge",
#                                 (16,128): "r6i.4xlarge",
#                                 (32,64): "c6i.8xlarge",
#                                 (32,128): "m6i.8xlarge",
#                                 (32,256): "r6i.8xlarge",
#                                 (64,128): "c6i.16xlarge",
#                                 (64,256): "m6i.16xlarge",
#                                 (64,512): "r6i.16xlarge",
#                                 (4,16): "g4dn.xlarge",
#                                 (8,32): "g4dn.2xlarge",
#                                 (16,64): "g4dn.4xlarge"
#                             }

# parser = argparse.ArgumentParser()
# parser.add_argument(
#     "-f",
#     "--file_path",
#     required=True,
#     help="Path to file with all the instances"
# )
# args = parser.parse_args()
# file_path = args.file_path
# with open(file_path, 'r') as file:
#     # Read the entire contents of the file into a variable
#     file_contents = file.read()

# file_array = file_contents.split(',')
# instance_array = []
# for file_instance in file_array:
#     if("AWS" in file_instance):
#         print(file_instance[4:-1])
import re
import numpy as np

def convert(match):
    base = float(match.group(1))
    exponent = int(match.group(2))
    return str(base * 10**exponent)


pattern = r"(\d+\.\d+)e(-\d+)"

# DEXNET VERSION
# hardware_counts = [(32, 256), (4, 32), (16, 128), (2,8), (64,256), (16,64), (32,128), (4,8), (16,32), (64,512), (2,8),  (4,8), (4,32), (8,32), (32,64), (64,128), (2,16), (4,32), (8,16), (4,16), (8,21)]
# specs = []
# for hardware_count in hardware_counts:
#     x = hardware_count
#     seconds_per_step = -0.14066574 + 32.43587816 / x[0] + 0.09870288 * (x[1] ** 0.46624948)
#     seconds_per_step = float(format(seconds_per_step, '.10f'))
#     dollars_per_second = 1.45707607e-06 + 9.58021567e-06 * (x[0] ** 1.00293409e+00) + 9.61408821e-07 * x[1]
#     dollars_per_second = float(format(dollars_per_second, '.10f'))
#     specs.append((seconds_per_step,dollars_per_second))
# print(specs)

hardware_counts = [
    (4, 8),
    (2, 8),
    (2, 16),
    (4, 8),
    (4, 16),
    (4, 32),
    (16, 32),
    (16, 64),
    (16, 128),
    (32, 64),
    (32, 128),
    (32, 256),
    (2, 16),
    (2, 8),
    (4, 7),
    (4, 32),
    (4, 16),
    (4, 16),
    (4, 16),
    (2, 3),
    (8, 21),
    (8, 32),
]
specs = []
for hardware_count in hardware_counts:
    x = hardware_count
    time_coefficients = [ 3.13776699e-02,  1.91431635e+00 ,-1.17343110e+00 , 2.65481622e-05]
    seconds_per_step = time_coefficients[0] + time_coefficients[1] * np.exp(time_coefficients[2] * x[0]) + time_coefficients[3] * x[1]
    seconds_per_step = float(format(seconds_per_step, '.10f'))
    cost_coefficients = [-3.15325144e+01,  9.70322349e-06,  3.15325155e+01,  3.04913898e-08]
    dollars_per_second = cost_coefficients[0] + cost_coefficients[1] * x[0] + cost_coefficients[2] * np.exp(cost_coefficients[3] * x[1])
    dollars_per_second = float(format(dollars_per_second, '.10f'))
    specs.append((seconds_per_step,dollars_per_second))
print(specs)