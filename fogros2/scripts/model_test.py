from models import Model
import numpy as np

def hyper_power_model(x,*coefficients,predict=False):
    print(x)
    print(coefficients[0])
    exit()
    if predict:
        return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
    else:
        return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * (x[1] ** coefficients[3])

def power_log_model(x,*coefficients,predict=False,single=False):
    #print("HI GUYS")
    #print(x[0])
    #print(x[1])
    #print(coefficients[0])
    #print(coefficients[1])
    #print(coefficients[2])
    #print(coefficients[3])
    print(single)
    print(predict)
    if single:
        coefficients = coefficients[0] 
    if predict:
        return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * np.log(x[:,1])
    else:
        print(x[0])
        print(coefficients[2])
        print(x[0] ** coefficients[2])
        print(coefficients[1])
        print(coefficients[1] * (x[0] ** coefficients[2]))
        print(x[1])
        print(np.log(x[1]))
        print(coefficients[3] * np.log(x[1]))
        print(coefficients[0])
        print(coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.log(x[1]))
        return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.log(x[1])
hardware_count = (2, 4)
hardware_count = np.array(hardware_count)
coefficients = np.array([3.71975094e-01, 3.08394670e+01, 1.64572755e-02, 7.01790597e-01])
print(coefficients)
print(power_log_model(hardware_count,coefficients,predict=False,single=True))
# x = np.array([(64, 256), (64, 512) ,(64, 128), (16, 64) ,(16, 32) ,(16, 128) ,(2, 8) ,(2, 16),(2, 4)])
# y = np.array([1.9994, 1.9961, 1.9961, 2.3098, 2.3416 ,2.3698, 16.8036, 15.7978, 17.4908])
# print("Start x: ",x)
# print("Start x: ",x.shape)
# print(y)
# print(y.shape)
# print(type(y))
# data = [((64, 256), 1.9955), ((64, 128), 1.9996), ((64, 512), 1.9981), ((16, 64), 2.2834), ((16, 32), 2.2927), ((16, 128), 2.2994), ((2, 8), 15.5392), ((2, 16), 15.2976), ((2, 4), 15.9449)]
# #data = [(64, 1.9955), (2, 8), (4, 15.9449)]
# data_np = np.array(data)
# x = data_np[:, 0]
# y = data_np[:, 1]
# x_np = None
# y_np = None
# print(type(y[0]) is tuple)
# exit()
# if(len(x[0]) > 1):
#     x_np = np.array([list(item) for item in x])
# else:
#     x_np = np.array(x)
# if(len(y[0]) > 1):
#     y_np = np.array([list(item) for item in y])
# else:
#     y_np = np.array(y)
# print(len(x[0]))
# exit()

# y_np = np.array(y)

# model = Model(x_np,y_np)
# #print(type(data))
# #model = Model(x,y)