import numpy as np
from scipy.optimize import curve_fit


class Model:
    def __init__(self,x,y):
        print("Hello world")
        self.x_ = x
        self.y_ = y
        #print(x)
        #print(y)
        #print(self.x_)
        #print(self.y_)
        model_list = [
            (self.linear_linear_model,3,"Linear linear"),
            (self.linear_log_model,3,"Linear log"),
            (self.linear_exp_model,4,"Linear exp"),
            (self.linear_power_model,4,"Linear power"),
            (self.linear_hyper_model,3,"Linear hyper"),
            (self.log_linear_model,3,"Log linear"),
            (self.log_log_model,3,"Log log"),
            (self.log_exp_model,4,"Log exp"),
            (self.log_power_model,4,"Log power"),
            (self.log_hyper_model,3,"Log hyper"),
            (self.exp_linear_model,4,"Exp linear"),
            (self.exp_log_model,4,"Exp log"),
            (self.exp_exp_model,5,"Exp exp"),
            (self.exp_power_model,5,"Exp power"),
            (self.exp_hyper_model,4,"Exp hyper"),
            (self.power_linear_model,4,"Power linear"),
            (self.power_log_model,4,"Power log"),
            (self.power_exp_model,5,"Power exp"),
            (self.power_power_model,5,"Power power"),
            (self.power_hyper_model,4,"Power hyper"),
            (self.hyper_linear_model,3,"Hyper linear"),
            (self.hyper_log_model,3,"Hyper log"),
            (self.hyper_exp_model,4,"Hyper exp"),
            (self.hyper_power_model,4,"Hyper power"),
            (self.hyper_hyper_model,3,"Hyper hyper"),
        ]
        self.max_rsquared_ = 0
        self.model_ = None
        for (model,coef_len,name) in model_list:
            rsquared = None
            try:
                #print("I'm trying")
                rsquared,coefficients = self.regression(model,coef_len)
                if(rsquared > self.max_rsquared_ and rsquared <= 1):
                    self.max_rsquared_ = rsquared
                    self.model_ = (model,coefficients)
                print(name + ": " + str(rsquared))
            except:
                continue
                #print(name + ": " + "Didn't work")
        #print("Pass one")

    def getModel(self):
        print(self.max_rsquared_)
        print(self.model_)
        return self.model_

        # rsquared = self.regression(self.linear_linear_model,3)
        # print("Linear linear: " + str(rsquared))
        # rsquared = self.regression(self.linear_log_model,3)
        # print("Linear log: " + str(rsquared))
        # rsquared = self.regression(self.linear_exp_model,4)
        # print("Linear exp: " + str(rsquared))
        # rsquared = self.regression(self.linear_power_model,4)
        # print("Linear power: " + str(rsquared))
        # rsquared = self.regression(self.linear_hyper_model,3)
        # print("Linear hyper: " + str(rsquared))
        # rsquared = self.regression(self.log_linear_model,3)
        # print("Log linear: " + str(rsquared))
        # rsquared = self.regression(self.log_log_model,3)
        # print("Log log: " + str(rsquared))
        # rsquared = self.regression(self.log_exp_model,4)
        # print("Log exp: " + str(rsquared))
        # rsquared = self.regression(self.log_power_model,4)
        # print("Log power: " + str(rsquared))
        # rsquared = self.regression(self.log_hyper_model,3)
        # print("Log hyper: " + str(rsquared))
        # rsquared = self.regression(self.exp_linear_model,4)
        # print("Exp linear: " + str(rsquared))
        # rsquared = self.regressionThree(self.linear_linear_model)
        # print("Linear linear: " + str(rsquared))
        # rsquared = self.regressionThree(self.linear_log_model)
        # print("Linear log: " + str(rsquared))
        # rsquared = self.regressionFour(self.linear_exp_model)
        # print("Linear exp: " + str(rsquared))
        # rsquared = self.regressionFour(self.linear_power_model)
        # print("Linear power: " + str(rsquared))
        # rsquared = self.regressionThree(self.linear_hyper_model)
        # print("Linear hyper: " + str(rsquared))
        # rsquared = self.regressionThree(self.log_linear_model)
        # print("Log linear: " + str(rsquared))
        # rsquared = self.regressionThree(self.log_log_model)
        # print("Log log: " + str(rsquared))
        # rsquared = self.regressionFour(self.log_exp_model)
        # print("Log exp: " + str(rsquared))
        # rsquared = self.regressionFour(self.log_power_model)
        # print("Log power: " + str(rsquared))
        # rsquared = self.regressionThree(self.log_hyper_model)
        # print("Log hyper: " + str(rsquared))

    def regressionThree(self,model):
        coefficients, _ = curve_fit(model, (self.x_[:,0],self.x_[:,1]), self.y_,p0=[0,0,0])
        y_pred = model(np.array(self.x_),coefficients,predict=True)
        #y_pred = model(self.x_, coefficients)
        rsquared = 1.0 - np.sum((self.y_ - y_pred) ** 2) / np.sum((self.y_ - np.mean(self.y_)) ** 2)
        return (rsquared, coefficients)
    
    def regressionFour(self,model):
        coefficients, _ = curve_fit(model, (self.x_[:,0],self.x_[:,1]), self.y_,p0=[0,0,0,0])
        y_pred = model(np.array(self.x_),coefficients,predict=True)
        #y_pred = model(self.x_, coefficients)
        rsquared = 1.0 - np.sum((self.y_ - y_pred) ** 2) / np.sum((self.y_ - np.mean(self.y_)) ** 2)
        return (rsquared, coefficients)
    
    def regression(self,model,coeff_len):
        #print("Starting")
        #print((self.x_[:,0],self.x_[:,1]))
        #print(self.y_)
        coefficients, _ = curve_fit(model, (self.x_[:,0],self.x_[:,1]), self.y_,p0=np.zeros(coeff_len),maxfev=100000)
        #print("Coefficients: ",coefficients)
        y_pred = model(np.array(self.x_),coefficients,predict=True)
        #print("Y pred: ",y_pred)
        #y_pred = model(self.x_, coefficients)
        rsquared = 1.0 - np.sum((self.y_ - y_pred) ** 2) / np.sum((self.y_ - np.mean(self.y_)) ** 2)
        #print("Rsquared: ",rsquared)
        return (rsquared, coefficients)
    
    def linear_linear_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * x[1]
        
    def linear_log_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * np.log(x[1])
    
    def linear_exp_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * np.exp(coefficients[3] * x[1])
        
    def linear_power_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * (x[1] ** coefficients[3])
    
    def linear_hyper_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + (coefficients[2] / x[1])
        
    def log_linear_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * x[1]
        
    def log_log_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * np.log(x[1])
    
    def log_exp_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * np.exp(coefficients[3] * x[1])
        
    def log_power_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * (x[1] ** coefficients[3])
    
    def log_hyper_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + (coefficients[2] / x[1])
        
    def exp_linear_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * x[1]
        
    def exp_log_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * np.log(x[1])
    
    def exp_exp_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * np.exp(coefficients[0][4] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0])+ coefficients[3] * np.exp(coefficients[4] * x[1])
        
    def exp_power_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * (x[:,1] ** coefficients[0][4])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * (x[1] ** coefficients[4])
    
    def exp_hyper_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + (coefficients[0][3] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + (coefficients[3] / x[1])
        
    def power_linear_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * x[1]
        
    def power_log_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.log(x[1])
    
    def power_exp_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * np.exp(coefficients[0][4] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.exp(coefficients[4] * x[1])
        
    def power_power_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * (x[:,1] ** coefficients[0][4])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * (x[1] ** coefficients[4])
    
    def power_hyper_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + (coefficients[0][3] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + (coefficients[3] / x[1])
    def hyper_linear_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * x[1]
        
    def hyper_log_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * np.log(x[1])
    
    def hyper_exp_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * np.exp(coefficients[3] * x[1])
        
    def hyper_power_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * (x[1] ** coefficients[3])
    
    def hyper_hyper_model(self,x,*coefficients,predict=False,single=False):
        #print("HI GUYS")
        #print(x[0])
        #print(x[1])
        #print(coefficients[0])
        #print(coefficients[1])
        #print(coefficients[2])
        #print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + (coefficients[2] / x[1])


def main():
    x = np.array([(64, 256), (64, 512) ,(64, 128), (16, 64) ,(16, 32) ,(16, 128) ,(2, 8) ,(2, 16),(2, 4)])
    y = np.array([1.9994, 1.9961, 1.9961, 2.3098, 2.3416 ,2.3698, 16.8036, 15.7978, 17.4908])
    model = Model(x,y)
    


if __name__ == "__main__":
    main()
