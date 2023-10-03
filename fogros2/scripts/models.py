import numpy as np
from scipy.optimize import curve_fit

## @class GPUModel
# @brief Finds best fit time/cost model as a function of hardware for GPU instances
#
class GPUModel:

    ## @brief GPUModel Constructor
    # @param x Hardware Data
    # @param y Time, Cost, or Objective Cost
    #
    def __init__(self,x,y):
        ## @brief Hardware Data
        #
        self.x_ = x

        ## @brief Time, Cost, or Objective Cost
        #
        self.y_ = y

        ## @brief Minimum r^2 for best fit model (if it is below, then we say the correlation isn't strong enough)
        #
        self.r_squared_threshold_ = 0.93
        model_list = [
            (self.linear_model,2,"Linear"),
            (self.log_model,2,"Log"),
            (self.exp_model,2,"Exp"),
            (self.power_model,2,"Power"),
            (self.hyper_model,2,"Hyper")
        ]
        
        ## @brief R^2 for best fit model
        #
        self.max_rsquared_ = 0

        ## @brief Best fit model
        #
        self.model_ = None

        ## @brief Const model boolean is true if the best fit model doesn't have a strong enough correlation
        #
        self.is_const_model_ = False

        # Solves for best fit model
        for (model,coef_len,name) in model_list:
            rsquared = None
            try:                
                rsquared,coefficients = self.regression(model,coef_len)
                if(rsquared > self.max_rsquared_ and rsquared <= 1):
                    self.max_rsquared_ = rsquared
                    self.model_ = (model,coefficients)
                print(name + ": " + str(rsquared))
            except:
                continue

        if(self.max_rsquared_ < self.r_squared_threshold_):
            coefficients = [np.mean(self.y_)]
            self.model_ = (self.const_model,coefficients)

    ## @brief Gets best fit model
    # @return Best Fit Model
    #
    def getModel(self):
        if(not self.is_const_model_):
            print(self.max_rsquared_)
        print(self.model_)
        return self.model_
    
    ## @brief Performs regression based on model type
    # @param model Regression Model Type
    # @param coeff_len Number of coefficients required for this regression model
    # @return Tuple containing R^2 and coefficients of given model
    #
    def regression(self,model,coeff_len):
        coefficients, _ = curve_fit(model, self.x_, self.y_,p0=np.zeros(coeff_len),maxfev=100000)
        y_pred = model(np.array(self.x_),coefficients,predict=True)
        rsquared = 1.0 - np.sum((self.y_ - y_pred) ** 2) / np.sum((self.y_ - np.mean(self.y_)) ** 2)
        return (rsquared, coefficients)
    
    ## @brief Constant Regression Model
    # @return Model Outputs
    #
    def const_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0]
        elif(predict):
            return coefficients[0][0]
        else:
            return coefficients[0]
        
    ## @brief Linear Regression Model
    # @return Model Outputs
    #
    def linear_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0] + coefficients[0][1] * x[0]
        elif(predict):
            return coefficients[0][0] + coefficients[0][1] * x
        else:
            return coefficients[0] + coefficients[1] * x
        
    ## @brief Log Regression Model
    # @return Model Outputs
    #
    def log_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0] + coefficients[0][1] * np.log(x[0])
        elif(predict):
            return coefficients[0][0] + coefficients[0][1] * np.log(x)
        else:
            return coefficients[0] + coefficients[1] * np.log(x)
    
    ## @brief Exponential Regression Model
    # @return Model Outputs
    #
    def exp_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0] * np.exp(coefficients[0][1] * x[0])
        elif(predict):
            return coefficients[0][0] * np.exp(coefficients[0][1] * x)
        else:
            return coefficients[0] * np.exp(coefficients[1] * x)

    ## @brief Power Regression Model
    # @return Model Outputs
    #    
    def power_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0] * (x[0] ** coefficients[0][1])
        elif(predict):
            return coefficients[0][0] * (x ** coefficients[0][1])
        else:
            return coefficients[0] * (x ** coefficients[1])
    
    ## @brief Hyperbolic Regression Model
    # @return Model Outputs
    #
    def hyper_model(self,x,*coefficients,predict=False,single=False):
        if(single):
            return coefficients[0][0] + (coefficients[0][1] / x[0])
        elif(predict):
            return coefficients[0][0] + (coefficients[0][1] / x)
        else:
            return coefficients[0] + (coefficients[1] / x)

## @class Model
# @brief Finds best fit time/cost model as a function of hardware for non-GPU instances
#
class Model:

    ## @brief Model Constructor
    # @param x Hardware Data
    # @param y Time, Cost, or Objective Cost
    #
    def __init__(self,x,y):
        ## @brief Hardware Data
        #
        self.x_ = x

        ## @brief Time, Cost, or Objective Cost
        #
        self.y_ = y

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
        ## @brief R^2 for best fit model
        #
        self.max_rsquared_ = 0

        ## @brief Best fit model
        #
        self.model_ = None

        # Solves for best fit model
        for (model,coef_len,name) in model_list:
            rsquared = None
            try:
                rsquared,coefficients = self.regression(model,coef_len)
                print("Name: " + str(rsquared))        
                if(rsquared > self.max_rsquared_ and rsquared <= 1):
                    self.max_rsquared_ = rsquared
                    self.model_ = (model,coefficients)
                print(name + ": " + str(rsquared))
            except:
                continue

    ## @brief Gets best fit model
    # @return Best Fit Model
    #
    def getModel(self):
        print(self.max_rsquared_)
        print(self.model_)
        return self.model_

    ## @brief Performs regression based on model type
    # @param model Regression Model Type
    # @param coeff_len Number of coefficients required for this regression model
    # @return Tuple containing R^2 and coefficients of given model
    #
    def regression(self,model,coeff_len):
        coefficients, _ = curve_fit(model, (self.x_[:,0],self.x_[:,1]), self.y_,p0=np.zeros(coeff_len),maxfev=100000)
        y_pred = model(np.array(self.x_),coefficients,predict=True)
        rsquared = 1.0 - np.sum((self.y_ - y_pred) ** 2) / np.sum((self.y_ - np.mean(self.y_)) ** 2)
        return (rsquared, coefficients)
    
    ## @brief Model where CPU is linear and Memory is linear
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def linear_linear_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * x[1]
        
    ## @brief Model where CPU is linear and Memory is log
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def linear_log_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * np.log(x[1])
    
    ## @brief Model where CPU is linear and Memory is exponential
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def linear_exp_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * np.exp(coefficients[3] * x[1])
    
    ## @brief Model where CPU is linear and Memory is power
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def linear_power_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * (x[1] ** coefficients[3])
    
    ## @brief Model where CPU is linear and Memory is hyperbolic
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def linear_hyper_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * x[0] + (coefficients[2] / x[1])
    
    ## @brief Model where CPU is log and Memory is linear
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def log_linear_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * x[0] + coefficients[2] * x[1]
        
    ## @brief Model where CPU is log and Memory is log
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def log_log_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * np.log(x[1])
    
    ## @brief Model where CPU is log and Memory is exponential
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def log_exp_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * np.exp(coefficients[3] * x[1])
    
    ## @brief Model where CPU is log and Memory is power
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def log_power_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + coefficients[2] * (x[1] ** coefficients[3])
    
    ## @brief Model where CPU is log and Memory is hyper
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def log_hyper_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.log(x[:,0]) + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.log(x[0]) + (coefficients[2] / x[1])
    
    ## @brief Model where CPU is exponential and Memory is linear
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def exp_linear_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * x[1]
        
    ## @brief Model where CPU is exponential and Memory is log
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def exp_log_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * np.log(x[1])
    
    ## @brief Model where CPU is exponential and Memory is exponential
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def exp_exp_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * np.exp(coefficients[0][4] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0])+ coefficients[3] * np.exp(coefficients[4] * x[1])
        
    ## @brief Model where CPU is exponential and Memory is power
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def exp_power_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + coefficients[0][3] * (x[:,1] ** coefficients[0][4])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + coefficients[3] * (x[1] ** coefficients[4])
    
    ## @brief Model where CPU is exponential and Memory is hyperbolic
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def exp_hyper_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * np.exp(coefficients[0][2] * x[:,0]) + (coefficients[0][3] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * np.exp(coefficients[2] * x[0]) + (coefficients[3] / x[1])
        
    ## @brief Model where CPU is power and Memory is linear
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def power_linear_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * x[1]

    ## @brief Model where CPU is power and Memory is log
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    # 
    def power_log_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.log(x[1])
    
    ## @brief Model where CPU is power and Memory is exponential
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def power_exp_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * np.exp(coefficients[0][4] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * np.exp(coefficients[4] * x[1])
        
    ## @brief Model where CPU is power and Memory is power
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def power_power_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + coefficients[0][3] * (x[:,1] ** coefficients[0][4])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + coefficients[3] * (x[1] ** coefficients[4])
    
    ## @brief Model where CPU is power and Memory is hyperbolic
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def power_hyper_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] * (x[:,0] ** coefficients[0][2]) + (coefficients[0][3] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] * (x[0] ** coefficients[2]) + (coefficients[3] / x[1])
    
    ## @brief Model where CPU is hyperbolic and Memory is linear
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def hyper_linear_model(self,x,*coefficients,predict=False,single=False):
        print(coefficients[3])
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * x[:,1]
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * x[1]

    ## @brief Model where CPU is hyperbolic and Memory is log
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #  
    def hyper_log_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * np.log(x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * np.log(x[1])
    
    ## @brief Model where CPU is hyperbolic and Memory is exponential
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def hyper_exp_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * np.exp(coefficients[0][3] * x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * np.exp(coefficients[3] * x[1])
    
    ## @brief Model where CPU is hyperbolic and Memory is power
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def hyper_power_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + coefficients[0][2] * (x[:,1] ** coefficients[0][3])
        else:
            return coefficients[0] + coefficients[1] / x[0] + coefficients[2] * (x[1] ** coefficients[3])
    
    ## @brief Model where CPU is hyperbolic and Memory is hyperbolic
    # @param x Model Input
    # @param coefficients Model coefficients
    # @param predict Boolean if we are predicting (False if we are trying to curve fit and solve)
    # @param single Boolean if we just want to run the model for single input
    # @return Model Output
    #
    def hyper_hyper_model(self,x,*coefficients,predict=False,single=False):
        if single:
            coefficients = coefficients[0] 
        if predict:
            return coefficients[0][0] + coefficients[0][1] / x[:,0] + (coefficients[0][2] / x[:,1])
        else:
            return coefficients[0] + coefficients[1] / x[0] + (coefficients[2] / x[1])