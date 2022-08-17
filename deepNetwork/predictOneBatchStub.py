import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'
import numpy as np

class inference:
    def __init__(self,model=None):
        self.model = model
        print("using model: "+self.model)
        
    def run(self,points):  
        Npoints = len(points)
        #print(str(Npoints)+" points")
        return np.random.random_sample((2*Npoints,))
        
        