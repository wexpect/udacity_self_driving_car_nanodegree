import numpy as np
from numpy.linalg import inv

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = 2 # process model dimension

    def F(self):
        # system matrix
        
        # Note: assume delta_t = 1
        
        return np.matrix([[1, 1],  
                        [0, 1]])

    def Q(self):
        # process noise covariance Q
        
        # NOTE: assume constant velocity
        
        return np.matrix([[0, 0],  
                        [0, 0]])
        
    def H(self):
        # measurement matrix H
        return np.matrix([[1, 0]])
    
    def predict(self, x, P):
        # predict state and estimation error covariance to next timestep

        ############
        # TODO: implement prediction step
        ############
        
        x = self.F() * x
        P = self.F() * P * self.F().transpose() + self.Q()                
        
        return x, P

    def update(self, x, P, z, R):
        # update state and covariance with associated measurement

        ############
        # TODO: implement update step
        ############
        y = z - self.H() * x       
        S = self.H() * P * self.H().transpose() + R
        K = P * self.H().transpose() * inv(S)
        
        x = x + K * y
        I = np.identity(self.dim_state)
        P = (I - K * self.H()) * P
        
        return x, P     
        
        
def run_filter():
    ''' loop over data and call predict and update'''
    np.random.seed(10) # make random values predictable
    
    # init filter
    KF = Filter()
    
    # init track state and covariance
    x = np.matrix([[0],
                [0]])
    P = np.matrix([[5**2, 0],
                [0, 5**2]])  
    
    # loop over measurements and call predict and update
    for i in range(1,101):        
        print('------------------------------')
        print('processing measurement #' + str(i))
        
        # prediction
        x, P = KF.predict(x, P) # predict to next timestep
        print('x- =', x)
        print('P- =', P)
        
        # measurement generation
        sigma_z = 1 # measurement noise
        
        # Note: z has noise
        z = np.matrix([[i + np.random.normal(0, sigma_z)]]) # generate noisy measurement
        R = np.matrix([[sigma_z**2]]) # measurement covariance
        print('z =', z)
        
        # update
        x, P = KF.update(x, P, z, R) # update with measurement
        print('x+ =', x)
        print('P+ =', P)
        

# call main loop
run_filter()

"""
Print:

------------------------------
processing measurement #99
x- = [[98.99734631]
 [ 0.99807839]]
P- = [[4.14298141e-02 6.30649680e-04]
 [6.30649680e-04 1.27352017e-05]]
z = [[99.82699862]]
x+ = [[99.03035126]
 [ 0.99858079]]
P+ = [[3.97816670e-02 6.05561385e-04]
 [6.05561385e-04 1.23533046e-05]]
------------------------------
processing measurement #100
x- = [[100.02893205]
 [  0.99858079]]
P- = [[4.10051431e-02 6.17914689e-04]
 [6.17914689e-04 1.23533046e-05]]
z = [[98.04548788]]
x+ = [[99.95080428]
 [ 0.99740347]]
P+ = [[3.93899524e-02 5.93575059e-04]
 [5.93575059e-04 1.19865258e-05]]
"""