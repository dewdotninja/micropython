# PID controller class
# dew.ninja  october 2022

class PID:
    def __init__(self,kp,ki,kd,kt,n,wp,wd,ts):
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kt = kt
        self.n = n
        self.wp = wp
        self.wd = wd
        self.ts = ts
        
        # coefficients
        self.bi = 0.5*self.ts*self.ki
        self.bt = 0.5*self.ts*self.kt
        ad1 = 1+0.5*self.n*self.ts
        ad2 = 0.5*self.n*self.ts - 1
        self.ad = -ad2/ad1
        self.bd = self.kd*self.n/ad1

        # controller states
        self.e1 = 0
        self.e0 = 0
        self.ed1 = 0
        self.ed0 = 0
        self.eus1 = 0
        self.eus0 = 0
        self.up0 = 0
        self.ui1 = 0
        self.ui0 = 0
        self.ud1 = 0
        self.ud0 = 0
        self.u = 0
        self.ulim = 0

        # controller output limits
        self.output_limit = True
        self.umax = 1000  # some default values
        self.u_offset = False
    # get & set output limit
    def getulim(self):
        return self.umax
    def setulim(self,umax):
        self.umax = umax
    def getulimstatus(self):
        return self.output_limit
    def enableulim(self):
        self.output_limit = True
    def disableulim(self):
        self.output_limit = False

    # get & enable/disable output offset
    def getoffsetstatus(self):
        return self.u_offset
    def enableoffset(self):
        self.output_limit = True # need to use output limit
        self.u_offset = True 
    def disableoffset(self):
        self.u_offset = False
        
    # parameter getters & setters 
    def getkp(self):
        return self.kp
    def getki(self):
        return self.ki
    def getkd(self):
        return self.kd
    def getkt(self):
        return self.kt
    def getn(self):
        return self.n
    def getwp(self):
        return self.wp
    def getwd(self):
        return self.wd
    def getts(self):
        return self.ts
    def getparms(self):
        # parms = np.array([self.kp,self.ki,self.kd,self.kt,self.n,self.wp,self.wd,self.ts])
        parms = [0.0]*8
        return parms

    def setkp(self,kp):
        self.kp = kp
    def setki(self,ki):
        self.ki = ki
    def setkd(self,kd):
        self.kd = kd
    def setkt(self,kt):
        self.kt = kt
    def setn(self,n):
        self.n = n
    def setwp(self,wp):
        self.wp = wp
    def setwd(self,wd):
        self.wd = wd
    def setts(self,ts):
        self.ts = ts
    def setparms(self,parms):
        self.kp = parms[0]
        self.ki = parms[1]
        self.kd = parms[2]
        self.kt = parms[3]
        self.n = parms[4]
        self.wp = parms[5]
        self.wd = parms[6]
        self.ts = parms[7]
   
    # coefficient update
    def update(self):
        self.bi = 0.5*self.ts*self.ki
        self.bt = 0.5*self.ts*self.kt
        ad1 = 1+0.5*self.n*self.ts
        ad2 = 0.5*self.n*self.ts - 1
        self.ad = -ad2/ad1
        self.bd = self.kd*self.n/ad1
        
    # reset controller states
    def reset(self):
        self.e1 = 0
        self.e0 = 0
        self.ed1 = 0
        self.ed0 = 0
        self.eus1 = 0
        self.eus0 = 0
        self.up0 = 0
        self.ui1 = 0
        self.ui0 = 0
        self.ud1 = 0
        self.ud0 = 0
        self.u = 0
        self.ulim = 0

    def out(self,r,y):
        # state transfer
        self.e1 = self.e0
        self.ed1 = self.ed0
        self.eus1 = self.eus0
        
        self.ui1 = self.ui0
        self.ud1 = self.ud0
        # compute errors for each term
        self.e0 = r - y
        self.ep0 = self.wp*r - y # weighted proportional error
        self.ed0 = self.wd*r - y # weighted derivative error
        
        self.up0 = self.kp*self.ep0 # output of P term
        self.ui0 = self.ui1 +self.bi*(self.e0+self.e1) + self.bt*(self.eus0+self.eus1) # output of I term
        self.ud0 = self.ad*self.ud1 +self.bd*(self.ed0 - self.ed1) # output of D term
        self.u = self.up0 + self.ui0 + self.ud0        
        self.ulim = self.u
        if self.output_limit:
            if self.u_offset:  # offset to half of self.umax
                UMID = 0.5*self.umax
                if self.u > UMID:
                    self.eus0 = UMID - self.u  # compute error for back calculation term
                    self.ulim = UMID         # limit u to UMID
                elif self.u < -UMID:
                    self.eus0 = -self.u - UMID  # compute error for back calculation term
                    self.ulim = -UMID         # limit u to -UMID
                self.ulim += UMID
            else:
                if self.u > self.umax:
                    self.eus0 = self.umax - self.u  # compute error for back calculation term
                    self.ulim = self.umax         # limit u to umax
                elif self.u < -self.umax:
                    self.eus0 = -self.u - self.umax  # compute error for back calculation term
                    self.ulim = -self.umax         # limit u to -umax                
        return self.ulim 