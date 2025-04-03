import numpy as np 

class control_software:
  
  m = 10
  b = 30
  #k = 1000 
  
  def __init__(self, dt :float):
    """
    Initialize control_software
    dt (float) = sampling time
    """

    self.dt = dt
    self.b = control_software.b 
    self.m = control_software.m 

  def desired_eval(self, sensed_force: float, vel_des: float, pos_des: float):
    """
    control_software to evaluate desired position
    Sensed_force: float = Sensed Force from After surface contact model and sum
    pos_des (float) = Desired position of effector
    vel_des (float) = Desired velocity of effector
    Function returns Desired Acceleration, next(step-after 1ms) velocity desired, next desired position
    """
    acc_des = (sensed_force - self.b*vel_des)/self.m
    vel_des_next = vel_des + acc_des*self.dt
    pos_des_next = pos_des + vel_des_next*self.dt

    return acc_des, vel_des_next, pos_des_next
    
    
class robotic_arm:
  
  delay = 0.075

  def __init__(self, dt :float):
    """
    Initialize robotic arm
    dt (float) = sampling time
    """

    self.dt = dt
    self.delay = robotic_arm.delay

  def update_next(self, pos_actual: float, pos_des: float):
    """
    robotic arm to simulate next actual position
    pos_actual (float) = Actual position of effector
    pos_des (float) = Desired position of effector 
    Function returns next actual position, next velocity actual  
    """
    rate = self.dt/(self.dt + self.delay)
    pos_actual_next = pos_actual + rate*(pos_des - pos_actual)
    vel_actual = (pos_actual_next - pos_actual)/self.dt
    
    return pos_actual_next, vel_actual