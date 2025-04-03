import numpy as np 

class control_software_new:
   
  m = 10
  b = 30 
  kp = 1000   ## large gains values because position is m not mm
  ki = 3000   
  limit = 20

  def __init__(self, dt :float):
    """
    Initialize control_software
    dt (float) = sampling time
    """

    self.dt = dt
    self.m = control_software_new.m
    self.b = control_software_new.b
    self.kp = control_software_new.kp 
    self.ki = control_software_new.ki
    self.limit = control_software_new.limit 
    self.sum_pos_error = 0.0
    self.sum_vel_error = 0.0

  def desired_eval_pi(self, sensed_force: float, vel_des: float, pos_des: float, vel_actual: float, pos_actual: float):
    """
    control_software to evaluate desired position using PI
    Sensed_force: float = Sensed Force from After surface contact model and sum    
    pos_des (float) = Desired position of effector
    vel_des (float) = Desired velocity of effector   
    pos_actual (float) = Actual position of effector
    vel_actual (float) = Actual velocity of effector    
    Function returns Desired Acceleration, next(step-after 1ms) velocity desired, next desired position
    """
    pos_error = pos_des - pos_actual
    vel_error = vel_des - vel_actual
    self.sum_pos_error += pos_error*self.dt
    self.sum_vel_error += vel_error*self.dt

    proportional_term = self.kp*pos_error + self.kp*vel_error 
    integral_term = self.ki*self.sum_pos_error + self.ki*self.sum_vel_error

    ##anti-windup and rate limiter(not used, it works needs a bit of tuning)
    """
    if (integral_term + proportional_term < self.limit): 
      if(integral_term +proportional_term > -self.limit):
        sensed_force_pi = sensed_force - integral_term - proportional_term
      else:
        sensed_force_pi = sensed_force - proportional_term + self.limit
    else:
      sensed_force_pi = sensed_force - proportional_term - self.limit
    """

    ##control law using PI
    sensed_force_pi = sensed_force - integral_term - proportional_term

    acc_des_next = (sensed_force_pi - self.b*vel_des)/self.m
    vel_des_next = vel_des + acc_des_next*self.dt
    pos_des_next = pos_des + vel_des_next*self.dt

    return acc_des_next, vel_des_next, pos_des_next
