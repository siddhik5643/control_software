import numpy as np 

class surface_contact_model:
  
  m = 10
  b = 30 
  k = 100000
  
  def __init__(self, initial_pos: float, dt :float):
    """
    Initialize surface_contact_model
    initial: float = initial postion ( equilibrium point )
    dt (float) = sampling time
    """

    self.initial_pos = initial_pos
    self.dt = dt
    self.b = surface_contact_model.b 
    self.m = surface_contact_model.m 
    self.k = surface_contact_model.k

  def update_forward(self, constant_force: float, pos_actual: float, vel_actual: float):
    """
    surface contact model
    constant_force: float = constant_force
    pos_actual (float) = Actual position of effector
    vel_actual (float) = Actual velocity of effector
    Function returns value of sensed force in Newtons
    """
    if (pos_actual < self.initial_pos):
      sensed_force_cal = constant_force - self.b*vel_actual - self.k*(pos_actual - self.initial_pos)
    else:
      sensed_force_cal = constant_force

    return sensed_force_cal
    
    