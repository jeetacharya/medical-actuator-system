import numpy as np

class ResidualGenerator:
    '''
    Calculates difference between real actuator and digital twin
    '''
    @staticmethod
    def position(real, twin):
        return real - twin
    
    @staticmethod
    def velocity(real, twin):
        return real - twin