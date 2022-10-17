"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import numpy as np
from Robot_math_lib import *

from enum import Enum
class Ranking_function(Enum):
    Angular_similarity = 0
    Cosine_similarity = 1
    RHS_RRT_base = 2

class Ranker:
    def __init__(self, alpha = 0.9, beta = 0.1, ranking_function =Ranking_function.Angular_similarity) :
        self.alpha = alpha
        self.beta = beta
        self.ranking_function = ranking_function
    
    def set_alpha(self, x): 
        self.alpha = x
    def set_beta(self, x): 
        self.beta = x
        
    def angular_similarity_score(self, angle= math.pi/2, distance = 1):
        # ranking = alpha/(distance**2) + beta/abs(angle**2)
        
        angle = angle / math.pi     # norm angle
        distance = distance / 100   # norm distance
        # print ("Ranking score: angle {0}, distance {1}".format(angle, distance) )
        score = float('inf')
        if not math.isclose(angle, 0.0) and not math.isclose(distance, 0.0):
            score = self.alpha / (distance) + self.beta / abs(angle)
            
        return score
    
    def consine_similarity_score(self, angle= math.pi/2, distance = 1):
        # ranking = alpha/(distance**2) + beta/abs(angle**2)
        
        angle = math.cos(angle)     # norm angle
        distance = distance / 100   # norm distance
        # print ("Ranking score: angle {0}, distance {1}".format(angle, distance) )
        score = float('inf')
        if not math.isclose(angle, 0.0) and not math.isclose(distance, 0.0):
            score = self.alpha / (distance) + self.beta / abs(angle)
            
        return score

    ''' ranking a given point  by its angle (from center to point and to goal) and its distance (to goal)'''
    def rank(self, center, point, goal, given_rhs = 0.0):

        vector_cg = np.subtract(center, goal)
        vector_pg = np.subtract(center, point)

        sa = signed_angle(vector_cg, vector_pg)
        dist = point_dist(goal, point)
        rank_score = None
        if self.ranking_function == Ranking_function.Cosine_similarity:
            rank_score =  self.consine_similarity_score(angle=sa, distance=dist)
        elif self.ranking_function == Ranking_function.Angular_similarity:
            rank_score = self.angular_similarity_score(angle=sa, distance=dist)
        elif self.ranking_function == Ranking_function.RHS_RRT_base:
            if given_rhs == 0:
                rank_score = float("Inf")
            else:
                rank_score = 1/given_rhs
        return [rank_score]

    @property
    def ranking_function(self):
        return self._ranking_function

    @ranking_function.setter
    def ranking_function(self, value):
        if not isinstance(value, Ranking_function):
            raise TypeError("ranking_function must be an instance of Ranking_function")
        self._ranking_function = value