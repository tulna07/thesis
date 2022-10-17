"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import pandas as pd
import numpy as np
import argparse
import matplotlib.pyplot as plt
from sklearn.feature_extraction import img_to_graph
from Robot_math_lib import set_figure_name
from Program_config import *
from Result_log import Result_Log
class Experimental_Astar_Asp(Result_Log):
    def __init__(self) -> None:
        super().__init__()

    ''' visualizate result from result file '''
    def result_plot(self, result_file):
        # read result as frame
        result_data = pd.read_csv(result_file)
        
        df_time = result_data[['asp_time', 'Astar_time']]
        df_time.plot(logy=True, kind="bar")
        #data = np.log2(df_time[['asp_time', 'Astar_time']])
        
        #data.plot.bar()
        #data.
        df_path_cost = result_data[['asp_path_cost', 'Astar_path_cost']]
        df_path_cost.plot(kind="bar")
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-r', metavar="result file",  help='result file', default="result.csv")

    menu_result = parser.parse_args()

    # get user input
    result_file = menu_result.r

    result = Experimental_Astar_Asp()
    result.result_plot(result_file=result_file)