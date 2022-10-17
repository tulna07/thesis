"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import csv
import pandas as pd

class Result_Log:
    def __init__(self, header_csv = ["header1","header2"]):
        self.results_data = []
        self.header_csv = header_csv
    
    ''' save result in to list '''
    def add_result(self, result):
        self.results_data.append (result)
    
    ''' set header '''
    def set_header (self, header_csv):
        self.header_csv = header_csv

    ''' write result to csv file '''
    def write_csv(self, file_name):
        f = open(file_name, 'w', newline='', encoding="utf-8")
        writer = csv.writer(f, delimiter=",")
        writer.writerow(self.header_csv)
        for result_items in self.results_data:
            writer.writerow(result_items)
        f.close()

    ''' read csv as dataframe '''
    def read_csv_as_dataframe(self, result_file):
        # read result as frame
        return pd.read_csv(result_file)