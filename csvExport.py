import pandas as pd

class CSVWriter:
    def __init__(self, filename, columns=None):
        self.filename = filename
        self.columns = columns
        self.data = []

    def add_row(self, row_data):
        self.data.append(row_data)

    def write_csv(self,mode):
        data_df = pd.DataFrame(self.data, columns=self.columns)
        if(mode =='w'):
            print(f"CSV file '{self.filename}' has been successfully created.")
            hdr = True
        else:
            hdr = False
        data_df.to_csv(self.filename, mode=mode, index=False,header= hdr)
        self.__clear_data()

    def __clear_data(self):
        self.data = []