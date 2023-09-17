import pandas as pd

class CSVWriter:
    def __init__(self, filename, columns=None):
        self.filename = filename
        self.columns = columns
        self.data = []

    def add_row(self, row_data):
        self.data.append(row_data)

    def write_csv(self):
        data_df = pd.DataFrame(self.data, columns=self.columns)
        data_df.to_csv(self.filename, index=False)
        print(f"CSV file '{self.filename}' has been successfully created.")

    def clear_data(self):
        self.data = []