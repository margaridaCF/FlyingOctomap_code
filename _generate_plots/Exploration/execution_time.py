import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv




def singleDatafram(data, column):
    new_df = data[['timeline',column]].dropna()
    new_df.reset_index(drop=True, inplace=True)
    return new_df

