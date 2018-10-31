import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv


def calculate_success_rate(data, margin):
	count_success = data["success"][(data["success"] == 1) & (data["safety_margin_meters"] == margin) ].count()
	count_total = data["success"][(data["safety_margin_meters"] == 5) ].count()
	success_rate = count_success * 100 / count_total
	return success_rate