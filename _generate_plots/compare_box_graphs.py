
import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd

# plotly.tools.set_credentials_file(username='kotoko', api_key='')

# pd.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
# figsize(15, 5)

def plot_box_plot(data, file_name):
	print data
	trace0 = Box(
	    y=data
	)
	data = [trace0]
	plotly.offline.plot(data)
	# py.iplot(data, filename='jupyter/basic_bar')
	plotly.offline.plot(
    data, 
    filename='/media/mfaria/Ganesha/20171219_backup_home_catec/Margarida/20180226_sitl_ethz/plots/'+file_name+'.html',
    image='png')

def extract_computation_time_path(csv_filepath):
    frontiers_computation_time = pd.read_csv(csv_filepath)
    frontiers_computation_time_millis = frontiers_computation_time['computation_time_millis']
    return frontiers_computation_time_millis

def run_frontiers_computation_time():
	data = extract_computation_time_path('../data/frontiers_computation_time.csv')
	plot_box_plot(data, "frontiers_computation_time_millis_box")

run_frontiers_computation_time()

# variables= {}
# execfile( "compare_box_graphs.py", variables )