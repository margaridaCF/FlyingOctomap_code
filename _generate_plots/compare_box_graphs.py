import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import os.path
import plotly.graph_objs as go

path = "/ros_ws/src/data"

def plot_box_plot(data, title, y_key, image_file):
	keys = data.keys()

	traces = []

	for k in keys:
		traces.append(Box(y=data[k][y_key], name=k, boxpoints='all', jitter=0.3, pointpos=-1.8))
	layout = go.Layout(
	    title=title,
	    # xaxis=dict(
	    #     title='Time in milli seconds',
	    #     titlefont=dict(
	    #         family='Courier New, monospace',
	    #         size=18,
	    #         color='#7f7f7f'
	    #     )
	    # ),
	    yaxis=dict(
	        title='Time in milliseconds',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=traces, layout=layout)
	plotly.offline.plot(
    fig,
    filename= './'+image_file+'.html',
    image='png')

def plot_volume_exploration_vs_time(csv_filepath, dataset_name):
	volume = pd.read_csv(csv_filepath)
	trace0 = go.Scatter(
	    x = volume['time ellapsed millis'],
	    y = volume['volume'],
	    mode = 'lines',
	    name = dataset_name
	)
	layout = go.Layout(
	    title='Progression of volume of explored space cases analyzed',
	    xaxis=dict(
	        title='Time in milli seconds',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='Explored volume in cubic meters',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=[trace0], layout=layout)
	plotly.offline.plot(fig, filename=path + '/volume_exploration_vs_time_'+dataset_name+'.html', image='png')

def extract_frontier_data(csv_filepath):
    frontiers_computation_time = pd.read_csv(csv_filepath)
    data = {}
    data['computation_time_millis'] = frontiers_computation_time['computation_time_millis']
    return data

def extract_lazy_theta_star_data(csv_filepath):
    frontiers_computation_time = pd.read_csv(csv_filepath)
    data = {}
    data['computation_time_millis'] = frontiers_computation_time['computation_time_millis']
    return data

def run_computation_time(csv_filepath):
	data = extract_computation_time_path(csv_filepath)
	plot_box_plot(data, "frontiers_computation_time_millis_box")

# # frontiers_computation_time.csv
# frontiers_csv_path = '../data/frontiers_computation_time.csv'
# run_computation_time(frontiers_csv_path)
# volume_explored.csv
# plot_volume_exploration_vs_time('./../data/volume_explored.csv', 'some obstacles')
# # lazyThetaStar_computation_time.csv
# ltstar_csv_path = '../data/lazyThetaStar_computation_time.csv'
# run_computation_time(ltstar_csv_path)

# exploration_time.csv

# variables= {}
# execfile( "compare_box_graphs.py", variables )

frontier_computation_time_data = {}
lazyThetaStar_computation_data = {}
for file in os.walk('./../data').next()[1]:
	frontier_filename = "./../data/"+str(file)+"/frontiers_computation_time.csv"
	frontier_computation_time_data[file] = extract_frontier_data( frontier_filename )
	lazyThetaStar_filename = "./../data/"+str(file)+"/lazyThetaStar_computation_time.csv"
	lazyThetaStar_computation_data[file] = extract_lazy_theta_star_data( lazyThetaStar_filename )
plot_box_plot(frontier_computation_time_data, 'Amount of time used for generating frontiers', 'computation_time_millis', 'frontiers_time_box')
plot_box_plot(lazyThetaStar_computation_data, 'Amount of time used for path planning with Lazy Theta Star', 'computation_time_millis', 'lazyThetaStar_time_box')
