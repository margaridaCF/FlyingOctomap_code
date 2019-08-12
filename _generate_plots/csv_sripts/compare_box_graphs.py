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

def plot_volume_exploration_vs_time(data, max_time):
	keys = data.keys()
	traces = []
	x_side = 12 - (-15)
	y_side = 20 - (-20)
	z_side = 14 - 1
	max_volume_m = x_side*y_side*z_side
	trace_max_volume = go.Scatter(
	    x=[0, max_time],
	    y=[max_volume_m, max_volume_m],
	    name='Geofence volume',
	    mode='lines',
	)
	traces.append(trace_max_volume)
	for k in keys:
		traces.append(Scatter(x = data[k]['time ellapsed millis'], y=data[k]['volume'], name=k, mode = 'lines'))
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
	fig=dict(data=traces, layout=layout)
	plotly.offline.plot(fig, filename='./volume_exploration_vs_time.html', image='png')


def plot_ram_vs_time (data):
	keys = data.keys()
	traces = []
	for k in keys:
		traces.append(Scatter(y=data[k]['RAM'], name=k, mode = 'lines'))

	layout = go.Layout(
	    title='Progression of RAM used',
	    xaxis=dict(
	        title='Time in seconds',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='RAM used in Bytes',
	        exponentformat = 'none',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=traces, layout=layout)
	plotly.offline.plot(fig, filename='./ram_vs_time.html', image='png')

def extract_ram_data():
	csv_filepath = "./../data/"+str(file)+"/ram.csv"
	ram_data = pd.read_csv(csv_filepath, delim_whitespace=True)
	# print ram_data
	# print ram_data['free']
	data = {}
	data['RAM'] = ram_data['used']
	return data

def extract_volume_data(csv_filepath):
	volume_data = pd.read_csv(csv_filepath)
	data = {}
	data['time ellapsed millis'] = volume_data['time ellapsed millis'];
	data['volume'] = volume_data['volume'];
	return data

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

# exploration_time.csv


frontier_computation_time_data = {}
lazyThetaStar_computation_data = {}
volume_data = {}
ram_data = {}
max_time = 0
for file in os.walk('./../data').next()[1]:
	frontier_filename = "./../data/"+str(file)+"/current/frontiers_computation_time.csv"
	frontier_computation_time_data[file] = extract_frontier_data( frontier_filename )
	lazyThetaStar_filename = "./../data/"+str(file)+"/lazyThetaStar_computation_time.csv"
	lazyThetaStar_computation_data[file] = extract_lazy_theta_star_data( lazyThetaStar_filename )
	volume_filename = "./../data/"+str(file)+"/volume_explored.csv"
	volume_data[file] = extract_volume_data( volume_filename )
	ram_data[file]    = extract_ram_data()
	max_time = max(volume_data[file]['time ellapsed millis'])
plot_box_plot(frontier_computation_time_data, 'Amount of time used for generating frontiers', 'computation_time_millis', 'frontiers_time_box')
plot_box_plot(lazyThetaStar_computation_data, 'Amount of time used for path planning with Lazy Theta Star', 'computation_time_millis', 'lazyThetaStar_time_box')
plot_ram_vs_time(ram_data)
plot_volume_exploration_vs_time(volume_data, max_time)


# variables= {}
# execfile( "compare_box_graphs.py", variables )
