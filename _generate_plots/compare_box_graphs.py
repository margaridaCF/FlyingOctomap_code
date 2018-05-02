import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd

def plot_box_plot(data, file_name):
	trace0 = Box(
	    y=data
	)
	data = [trace0]
	layout = go.Layout(
	    title='Amount of time used for exploration',
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
	fig=dict(data=[trace0], layout=layout)
	plotly.offline.plot(
    fig,
    filename='/media/mfaria/Ganesha/20171219_backup_home_catec/Margarida/20180226_sitl_ethz/plots/'+file_name+'.html',
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
	        title='Explored volume in cubuc meters',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=[trace0], layout=layout)
	plotly.offline.plot(fig, filename='/media/mfaria/Ganesha/20171219_backup_home_catec/Margarida/20180226_sitl_ethz/plots/volume_exploration_vs_time_'+dataset_name+'.html', image='png')

def extract_computation_time_path(csv_filepath):
    frontiers_computation_time = pd.read_csv(csv_filepath)
    frontiers_computation_time_millis = frontiers_computation_time['computation_time_millis']
    return frontiers_computation_time_millis

def run_frontiers_computation_time():
	data = extract_computation_time_path('../data/frontiers_computation_time.csv')
	plot_box_plot(data, "frontiers_computation_time_millis_box")

run_frontiers_computation_time()
plot_volume_exploration_vs_time('../data/volume_explored.csv', 'some obstacles')

# variables= {}
# execfile( "compare_box_graphs.py", variables )