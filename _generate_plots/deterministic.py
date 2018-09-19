import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv


from collections import defaultdict

def plot_box_plot(data, title, y_key, image_file):
	keys = data.keys()
	traces = []
	for k in keys:
		traces.append(Box(y=data[k], name=k, boxpoints='all', jitter=0.3, pointpos=-1.8))
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

def calculate_time_variability(data):

	# path_by_bucket = [( i // 10 + 10) for i in data['path_lenght_total_meters']]
	# print path_by_bucket

	path_by_bucket = {}


	for i in range( 0, len(data['computation_time_millis']), 1 ):
		# print data['path_lenght_total_meters'][i]
		bucket = (data['path_lenght_straight_line_meters'][i] // 10) * 10
		# print str(i) + " bucket " + str(bucket) + " for " + str(data['path_lenght_total_meters'][i]) + " @ " + str(data['dataset_name'][i] )
		
		if not str(bucket) in path_by_bucket:
			path_by_bucket[str(bucket)] =  []

		path_by_bucket[str(bucket)].append(data['computation_time_millis'][i]);

	print "Analysis of time in millis by lenght of straight line path in meters. buckets by 10 m"
	for key  in path_by_bucket:
		standard_deviation = np.std(path_by_bucket[str(key)]) 
		mean_ = np.mean(path_by_bucket[str(key)])
		print " = " +  str(key) + " = "
		print "Amount of data points " + str(   len(path_by_bucket[str(key)])   )
		print "Standard deviation: " + str( standard_deviation )
		print "Mean: " + str( mean_ )
		minimum_time =  min(path_by_bucket[str(key)])
		maximum_time =  max(path_by_bucket[str(key)])
		span = maximum_time - minimum_time
		percentage = standard_deviation * 100 / span
		print "Percentage: " + str(percentage)
		print "Span: " + str(span) + " millis"

		
def extract_lazy_theta_star_data(csv_filepath):
    data = pd.read_csv(csv_filepath)
    return data


def plot_scatter(dataSparse, dataOrtho):
	traceSparse = go.Scatter(
	    x = dataSparse['path_lenght_straight_line_meters'],
	    y = dataSparse['computation_time_millis'],
	    mode = 'markers',
        name='Sparse Neighbors (SN)'
	)

	traceOrtho = go.Scatter(
	    x = dataOrtho['path_lenght_straight_line_meters'],
	    y = dataOrtho['computation_time_millis'],
	    mode = 'markers',
        name='SN + Geometric obstacle avoidance'
	)
	data = [traceSparse, traceOrtho]

	layout = go.Layout(
	    title='Straight path (meters)',
	    xaxis=dict(
	        title='Time in milli seconds',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='Computation time (millis)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=data, layout=layout)
	plotly.offline.plot(fig, filename='./scatter_time_pathLenght.html', image='png')



def plot_finalLength_scatter(dataSparse, dataOrtho):
	traceSparse = go.Scatter(
	    x = dataSparse['path_lenght_total_meters'],
	    y = dataSparse['computation_time_millis'],
	    mode = 'markers',
        name='Sparse Neighbors (SN)'
	)

	traceOrtho = go.Scatter(
	    x = dataOrtho['path_lenght_total_meters'],
	    y = dataOrtho['computation_time_millis'],
	    mode = 'markers',
        name='SN + Geometric obstacle avoidance'
	)
	data = [traceSparse, traceOrtho]

	layout = go.Layout(
	    title='Final path (meters)',
	    xaxis=dict(
	        title='Path lenght (meters)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='Computation time (millis)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=data, layout=layout)
	plotly.offline.plot(fig, filename='./scatter_time_pathLenght.html', image='png')

def create_polyFitTrace(data):
	# calculate polynomial
	x_original = data['path_lenght_total_meters']
	y_original = data['computation_time_millis']

	z = np.polyfit(x_original, y_original, 2)
	f = np.poly1d(z)

	# calculate new x's and y's
	x_new = np.linspace(x_original[0], x_original[len(x_original)-1], 100)
	y_new = f(x_new)

	trace = go.Scatter(
                  x=x_new,
                  y=y_new,
                  mode='lines',
                  marker=go.Marker(color='rgb(31, 119, 180)'),
                  name='Fit'
                  )

	return trace


def plot_polynomialFit_scatter(dataSparse, dataOrtho):
	traceSparse = go.Scatter(
	    x = dataSparse['path_lenght_total_meters'],
	    y = dataSparse['computation_time_millis'],
	    mode = 'markers',
        name='Sparse Neighbors (SN)'
	)

	traceOrtho = go.Scatter(
	    x = dataOrtho['path_lenght_total_meters'],
	    y = dataOrtho['computation_time_millis'],
	    mode = 'markers',
        name='SN + Geometric obstacle avoidance'
	)

	trace_poly_sparse = create_polyFitTrace(dataSparse)
	trace_poly_ortho = create_polyFitTrace(dataOrtho)



	data = [traceSparse, traceOrtho, trace_poly_sparse, trace_poly_ortho]

	layout = go.Layout(
	    title='Final path (meters)',
	    xaxis=dict(
	        title='Path lenght (meters)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='Computation time (millis)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=data, layout=layout)
	plotly.offline.plot(fig, filename='./scatter_time_pathLenght.html', image='png')
	

def scatter_time_length (csv_filepathSparse, csv_filepathOrtho):
	dataSparse = extract_lazy_theta_star_data(csv_filepathSparse)
	dataOrtho = extract_lazy_theta_star_data(csv_filepathOrtho)
	plot_scatter(dataSparse, dataOrtho)


def calculate_time_variability_from_file(csv_filepath):
	lazyThetaStar_computation_data = extract_lazy_theta_star_data( csv_filepath )
	calculate_time_variability(lazyThetaStar_computation_data)


def scatter_time_finalLength (csv_filepathSparse, csv_filepathOrtho):
	dataSparse = extract_lazy_theta_star_data(csv_filepathSparse)
	dataOrtho = extract_lazy_theta_star_data(csv_filepathOrtho)
	plot_finalLength_scatter(dataSparse, dataOrtho)




# lazyThetaStar_filename = "/home/mfaria/Flying_Octomap_code/src/data/lazyThetaStar_computation_time.csv"
# calculate_time_variability_from_file(lazyThetaStar_filename)


# variables= {}
# execfile( "20180830_deterministic.py", variables )
