import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv


from collections import defaultdict

		
def extract_lazy_theta_star_data(csv_filepath):
    data = pd.read_csv(csv_filepath)
    return data

def create_polyFitTrace(data, trace_color, polynomial_degree, start):
	# calculate polynomial
	x_original = data['path_lenght_total_meters']
	y_original = data['computation_time_millis']

	z = np.polyfit(x_original, y_original, polynomial_degree)
	f = np.poly1d(z)

	# calculate new x's and y's
	x_new = np.linspace(start, max(x_original)+5, 120)
	y_new = f(x_new)

	trace = go.Scatter(
                  x=x_new,
                  y=y_new,
                  mode='lines',
                  marker=go.Marker(color=trace_color),
                  name=str(polynomial_degree)+' degree polynomial fit'
                  )

	return trace


def plot_polynomialFit_scatter(dataOriginal, dataSparse, dataOrtho, security_margin, polynomial_degree, start):
	traceOriginal = go.Scatter(
	    x = dataOriginal['path_lenght_total_meters'],
	    y = dataOriginal['computation_time_millis'],
	    mode = 'markers',
	    marker=go.Marker(color='rgb(255,255,191)'),
        name='Lazy Theta * Offline'
	)

	traceSparse = go.Scatter(
	    x = dataSparse['path_lenght_total_meters'],
	    y = dataSparse['computation_time_millis'],
	    mode = 'markers',
	    marker=go.Marker(color='rgb(252,141,89)'),
        name='Sparse Neighbors (SN)'
	)

	traceOrtho = go.Scatter(
	    x = dataOrtho['path_lenght_total_meters'],
	    y = dataOrtho['computation_time_millis'],
	    mode = 'markers',
	    marker=go.Marker(color='rgb(145,191,219)'),
        name='SN + Geometric obstacle avoidance'
	)
	trace_poly_sparse 	= create_polyFitTrace(dataSparse, 'rgb(252,141,89)', polynomial_degree, start)
	trace_poly_ortho 	= create_polyFitTrace(dataOrtho, 'rgb(145,191,219)', polynomial_degree, start)
	trace_poly_original = create_polyFitTrace(dataOriginal, 'rgb(255,255,191)', polynomial_degree, start)

	data = [traceSparse, traceOrtho, traceOriginal, trace_poly_sparse, trace_poly_ortho, trace_poly_original]
	layout = go.Layout(
	    title='Time to find path with ' + security_margin + ' m security margin',
	    xaxis=dict(
	        title='Distance between points (in meters)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    ),
	    yaxis=dict(
	        title='Computation time (seconds)',
	        titlefont=dict(
	            family='Courier New, monospace',
	            size=18,
	            color='#7f7f7f'
	        )
	    )
	)
	fig=dict(data=data, layout=layout)
	plotly.offline.iplot(fig, filename='./scatter_time_pathLenght.html', image='png')
	

def plot_polynomialFit_dynamicTraces(datasets, datasets_labels, datsets_colors, security_margin, polynomial_degree, start, title):

    traces = []
    for x in xrange(0,len(datasets)):
        traces.append(go.Scatter(
                x = datasets[x]['path_lenght_total_meters'],
                y = datasets[x]['computation_time_millis'],
                mode = 'markers',
                marker=go.Marker(color=datsets_colors[x]),
                name=datasets_labels[x]
            )
        )
        traces.append(create_polyFitTrace(datasets[x], datsets_colors[x], polynomial_degree, start))

    
    layout = go.Layout(
        title=title,
        xaxis=dict(
            title='Distance between points (in meters)',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        ),
        yaxis=dict(
            title='Computation time (seconds)',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        )
    )
    fig=dict(data=traces, layout=layout)
    plotly.offline.iplot(fig, filename='./scatter_time_pathLenght.html', image='png')

def filterFailed(data):
	data.drop(data[data['success'] == 0].index, inplace=True)
	data.reset_index(drop=True, inplace=True)

def filterTooLong(data):
	data.drop(data[data['path_lenght_total_meters'] > 100].index, inplace=True)
	data.reset_index(drop=True, inplace=True)

# variables= {}
# execfile( "time_vs_pathLegth_lazyThetaStar.py", variables )
