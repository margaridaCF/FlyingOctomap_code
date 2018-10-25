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


def create_polyFitTrace(data, trace_color, polynomial_degree):
	# calculate polynomial
	x_original = data['path_lenght_total_meters']
	y_original = data['computation_time_millis']

	z = np.polyfit(x_original, y_original, polynomial_degree)
	f = np.poly1d(z)

	# calculate new x's and y's
	x_new = np.linspace(0, max(x_original)+5, 100)
	y_new = f(x_new)

	trace = go.Scatter(
                  x=x_new,
                  y=y_new,
                  mode='lines',
                  marker=go.Marker(color=trace_color),
                  name=str(polynomial_degree)+' degree polynomial fit'
                  )

	return trace


def plot_polynomialFit_scatter(dataSparse, dataOrtho, security_margin, polynomial_degree):
	# dataSparse.loc[:,'computation_time_millis'] /= 1000
	# dataOrtho.loc[:,'computation_time_millis'] /= 1000


	traceSparse = go.Scatter(
	    x = dataSparse['path_lenght_total_meters'],
	    y = dataSparse['computation_time_millis'],
	    mode = 'markers',
	    marker=go.Marker(color='rgb(44,123,182)'),
        name='Sparse Neighbors (SN)'
	)

	traceOrtho = go.Scatter(
	    x = dataOrtho['path_lenght_total_meters'],
	    y = dataOrtho['computation_time_millis'],
	    mode = 'markers',
	    marker=go.Marker(color='rgb(253,174,97)'),
        name='SN + Geometric obstacle avoidance'
	)

	trace_poly_sparse = create_polyFitTrace(dataSparse, 'rgb(171,217,233)', polynomial_degree)
	trace_poly_ortho = create_polyFitTrace(dataOrtho, 'rgb(255,255,191)', polynomial_degree)



	data = [traceSparse, traceOrtho, trace_poly_sparse, trace_poly_ortho]

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
	
def filterFailed(data):
	data.drop(data[data['success'] == 0].index, inplace=True)
	data.reset_index(drop=True, inplace=True)

# variables= {}
# execfile( "time_vs_pathLegth_lazyThetaStar.py", variables )
