# -*- coding: utf-8 -*-
import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv
import plotly.io as pio

from collections import defaultdict

        
def extract_lazy_theta_star_data(csv_filepath):
    data = pd.read_csv(csv_filepath)
    return data

def create_polyFitTrace(data, trace_color, polynomial_degree, start, label, legend=True):
    # calculate polynomial
    x_original = data['path_lenght_total_meters']
    y_original = data['computation_time_millis']

    z = np.polyfit(x_original, y_original, polynomial_degree)
    f = np.poly1d(z)

    # calculate new x's and y's
    x_new = np.linspace(start, max(x_original)+5, 120)
    y_new = f(x_new)
    degree_sym = u'\u00b0'.encode('utf-8').strip()
    trace = go.Scatter(
                  x=x_new,
                  y=y_new,
                  mode='lines',
                  marker=go.Marker(color=trace_color),
                  name=str(polynomial_degree)+degree_sym+' poly fit of '+label,
                  showlegend=legend
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
    trace_poly_sparse   = create_polyFitTrace(dataSparse, 'rgb(252,141,89)', polynomial_degree, start)
    trace_poly_ortho    = create_polyFitTrace(dataOrtho, 'rgb(145,191,219)', polynomial_degree, start)
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
    plotly.offline.iplot(fig, filename='./scatter_time_vs_pathLength_fit', image='png')
    

def plot_polynomialFit_dynamicTraces(datasets, datasets_labels, datsets_colors, polynomial_degree, start, title, fname, yMax, yUnits='seconds'):

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
        traces.append(create_polyFitTrace(datasets[x], datsets_colors[x], polynomial_degree, start, datasets_labels[x]))

    range__ = [0, yMax]
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
            title='Computation time ('+yUnits+')',
            range = [0, yMax],
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        )
    )
    fig=dict(data=traces, layout=layout)
    plotly.offline.iplot(fig, filename='./scatter_time_vs_pathLength_fit_'+fname, image='png')

def plot_scatter_dynamicTraces(datasets, datasets_labels, datsets_colors, title):
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
    plotly.offline.iplot(fig, filename='./scatter_time_vs_pathLength', image='png')
    pio.write_image(fig, '/home/mfaria/Downloads/plot_scatter_dynamicTraces.svg', scale=2)

def filterFailed(data):
    data.drop(data[data['success'] == 0].index, inplace=True)
    data.reset_index(drop=True, inplace=True)

def filterTooLong(data):
    data.drop(data[data['path_lenght_total_meters'] > 100].index, inplace=True)
    data.reset_index(drop=True, inplace=True)

def calculateObstacleDensity(row):
    return row.obstacle_hit_count * 100 / row.total_obstacle_checks


def plot_obstacleDensity_dynamicTraces(datasets, datasets_labels, datsets_colors, title):
    traces = []
    for x in xrange(0,len(datasets)):
        traces.append(
            Box(
                y=datasets[x].apply (lambda row: calculateObstacleDensity (row),axis=1), 
                name=datasets_labels[x], 
                boxpoints='all', 
                jitter=0.3, 
                pointpos=-1.8))

    
    layout = Layout(
        title=title,
        yaxis=dict(
            title='Obstacle presence (%)',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        ),
        #showlegend=False
    )
    fig=dict(data=traces, layout=layout)
    plotly.offline.iplot(fig)
    pio.write_image(fig, '/home/mfaria/Downloads/box_obstacleDensity.svg')

def plot_obstacleDetection(csv_filepathSparse, csv_filepathOrtho, margin, yMax, seconds=False):
    dataSparse = pd.read_csv(csv_filepathSparse)
    dataOrtho = pd.read_csv(csv_filepathOrtho)

    if(seconds):
        dataSparse.loc[:,'computation_time_millis'] /= 1000
        dataOrtho.loc[:,'computation_time_millis'] /= 1000
        yUnits = "seconds"
    else:
        yUnits = "millis"


    datasets_labels = ['3d Discretization', 'Geometric Definition']
    datsets_colors = ['rgb(252,141,89)', 'rgb(145,191,219)']
    polynomial_degree = 2
    start = 0
    title = "Processing time for obstacle detection with a "+str(margin)+"-meter safety margin"
    datasets = [dataSparse, dataOrtho]
    plot_polynomialFit_dynamicTraces(datasets, datasets_labels, datsets_colors, polynomial_degree, start, title, "obstDetect_"+str(margin)+"m", yMax, yUnits=yUnits)


# variables= {}
# execfile( "time_vs_pathLegth_lazyThetaStar.py", variables )
