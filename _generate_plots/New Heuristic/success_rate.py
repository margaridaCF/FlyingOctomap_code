import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv


def calculate_success_rate(data, margin):
    count_success = data["success"][(data["success"] == 1) & (data["safety_margin_meters"] == margin) ].count()
    print "For margin " + str(margin) + " there were " + str(count_success) + " successes"
    count_total = data["success"][(data["safety_margin_meters"] == margin) ].count()
    success_rate = count_success * 100 / count_total
    return success_rate

def load_success_rate_fromFile(csv_filepath):
    data = pd.read_csv(csv_filepath)
    return load_success_rate_fromData(data)

def load_success_rate_fromData(data):
    margins = [3.9, 5, 5.4]
    return  [calculate_success_rate(data, margin) for margin in margins]


def plot_successRate_dynamicTraces(datasets, datasets_labels, datsets_colors, title):

    traces = []
    for x in xrange(0,len(datasets)):
        traces.append(Box(y=datasets[x], name=datasets_labels[x], boxpoints='all', jitter=0.3, pointpos=-1.8))

    
    layout = Layout(
        title=title,
        yaxis=dict(
            title='Success rate (%)',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        ),
        #showlegend=False
    )
    fig=dict(data=traces, layout=layout)
    plotly.offline.iplot(fig, filename='./box_successRate.html', image='png')

