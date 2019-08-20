import plotly.plotly as py
from plotly.graph_objs import *
import plotly
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import csv
import os
import plotly.io as pio

def loadRuns(folder_path):
    files = []
    # r=root, d=directories, f = files
    for r, d, f in os.walk(folder_path):
        for file in f:
            if "lazyThetaStar_computation_time.csv" in file:
                files.append(os.path.join(r, file))
    return files

def countSuccess(file_path):
    data = pd.read_csv(file_path)
    return pd.DataFrame({"success": data["success"][(data["success"] == 1) ].count(), "total_runs": data.success.count()}, index=[0])

def calculateObstacleDensity(row):
    return row.obstacle_hit_count * 100 / row.total_obstacle_checks

def plot_obstacleDensity_dynamicTraces(datasets, datasets_labels, title):
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
    pio.write_image(fig, '/media/mfaria/Ganesha/20171219_backup_home_catec/Margarida/20190100_exploration/sim_data/valid/obstacleDensity.pdf')
