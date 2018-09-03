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
		bucket = (data['path_lenght_total_meters'][i] // 10) * 10
		# print str(i) + " bucket " + str(bucket) + " for " + str(data['path_lenght_total_meters'][i]) + " @ " + str(data['dataset_name'][i] )
		
		if not str(bucket) in path_by_bucket:
			path_by_bucket[str(bucket)] =  []

		path_by_bucket[str(bucket)].append(data['computation_time_millis'][i]);
	print "Analysis of time in millis by lenght of output path in meters. buckets by 10 m"
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

# def extract_lazy_theta_star_data(csv_filepath):

#     # with open(csv_filepath,"rb") as f:
#     #     reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
#     # 	data = defaultdict(list)
#     #     for row in reader:
#     #     	path_lenght = row[2]
#     #     	print row
#     #     	label = path_lenght//10
#     #     	dataset_name = str(label * 10) + " m"  
#     #     	data[dataset_name].append(row[0])
#     #     return data


# lazyThetaStar_filename = "/media/mfaria/Ganesha/20171219_backup_home_catec/Margarida/20180829_paper/data/20180830_unitTests/lazyThetaStar_computation_time.csv"

lazyThetaStar_filename = "/home/mfaria/Flying_Octomap_code/src/data/lazyThetaStar_computation_time.csv"
lazyThetaStar_computation_data = extract_lazy_theta_star_data( lazyThetaStar_filename )
# print lazyThetaStar_computation_data.keys
# plot_box_plot(lazyThetaStar_computation_data, 'Runtime variability for same intput', 'computation_time_millis', 'lazyThetaStar_time_box')
calculate_time_variability(lazyThetaStar_computation_data)

# variables= {}
# execfile( "20180830_deterministic.py", variables )
