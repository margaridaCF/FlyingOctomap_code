import csv
import plotly
import plotly.plotly as py
# from plotly.graph_objs import *
import plotly.graph_objs as go 
import collections 


# PLOT
def plot_pie_iteration_distribution(iteration_dict, title, file_name, total_cases_analyzed):
    colors = ['#e66101', '#fdb863', '#b2abd2', '#5e3c99']
    trace = go.Pie(
        labels=iteration_dict.keys(), 
        values=iteration_dict.values(),
        hole=0.4,
        textfont=dict(color='#ffffff'),
        marker=dict(colors=colors, 
                           line=dict(color='#ffffff', width=2)))
    plotly.offline.plot(
    {
        "data": [trace],
        "layout": {
            "title":"Lazy Theta* iterations for "+title+"<br>"+str(total_cases_analyzed)+" cases analyzed",
            "annotations": [
                {
                    "font": {
                        "size": 20
                    },
                    "showarrow": False,
                    "text": "10m free line",
                }
            ]
        }
    }, 
    filename='/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+file_name+'.html',
    image='png')

def plot_voxelSizeDistribution_scatter (voxel_size, count, run):
    plotly.offline.plot({
        "data": [Bar(x=voxel_size, y=count)],
        "layout": Layout(title="Voxel Size Distribution @ "+run)
        
    }, filename='plots/voxelSizeDistribution_'+run)

def plot_path_bubble(waypoints, closed, run):
    trace_closed = Scatter3d(
        x = closed['x'],
        y = closed['y'],
        z = closed['z'],
        mode = 'markers',
        marker = dict(
            sizemode = 'diameter',
            sizeref = 0.009,
            size = closed['size'],
            symbol= "square"
            )  
    )
    trace_waypoints = Scatter3d(
        x = waypoints['x'],
        y = waypoints['y'],
        z = waypoints['z'],
        mode = 'markers',
        marker = dict(
            sizemode = 'diameter',
            sizeref = 0.009,
            size = waypoints['size'],
            symbol= "square"
            )  
    )
    data=[trace_closed, trace_waypoints]

    layout=Layout(title = 'Analyzed voxels to find path in '+run,
                  scene = dict(xaxis=dict(title='X',
                                          titlefont=dict(color='Orange')),
                                yaxis=dict(title='Y',
                                           titlefont=dict(color='rgb(220, 220, 220)')),
                                zaxis=dict(title='Z',
                                           titlefont=dict(color='rgb(220, 220, 220)')),
                                bgcolor = 'rgb(20, 24, 54)'
                               )
                 )

    fig=dict(data=data, layout=layout)
    plotly.offline.plot(fig, filename='analyzed_voxels.html')

def plot_one_size(iteration_count, voxel_count, frequency, run):
    # Voxel with 0.2 size
    trace = go.Scatter(
        x = iteration_count,
        y = voxel_count,
        mode = 'markers',
        marker=dict(size=frequency)
    )
    data = [trace]
    layout = go.Layout (title = 'For voxels with 0,2 size '+run,
        xaxis=dict(
            title='Iteration count'
            ),
        yaxis=dict(
            title='Voxel count'
            ),
        )
    plotly.offline.plot({
        "data": data,
        "layout": layout,        
        },
        filename= 'plots/voxelSizeDistribution_'+run+'.html')

def plot_iteration_voxelSize_frequency(data, run):
    totalVoxels_per_iterationCount = {}
    iteration_labels = sorted(data[0.2].keys())
    size_labels = sorted(data.keys())
    # for it_count in iteration_labels:
    #     it_line = str(it_count)+ ": "
    #     for size_label in size_labels:
    #         if it_count in data[size_label].keys():
    #             # print data[size_label][it_count]
    #             it_line += " ["+ str(size_label)+ "] "+ str(data[size_label][it_count])
    #     print it_line
        
    # To Percentage
    for it_count in iteration_labels:

        for size_label in size_labels:

            if it_count in data[size_label].keys():
                if not it_count in totalVoxels_per_iterationCount.keys():
                    totalVoxels_per_iterationCount[it_count] = 0
                totalVoxels_per_iterationCount[it_count] += data[size_label][it_count]
    for size_label in size_labels:
        for it_count in data[size_label].keys():
            data[size_label][it_count] = data[size_label][it_count]*100/totalVoxels_per_iterationCount[it_count]

    traces = []
    for size_label in size_labels:
        sorted_iteration = sorted(data[size_label].keys())
        trace = go.Bar(
            x = data[size_label].keys(),
            y = data[size_label].values(),
            # mode = 'markers',
            name = size_label
        )
        traces.append(trace)

    layout = go.Layout (title = run,
        xaxis=dict(
            title='Iteration count',
            ),
        yaxis=dict(
            title='Voxel count'
            ),
        barmode='stack'
        )
    plotly.offline.plot({
        "data": traces,
        "layout": layout,        
        },
        filename= 'plots/voxelSizeDistribution_'+run+'.html')

def plot_pointVisualization_longDebug(X, Y, Z, X_A, Y_A, Z_A, prev_s_coord, current_s_coord, 
    start_coordinates, goal_point_coordinates, s_stream, run ):
    s_X = []
    s_Y = []
    s_Z = []
    for s in s_stream:
        s_X.append(s[0])
        s_Y.append(s[1])
        s_Z.append(s[2])
    trace_s_stream = go.Scatter3d(
        x=s_X,
        y=s_Y,
        z=s_Z,
        mode='markers',
        name = "S stream",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_raw = go.Scatter3d(
        x=X,
        y=Y,
        z=Z,
        mode='markers',
        name = "Raw neighbors",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_centers = go.Scatter3d(
        x=X_A,
        y=Y_A,
        z=Z_A,
        mode='markers',
        name = "Voxel centers",
        marker=dict(
            size=12,
        )
    )
    # trace_prev_s_neighbors_raw = go.Scatter3d(
    #     x=prev_s_X,
    #     y=prev_s_Y,
    #     z=prev_s_Z,
    #     mode='markers',
    #     name = "Previous s raw neighbors",
    #     marker=dict(
    #         size=12,
    #     )
    # )
    # trace_prev_s_neighbors_centers = go.Scatter3d(
    #     x=prev_s_X_A,
    #     y=prev_s_Y_A,
    #     z=prev_s_Z_A,
    #     mode='markers',
    #     name = "Previous s voxel centers ",
    #     marker=dict(
    #         size=12,
    #     )
    # )
    trace_prev_s = go.Scatter3d(
        x=(prev_s_coord[0], ),
        y=(prev_s_coord[1], ),
        z=(prev_s_coord[2], ) ,
        mode='markers',
        name = "Previous s",
        marker=dict(
            size=12,
        )
    )
    trace_current_s = go.Scatter3d(
        x=(current_s_coord[0], ),
        y=(current_s_coord[1], ),
        z=(current_s_coord[2], ) ,
        mode='markers',
        name = "Current s",
        marker=dict(
            size=12,
        )
    )
    # trace_target = go.Scatter3d(
    #     x=(correct_neighbor_coordinates['point'][0], correct_neighbor_coordinates['center'][0]),
    #     y=(correct_neighbor_coordinates['point'][1], correct_neighbor_coordinates['center'][1]),
    #     z=(correct_neighbor_coordinates['point'][2], correct_neighbor_coordinates['center'][2]) ,
    #     mode='markers',
    #     name = "Correct neighbor for path",
    #     marker=dict(
    #         size=12,
    #     )
    # )
    trace_start_center = go.Scatter3d(
        x=(start_coordinates[0], ),
        y=(start_coordinates[1], ),
        z=(start_coordinates[2], ) ,
        mode='markers',
        name = "Start center",
        marker=dict(
            size=12,
        )
    )
    #the distance takes into account the final point, not it's center
    trace_end_point = go.Scatter3d(
        x=(goal_point_coordinates[0], ),
        y=(goal_point_coordinates[1], ),
        z=(goal_point_coordinates[2], ) ,
        mode='markers',
        name = "End POINT",
        marker=dict(
            size=12,
        )
    )
    traces = [trace_neighbors_raw, trace_neighbors_centers, trace_prev_s, trace_current_s, trace_start_center, trace_end_point, trace_s_stream]
    layout = go.Layout(
    )
    plotly.offline.plot({
            "data": traces,
            "layout": layout,        
            },
            filename= 'plots/voxelSizeDistribution_'+run+'.html')

def plot_pointVisualization_eurocRun1(X, Y, Z, X_A, Y_A, Z_A, prev_s_coord, current_s_coord, 
    start_coordinates, goal_point_coordinates, s_stream, run, open_X, open_Y, open_Z ):
    s_X = []
    s_Y = []
    s_Z = []
    for s in s_stream:
        s_X.append(s[0])
        s_Y.append(s[1])
        s_Z.append(s[2])
    trace_s_stream = go.Scatter3d(
        x=s_X,
        y=s_Y,
        z=s_Z,
        mode='markers',
        name = "S stream",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_raw = go.Scatter3d(
        x=X,
        y=Y,
        z=Z,
        mode='markers',
        name = "Raw neighbors",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_centers = go.Scatter3d(
        x=X_A,
        y=Y_A,
        z=Z_A,
        mode='markers',
        name = "Voxel centers",
        marker=dict(
            size=12,
        )
    )
    # trace_prev_s_neighbors_raw = go.Scatter3d(
    #     x=prev_s_X,
    #     y=prev_s_Y,
    #     z=prev_s_Z,
    #     mode='markers',
    #     name = "Previous s raw neighbors",
    #     marker=dict(
    #         size=12,
    #     )
    # )
    trace_open = go.Scatter3d(
        x=open_X,
        y=open_Y,
        z=open_Z,
        mode='markers',
        name = "Top nodes on open ",
        marker=dict(
            size=12,
        )
    )
    trace_prev_s = go.Scatter3d(
        x=(prev_s_coord[0], ),
        y=(prev_s_coord[1], ),
        z=(prev_s_coord[2], ) ,
        mode='markers',
        name = "Previous s",
        marker=dict(
            size=12,
        )
    )
    trace_current_s = go.Scatter3d(
        x=(current_s_coord[0], ),
        y=(current_s_coord[1], ),
        z=(current_s_coord[2], ) ,
        mode='markers',
        name = "Current s",
        marker=dict(
            size=12,
        )
    )
    # trace_target = go.Scatter3d(
    #     x=(correct_neighbor_coordinates['point'][0], correct_neighbor_coordinates['center'][0]),
    #     y=(correct_neighbor_coordinates['point'][1], correct_neighbor_coordinates['center'][1]),
    #     z=(correct_neighbor_coordinates['point'][2], correct_neighbor_coordinates['center'][2]) ,
    #     mode='markers',
    #     name = "Correct neighbor for path",
    #     marker=dict(
    #         size=12,
    #     )
    # )
    trace_start_center = go.Scatter3d(
        x=(start_coordinates[0], ),
        y=(start_coordinates[1], ),
        z=(start_coordinates[2], ) ,
        mode='markers',
        name = "Start center",
        marker=dict(
            size=12,
        )
    )
    #the distance takes into account the final point, not it's center
    trace_end_point = go.Scatter3d(
        x=(goal_point_coordinates[0], ),
        y=(goal_point_coordinates[1], ),
        z=(goal_point_coordinates[2], ) ,
        mode='markers',
        name = "End POINT",
        marker=dict(
            size=12,
        )
    )
    traces = [trace_s_stream, trace_start_center, trace_end_point, trace_open, trace_neighbors_raw, trace_neighbors_centers, trace_prev_s, trace_current_s]
    layout = go.Layout(
    )
    plotly.offline.plot({
            "data": traces,
            "layout": layout,        
            },
            filename= 'plots/voxelSizeDistribution_'+run+'.html')
  
def plot_3Dwaypoint_neighbors_center_goal_start():
    X = ( -2.5, -2.5, -2.5, -2.5, -2.300000191, -2.699999809, )
    Y = ( -3.900000095, -3.900000095, -4.099999905, -3.700000286, -3.900000095, -3.900000095, )
    Z = ( 3.100000143, 3.499999762, 3.299999952, 3.299999952, 3.299999952, 3.299999952, )
    X_A = ( -2.599999905, -2.5, -2.700000048, )
    Y_A = ( -3.799999952, -4.099999905, -3.900000095, )
    Z_A = ( 3, 3.299999952, 3.299999952, )
    X_UNKNOWN = ( -2.5, -2.5, -2.300000191, )
    Y_UNKNOWN = ( -3.900000095, -3.700000286, -3.900000095, )
    Z_UNKNOWN = ( 3.499999762, 3.299999952, 3.299999952, )



    trace_neighbors_unknown = go.Scatter3d(
        x= X_UNKNOWN,
        y= Y_UNKNOWN,
        z= Z_UNKNOWN,
        mode='markers',
        name = "Unknown",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_raw = go.Scatter3d(
        x= X,
        y= Y,
        z= Z,
        mode='markers',
        name = "Raw",
        marker=dict(
            size=12,
        )
    )
    trace_neighbors_centers = go.Scatter3d(
        x= X_A,
        y= Y_A,
        z= Z_A,
        mode='markers',
        name = "Centers",
        marker=dict(
            size=12,
        )
    )
    trace_center = go.Scatter3d(
        x=(-2.5, ),
        y=(-3.9, ),
        z=(3.3, ) ,
        mode='markers',
        name = "Start center",
        marker=dict(
            size=12,
        )
    )
    goal_point_coordinates = (2.0, 5.2, 1.2)
    trace_end_point = go.Scatter3d(
        x=(goal_point_coordinates[0], ),
        y=(goal_point_coordinates[1], ),
        z=(goal_point_coordinates[2], ) ,
        mode='markers',
        name = "End POINT",
        marker=dict(
            size=12,
        )
    )

    traces = [trace_neighbors_centers, trace_neighbors_raw, trace_center, trace_neighbors_unknown, trace_end_point]
    layout = go.Layout(
    )
    plotly.offline.plot({
            "data": traces,
            "layout": layout,        
            },
            filename= 'plots/3d_neighbors.html')

# CSV EXTRACTION
def extract_data_ManyPathsPerFile_iterationDistribution_group(csv_filepath, groups_highToLow):
    iteration_dict = {}
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        iteration_dict = {}
        for row in reader:
            for max_i in range(0, len(groups_highToLow)):
                if row[0] > groups_highToLow[max_i]:
                    min_bound = groups_highToLow[max_i]
                    max_bound = groups_highToLow[max_i-1]
                    if   (max_i == 1):
                        # print row[0]
                        # label = "[" + str(min_bound+1) +" +inf ["
                        label = "[" + '{0:g}'.format(float(min_bound+1)) +" +inf ["
                    else:
                        lable = "[ " + '{0:g}'.format(float(min_bound+1)) + ", " + '{0:g}'.format(float(max_bound+1)) + " ]"
                    if lable in iteration_dict:
                        iteration_dict[lable] = iteration_dict[lable] + 1;
                    else:
                        iteration_dict[lable] = 1;
                    break

        return iteration_dict

def extract_data_ManyPathsPerFile_iterationDistribution(csv_filepath):
    iteration_dict = {}
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        iteration_dict = {}
        count = 0;
        for row in reader:
            count +=1;
            if row[0] in iteration_dict:
                iteration_dict[row[0]] = iteration_dict[row[0]] + 1;
            else:
                iteration_dict[row[0]] = 1;
        print str(count) + " paths analyzed"
        iteration_dict["total"] = count

        return iteration_dict

def extract_data_FixedItCount_VoxelSizeDistribution(csv_filepath):
    with open(csv_filepath,"rb") as f:
        sizes = {}
        total_voxels = 0
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            iteration_count = row[0]
            if(iteration_count == 5):
                hetrogenity = (len(row)-1)/2
                for h in range(hetrogenity):
                    size_label = row[1+(h)*2]
                    if not size_label in sizes:
                        sizes[size_label] = 0
                    sizes[size_label] = row[2+(h*2)] + sizes[size_label]
                    total_voxels = total_voxels + sizes[size_label]
        return {'total_voxels': total_voxels, 'sizes':sizes}

def extract_data_path(csv_filepath):
    iteration_dict = {}
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        x = []
        y = []
        z = []
        size = []
        count = 0;
        for row in reader:
            x.append(row[0])
            y.append(row[1])
            z.append(row[2])
            size.append(row[3])
            count += 1
        print str(count) + " waypoints in path"

        return {'x':x, 'y':y, 'z':z, 'size':size}

def extract_voxelSize_Distributuion(csv_filepath):
    iteration_dict = {}
    sizes = {}
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        sizes = {}
        sizes_histogram =  {}
        for row in reader:
            hetrogenity = (len(row)-1)/2
            for h in range(hetrogenity):
                size_label = row[1+(h)*2]
                if not size_label in sizes:
                    sizes[size_label] = []
                # print row
                sizes[size_label].append( (row[0], row[2+(h*2)]) )
        for k in sizes.keys():
            cnt = collections.Counter(sizes[k])
            common = cnt.most_common(10000)
            frequency = [i[1] for i in common]
            pairs = [i[0] for i in common]
            voxel_count = [i[1] for i in pairs]
            iteration_count = [i[0] for i in pairs]
            sizes_histogram[k] = {'frequency': frequency, 'voxel_count': voxel_count, 'iteration_count': iteration_count}
            # print k, ': ', sizes_histogram[k]
            # print iteration_count
            # print voxel_count
            # print frequency
    return sizes_histogram

def extract_voxelSize_iterationCount_voxelSum(csv_filepath):
    data = {}
    # CSV  run (iteration count, distribution of voxels)
    # DATA [voxel size] [iteration count] = sum of voxel count (Fixed iteration & size) 
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)

        for row in reader:
            hetrogenity = (len(row)-1)/2
            for h in range(hetrogenity):
                size_label = row[1+(h)*2]
                voxel_count = row[2+h*2]
                iteration_count = row[0]
                if not size_label in data:
                    data[size_label] = {}
                if not iteration_count in data[size_label]:
                    data[size_label][iteration_count] = 0
                data[size_label][iteration_count] = data[size_label][iteration_count] + voxel_count

            if len(row) > 7:
                print row
                iteration_labels = sorted(data[0.2].keys())
                size_labels = sorted(data.keys())
                for it_count in iteration_labels:
                    it_line = str(it_count)+ ": "
                    for size_label in size_labels:
                        if it_count in data[size_label].keys():
                            # print data[size_label][it_count]
                            it_line += " ["+ str(size_label)+ "] "+ str(data[size_label][it_count])
                    print it_line
                return data
    return data

# ANALYSIS    
def analyzedVoxels():
    dataset_name = "long_path"
    closed_csv = 'waypoints_closed'
    waypoints_csv = "waypoints_final_path"
    closed = extract_data_path('/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+closed_csv+'.csv')
    waypoints = extract_data_path('/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+waypoints_csv+'.csv')
    plot_path_bubble(waypoints, closed, dataset_name)

def analyzeIterationCsvFile(dataset_name, groups_highToLow, dataset_displayName):
    result_dict = extract_data_ManyPathsPerFile_iterationDistribution('/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+dataset_name+'.csv')
    result_dict_group = extract_data_ManyPathsPerFile_iterationDistribution_group('/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+dataset_name+'.csv', groups_highToLow)
    plot_pie_iteration_distribution(result_dict, dataset_displayName, dataset_name+'_raw', result_dict["total"])
    plot_pie_iteration_distribution(result_dict_group, dataset_displayName, dataset_name+'_group', result_dict["total"])

def run_analyzeIterationCsvFile():
    groups_highToLow_back = [1000000000, 100, 50, 10, 5, 4, 1]
    analyzeIterationCsvFile('iterations_voxelDistributionoffShoreOil_2Obst_1m_back', groups_highToLow_back)

def run_extract_voxelSize_iterationCount_voxelSum():
    dataset_name = "Stright line, 1m, no obstacles"
    csv_name = '1m/iterations_cellDistributionoffShoreOil_2Obst_.csv'
    # dataset_name = "Test"
    # csv_name = 'test_2.csv'
    data = extract_voxelSize_iterationCount_voxelSum('/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/'+csv_name)
    plot_iteration_voxelSize_frequency(data, dataset_name)

def run_plot_pointVisualization_longDebug():
    run = "neighbors"
    s39_X = ( -3.300000191, -3.500000238, -3.500000238, -3.500000238, -3.700000048, -3.500000238, -3.700000048, -3.300000191, -3.500000238, -3.500000238, -3.100000143, -3.100000143, -3.500000238, -3.700000048, -3.100000143, -3.300000191, -3.300000191, -3.300000191, -3.300000191, -3.700000048, -3.100000143, -3.300000191, -3.500000238, -3.300000191, )
    s39_Y = ( -4.900000095, -5.299999714, -4.900000095, -4.700000286, -5.099999905, -5.099999905, -5.099999905, -4.900000095, -5.099999905, -4.700000286, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.299999714, )
    s39_Z = ( 0.3000000417, 0.7000000477, 0.3000000417, 0.7000000477, 0.7000000477, 0.3000000417, 0.5000000596, 0.9000000358, 0.9000000358, 0.5000000596, 0.7000000477, 0.5000000596, 0.9000000358, 0.5000000596, 0.5000000596, 0.5000000596, 0.5000000596, 0.9000000358, 0.3000000417, 0.7000000477, 0.7000000477, 0.7000000477, 0.5000000596, 0.7000000477, )
    s39_X_A = ( -3.400000095, -3.400000095, -3.599999905, -3.400000095, -3.700000048, -3.599999905, -3.599999905, -3.599999905, -3, -3, -3.599999905, -3.700000048, -3, -3.599999905, -3.299999952, -3.599999905, -3.400000095, -3.700000048, -3, -3.599999905, -3.5, )
    s39_Y_A = ( -5, -5, -4.400000095, -5, -5.099999905, -5.199999809, -5.199999809, -4.400000095, -5, -5, -5.199999809, -4.900000095, -5, -4.400000095, -5.300000191, -5.199999809, -5, -4.900000095, -5, -4.400000095, -5.300000191, )
    s39_Z_A = ( 0.200000003, 0.200000003, 0.400000006, 0.200000003, 0.5, 1.200000048, 1.200000048, 0.400000006, 0.6000000238, 0.6000000238, 1.200000048, 0.5, 0.6000000238, 0.400000006, 0.5, 1.200000048, 0.200000003, 0.6999999881, 0.6000000238, 0.400000006, 0.5, )

    s223_X = ( -4.099999905, -3.299999952, -3.299999952, -3.299999952, -3.099999905, -4.099999905, -3.5, -3.5, -3.5, -3.5, -3.099999905, -4.099999905, -3.5, -3.5, -3.5, -3.5, -3.099999905, -4.099999905, -3.5, -3.5, -3.5, -3.099999905, -4.099999905, -3.5, -3.5, -3.699999809, -3.299999952, -3.699999809, -3.299999952, -3.299999952, -3.699999809, -3.099999905, -4.099999905, -3.899999857, -3.899999857, -3.299999952, -3.699999809, -3.899999857, -4.099999905, -3.699999809, -3.099999905, -4.099999905, -3.099999905, -3.299999952, -3.099999905, -3.699999809, -3.099999905, -3.099999905, -3.299999952, -3.099999905, -4.099999905, -3.699999809, -4.099999905, -3.899999857, -3.099999905, -4.099999905, -3.899999857, -3.299999952, -3.899999857, -3.099999905, -3.899999857, -3.899999857, -3.299999952, -3.899999857, -3.699999809, -3.899999857, -3.899999857, -3.299999952, -3.899999857, -3.699999809, -3.299999952, -4.099999905, -4.099999905, -3.899999857, -3.299999952, -3.699999809, -3.699999809, -3.899999857, -3.299999952, -3.699999809, -3.899999857, -3.299999952, -3.699999809, -3.899999857, -4.099999905, -3.699999809, -4.099999905, -3.099999905, -4.099999905, -3.5, -3.099999905, -3.5, -3.099999905, -3.5, -3.699999809, -3.699999809, )
    s223_Y = ( -4.099999905, -4.700000286, -4.700000286, -3.900000095, -4.099999905, -4.099999905, -4.099999905, -4.099999905, -4.900000095, -3.900000095, -4.300000191, -4.300000191, -4.300000191, -4.300000191, -4.900000095, -3.900000095, -4.300000191, -4.300000191, -4.5, -4.900000095, -3.900000095, -4.300000191, -4.300000191, -4.700000286, -4.700000286, -4.099999905, -4.5, -4.700000286, -4.5, -3.900000095, -4.900000095, -4.700000286, -4.700000286, -4.5, -4.5, -4.300000191, -4.5, -3.900000095, -4.099999905, -4.700000286, -4.099999905, -4.5, -4.700000286, -4.900000095, -4.099999905, -3.900000095, -4.700000286, -4.700000286, -3.900000095, -4.5, -4.099999905, -4.5, -4.700000286, -4.900000095, -4.099999905, -4.5, -3.900000095, -3.900000095, -3.900000095, -4.5, -4.700000286, -4.900000095, -4.900000095, -4.900000095, -3.900000095, -4.700000286, -4.300000191, -4.099999905, -4.099999905, -4.900000095, -4.900000095, -4.700000286, -4.700000286, -4.300000191, -4.099999905, -4.099999905, -4.300000191, -3.900000095, -4.900000095, -3.900000095, -4.900000095, -4.300000191, -4.900000095, -4.099999905, -4.300000191, -4.300000191, -4.5, -4.300000191, -4.5, -4.5, -4.5, -3.900000095, -4.5, -4.900000095, -3.900000095, -4.900000095, )
    s223_Z = ( 0.3000000119, -0.09999999404, 0.8999999762, 0.1000000089, 0.1000000089, 0.1000000089, -0.09999999404, 0.8999999762, 0.6999999881, 0.6999999881, 0.6999999881, 0.6999999881, -0.09999999404, 0.8999999762, 0.5, 0.5, 0.5, 0.5, 0.8999999762, 0.3000000119, 0.3000000119, 0.3000000119, 0.3000000119, -0.09999999404, 0.8999999762, 0.8999999762, -0.09999999404, 0.8999999762, 0.8999999762, 0.3000000119, 0.1000000089, 0.5, 0.5, -0.09999999404, 0.8999999762, -0.09999999404, 0.8999999762, 0.1000000089, 0.5, -0.09999999404, 0.6999999881, 0.5, 0.6999999881, 0.3000000119, 0.3000000119, 0.1000000089, 0.1000000089, 0.3000000119, 0.5, 0.3000000119, 0.6999999881, -0.09999999404, 0.6999999881, 0.1000000089, 0.5, 0.3000000119, 0.5, 0.6999999881, 0.6999999881, 0.5, 0.8999999762, 0.5, 0.6999999881, 0.6999999881, 0.5, -0.09999999404, 0.8999999762, 0.8999999762, 0.8999999762, 0.5, 0.1000000089, 0.1000000089, 0.3000000119, -0.09999999404, -0.09999999404, -0.09999999404, 0.8999999762, 0.3000000119, 0.5, 0.3000000119, 0.3000000119, 0.8999999762, 0.3000000119, -0.09999999404, 0.1000000089, -0.09999999404, 0.1000000089, 0.1000000089, 0.6999999881, -0.09999999404, 0.1000000089, 0.1000000089, 0.6999999881, 0.1000000089, 0.6999999881, 0.6999999881, )
    s223_X_A = ( -4.199999809, -3.400000095, -3.599999905, -3.599999905, -2.400000095, -4.199999809, -3.400000095, -3.599999905, -3.400000095, -3.599999905, -2.400000095, -4.199999809, -3.400000095, -3.599999905, -3.400000095, -3.599999905, -2.400000095, -4.199999809, -3.599999905, -3.400000095, -3.599999905, -2.400000095, -4.199999809, -3.400000095, -3.599999905, -3.599999905, -3.400000095, -3.599999905, -3.599999905, -3.599999905, -3.799999952, -2.400000095, -4.099999905, -3.900000095, -3.599999905, -3.400000095, -3.599999905, -3.599999905, -4.199999809, -3.700000048, -2.400000095, -4.099999905, -2.400000095, -3.400000095, -2.400000095, -3.599999905, -2.400000095, -2.400000095, -3.599999905, -2.400000095, -4.199999809, -3.700000048, -4.099999905, -3.799999952, -2.400000095, -4.199999809, -3.599999905, -3.599999905, -3.599999905, -2.400000095, -3.599999905, -3.900000095, -3.400000095, -3.900000095, -3.599999905, -3.900000095, -3.599999905, -3.599999905, -3.599999905, -3.700000048, -3.400000095, -4.199999809, -4.199999809, -3.799999952, -3.400000095, -3.799999952, -3.599999905, -3.599999905, -3.400000095, -3.599999905, -3.799999952, -3.599999905, -3.799999952, -3.799999952, -4.199999809, -3.799999952, -4.199999809, -2.400000095, -4.099999905, -3.400000095, -2.400000095, -3.599999905, -2.400000095, -3.400000095, -3.599999905, -3.700000048, )
    s223_Y_A = ( -4.199999809, -4.599999905, -4.400000095, -3.599999905, -4, -4.199999809, -4.199999809, -4.400000095, -5, -3.599999905, -4, -4.199999809, -4.199999809, -4.400000095, -5, -3.599999905, -4, -4.199999809, -4.400000095, -5, -3.599999905, -4, -4.199999809, -4.599999905, -4.400000095, -4.400000095, -4.599999905, -4.400000095, -4.400000095, -3.599999905, -5, -4, -4.699999809, -4.5, -4.400000095, -4.199999809, -4.400000095, -3.599999905, -4.199999809, -4.699999809, -4, -4.5, -4, -5, -4, -3.599999905, -4, -4, -3.599999905, -4, -4.199999809, -4.5, -4.699999809, -5, -4, -4.599999905, -3.599999905, -3.599999905, -3.599999905, -4, -4.400000095, -4.900000095, -5, -4.900000095, -3.599999905, -4.699999809, -4.400000095, -4.400000095, -4.400000095, -4.900000095, -5, -4.599999905, -4.599999905, -4.199999809, -4.199999809, -4.199999809, -4.400000095, -3.599999905, -5, -3.599999905, -5, -4.400000095, -5, -4.199999809, -4.199999809, -4.199999809, -4.599999905, -4, -4.5, -4.599999905, -4, -3.599999905, -4, -5, -3.599999905, -4.900000095, )
    s223_Z_A = ( 0.200000003, -0.200000003, 1.200000048, 0.400000006, 0.8000000119, 0.200000003, -0.200000003, 1.200000048, 0.6000000238, 0.400000006, 0.8000000119, 0.6000000238, -0.200000003, 1.200000048, 0.6000000238, 0.400000006, 0.8000000119, 0.6000000238, 1.200000048, 0.200000003, 0.400000006, 0.8000000119, 0.200000003, -0.200000003, 1.200000048, 1.200000048, -0.200000003, 1.200000048, 1.200000048, 0.400000006, 0.200000003, 0.8000000119, 0.5, -0.1000000015, 1.200000048, -0.200000003, 1.200000048, 0.400000006, 0.6000000238, -0.1000000015, 0.8000000119, 0.5, 0.8000000119, 0.200000003, 0.8000000119, 0.400000006, 0.8000000119, 0.8000000119, 0.400000006, 0.8000000119, 0.6000000238, -0.1000000015, 0.6999999881, 0.200000003, 0.8000000119, 0.200000003, 0.400000006, 0.400000006, 0.400000006, 0.8000000119, 1.200000048, 0.5, 0.6000000238, 0.6999999881, 0.400000006, -0.1000000015, 1.200000048, 1.200000048, 1.200000048, 0.5, 0.200000003, 0.200000003, 0.200000003, -0.200000003, -0.200000003, -0.200000003, 1.200000048, 0.400000006, 0.6000000238, 0.400000006, 0.200000003, 1.200000048, 0.200000003, -0.200000003, 0.200000003, -0.200000003, 0.200000003, 0.8000000119, 0.6999999881, -0.200000003, 0.8000000119, 0.400000006, 0.8000000119, 0.200000003, 0.400000006, 0.6999999881, )

    s40_X = ( -2.900000095, -3.100000143, -3.100000143, -3.100000143, -3.299999952, -3.100000143, -3.299999952, -2.900000095, -3.100000143, -3.100000143, -2.700000048, -2.700000048, -3.100000143, -3.299999952, -2.700000048, -2.900000095, -2.900000095, -2.900000095, -2.900000095, -3.299999952, -2.700000048, -2.900000095, -3.100000143, -2.900000095, )
    s40_Y = ( -4.900000095, -5.299999714, -4.900000095, -4.700000286, -5.099999905, -5.099999905, -5.099999905, -4.900000095, -5.099999905, -4.700000286, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.299999714, )
    s40_Z = ( 0.3000000417, 0.7000000477, 0.3000000417, 0.7000000477, 0.7000000477, 0.3000000417, 0.5000000596, 0.9000000358, 0.9000000358, 0.5000000596, 0.7000000477, 0.5000000596, 0.9000000358, 0.5000000596, 0.5000000596, 0.5000000596, 0.5000000596, 0.9000000358, 0.3000000417, 0.7000000477, 0.7000000477, 0.7000000477, 0.5000000596, 0.7000000477, )
    s40_X_A = ( -3, -3.099999905, -3, -2.400000095, -3.400000095, -3, -3.400000095, -2.799999952, -2.799999952, -2.400000095, -2.599999905, -2.599999905, -2.799999952, -3.400000095, -2.599999905, -2.400000095, -2.900000095, -2.799999952, -3, -3.400000095, -2.599999905, -2.400000095, -3.099999905, -2.900000095, )
    s40_Y_A = ( -5, -5.300000191, -5, -4, -5, -5, -5, -5.199999809, -5.199999809, -4, -5, -5, -5.199999809, -5, -5, -4, -5.300000191, -5.199999809, -5, -5, -5, -4, -5.300000191, -5.300000191, )
    s40_Z_A = ( 0.200000003, 0.6999999881, 0.200000003, 0.8000000119, 0.6000000238, 0.200000003, 0.6000000238, 1.200000048, 1.200000048, 0.8000000119, 0.6000000238, 0.6000000238, 1.200000048, 0.6000000238, 0.6000000238, 0.8000000119, 0.5, 1.200000048, 0.200000003, 0.6000000238, 0.6000000238, 0.8000000119, 0.5, 0.6999999881, )

    s41_X = ( -2.5, -2.700000048, -2.700000048, -2.700000048, -2.899999857, -2.700000048, -2.899999857, -2.5, -2.700000048, -2.700000048, -2.299999952, -2.299999952, -2.700000048, -2.899999857, -2.299999952, -2.5, -2.5, -2.5, -2.5, -2.899999857, -2.299999952, -2.5, -2.700000048, -2.5, )
    s41_Y = ( -4.900000095, -5.299999714, -4.900000095, -4.700000286, -5.099999905, -5.099999905, -5.099999905, -4.900000095, -5.099999905, -4.700000286, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.099999905, -5.099999905, -4.900000095, -4.900000095, -4.700000286, -5.299999714, -5.299999714, )
    s41_Z = ( 0.3000000417, 0.7000000477, 0.3000000417, 0.7000000477, 0.7000000477, 0.3000000417, 0.5000000596, 0.9000000358, 0.9000000358, 0.5000000596, 0.7000000477, 0.5000000596, 0.9000000358, 0.5000000596, 0.5000000596, 0.5000000596, 0.5000000596, 0.9000000358, 0.3000000417, 0.7000000477, 0.7000000477, 0.7000000477, 0.5000000596, 0.7000000477, )
    s41_X_A = ( -2.599999905, -2.599999905, -2.599999905, -2.400000095, -3, -2.599999905, -3, -2.799999952, -2.799999952, -2.400000095, -2, -2, -2.799999952, -3, -2, -2.400000095, -2.599999905, -2.799999952, -2.599999905, -3, -2, -2.400000095, -2.599999905, -2.599999905, )
    s41_Y_A = ( -5, -5.400000095, -5, -4, -5, -5, -5, -5.199999809, -5.199999809, -4, -5.199999809, -5.199999809, -5.199999809, -5, -5.199999809, -4, -5.400000095, -5.199999809, -5, -5, -5.199999809, -4, -5.400000095, -5.400000095, )
    s41_Z_A = ( 0.200000003, 0.6000000238, 0.200000003, 0.8000000119, 0.6000000238, 0.200000003, 0.6000000238, 1.200000048, 1.200000048, 0.8000000119, 0.400000006, 0.400000006, 1.200000048, 0.6000000238, 0.400000006, 0.8000000119, 0.6000000238, 1.200000048, 0.200000003, 0.6000000238, 0.400000006, 0.8000000119, 0.6000000238, 0.6000000238, )

    X = ( -1.700000167, -1.5, -2.5, -1.700000167, -1.700000167, -1.700000167, -1.700000167, -1.5, -2.5, -1.700000167, -1.700000167, -1.700000167, -1.700000167, -2.5, -1.900000215, -1.900000215, -1.900000215, -1.900000215, -1.5, -2.5, -1.900000215, -1.900000215, -1.900000215, -1.900000215, -1.5, -2.5, -1.900000215, -1.900000215, -1.900000215, -1.5, -1.700000167, -2.5, -1.700000167, -1.900000215, -1.700000167, -1.900000215, -2.100000143, -2.300000191, -1.700000167, -2.100000143, -2.300000191, -1.700000167, -2.100000143, -1.5, -1.5, -2.100000143, -2.5, -1.900000215, -1.5, -1.5, -1.5, -2.100000143, -1.5, -2.300000191, -2.300000191, -1.700000167, -2.300000191, -2.100000143, -1.700000167, -2.5, -2.300000191, -2.300000191, -2.5, -2.5, -1.5, -2.300000191, -2.100000143, -2.5, -2.300000191, -2.300000191, -2.300000191, -2.100000143, -2.5, -2.100000143, -1.5, -2.5, -2.300000191, -1.5, -2.300000191, -2.100000143, -2.300000191, -2.100000143, -2.100000143, -2.100000143, -2.5, -2.300000191, -2.100000143, -1.5, -2.300000191, -2.5, -1.900000215, -2.5, -1.5, -1.900000215, -2.100000143, -2.100000143, )
    Y = ( -4.699999809, -4.900000095, -4.900000095, -5.099999905, -5.099999905, -5.699999809, -4.699999809, -4.900000095, -4.900000095, -5.300000191, -5.5, -5.5, -5.699999809, -4.900000095, -4.900000095, -4.900000095, -5.699999809, -4.699999809, -5.099999905, -5.099999905, -5.099999905, -5.099999905, -5.699999809, -4.699999809, -5.099999905, -5.099999905, -5.300000191, -5.699999809, -4.699999809, -5.099999905, -4.900000095, -5.099999905, -4.900000095, -5.5, -5.699999809, -5.5, -4.900000095, -5.699999809, -5.699999809, -5.5, -4.699999809, -4.699999809, -5.699999809, -5.5, -4.900000095, -4.699999809, -5.5, -5.300000191, -5.300000191, -5.5, -5.5, -4.699999809, -4.900000095, -4.699999809, -5.5, -5.300000191, -5.699999809, -5.5, -4.699999809, -5.5, -5.5, -5.099999905, -5.300000191, -5.5, -5.300000191, -4.699999809, -5.699999809, -4.900000095, -5.300000191, -5.699999809, -5.099999905, -5.300000191, -5.5, -5.300000191, -5.5, -5.300000191, -4.699999809, -5.300000191, -5.699999809, -4.699999809, -4.900000095, -5.699999809, -4.900000095, -5.099999905, -5.099999905, -5.300000191, -5.099999905, -5.099999905, -4.900000095, -5.300000191, -4.699999809, -5.300000191, -5.300000191, -5.699999809, -4.699999809, -5.699999809, )
    Z = ( 0.7000000477, 0.7000000477, 0.7000000477, -0.1000000015, 0.9000000358, 0.5, 0.5, 0.5, 0.5, -0.1000000015, -0.1000000015, 0.9000000358, 0.1000000015, 0.1000000015, -0.1000000015, 0.9000000358, 0.7000000477, 0.7000000477, 0.7000000477, 0.7000000477, -0.1000000015, 0.9000000358, 0.5, 0.5, 0.5, 0.5, 0.9000000358, 0.3000000119, 0.3000000119, 0.3000000119, -0.1000000015, 0.3000000119, 0.9000000358, -0.1000000015, 0.7000000477, 0.9000000358, 0.9000000358, 0.1000000015, 0.3000000119, 0.9000000358, 0.5, 0.3000000119, 0.1000000015, 0.5, 0.3000000119, 0.1000000015, 0.5, -0.1000000015, 0.1000000015, 0.1000000015, 0.3000000119, 0.3000000119, 0.1000000015, 0.1000000015, 0.9000000358, 0.9000000358, 0.5, -0.1000000015, 0.1000000015, 0.1000000015, -0.1000000015, 0.9000000358, 0.3000000119, 0.3000000119, 0.3000000119, 0.3000000119, 0.3000000119, 0.3000000119, -0.1000000015, 0.3000000119, -0.1000000015, 0.9000000358, 0.7000000477, -0.1000000015, 0.7000000477, 0.5, 0.7000000477, 0.5, 0.7000000477, 0.5, 0.9000000358, 0.5, -0.1000000015, 0.9000000358, 0.1000000015, 0.9000000358, -0.1000000015, 0.1000000015, -0.1000000015, 0.7000000477, 0.1000000015, 0.1000000015, 0.7000000477, 0.1000000015, 0.7000000477, 0.7000000477, )
    X_A = ( -2.400000095, -1.200000048, -2.599999905, -1.799999952, -2, -1.700000048, -2.400000095, -1.200000048, -2.599999905, -1.799999952, -1.799999952, -2, -1.799999952, -2.599999905, -1.799999952, -2, -1.899999976, -2.400000095, -1.200000048, -2.599999905, -1.799999952, -2, -1.899999976, -2.400000095, -1.200000048, -2.599999905, -2, -1.799999952, -2.400000095, -1.200000048, -1.799999952, -2.599999905, -2, -1.799999952, -1.700000048, -2, -2, -2.200000048, -1.799999952, -2, -2.400000095, -2.400000095, -2.200000048, -1.200000048, -1.200000048, -2.400000095, -2.599999905, -1.799999952, -1.200000048, -1.200000048, -1.200000048, -2.400000095, -1.200000048, -2.400000095, -2, -2, -2.299999952, -2.099999905, -2.400000095, -2.599999905, -2.299999952, -2, -2.599999905, -2.599999905, -1.200000048, -2.400000095, -2.200000048, -2.599999905, -2.299999952, -2.200000048, -2.200000048, -2, -2.599999905, -2.099999905, -1.200000048, -2.599999905, -2.400000095, -1.200000048, -2.400000095, -2, -2.099999905, -2.200000048, -2, -2.599999905, -2, -2.200000048, -1.200000048, -2.200000048, -2.599999905, -2.400000095, -2.599999905, -1.200000048, -1.799999952, -2.400000095, -2.099999905, )
    Y_A = ( -4, -5.199999809, -5, -5, -5.199999809, -5.699999809, -4, -5.199999809, -5, -5.400000095, -5.400000095, -5.199999809, -5.800000191, -5, -5, -5.199999809, -5.699999809, -4, -5.199999809, -5, -5, -5.199999809, -5.699999809, -4, -5.199999809, -5, -5.199999809, -5.800000191, -4, -5.199999809, -5, -5, -5.199999809, -5.400000095, -5.699999809, -5.199999809, -5.199999809, -5.800000191, -5.800000191, -5.199999809, -4, -4, -5.800000191, -5.199999809, -5.199999809, -4, -5.400000095, -5.400000095, -5.199999809, -5.199999809, -5.199999809, -4, -5.199999809, -4, -5.199999809, -5.199999809, -5.699999809, -5.5, -4, -5.400000095, -5.5, -5.199999809, -5.400000095, -5.400000095, -5.199999809, -4, -5.800000191, -5, -5.300000191, -5.800000191, -5, -5.199999809, -5.400000095, -5.300000191, -5.199999809, -5.400000095, -4, -5.199999809, -4, -5.199999809, -5.699999809, -5, -5.199999809, -5, -5.199999809, -5, -5.199999809, -5, -5.400000095, -4, -5.400000095, -5.199999809, -5.800000191, -4, -5.699999809, )
    Z_A = ( 0.8000000119, 0.400000006, 0.6000000238, -0.200000003, 1.200000048, 0.5, 0.8000000119, 0.400000006, 0.6000000238, -0.200000003, -0.200000003, 1.200000048, 0.200000003, 0.200000003, -0.200000003, 1.200000048, 0.6999999881, 0.8000000119, 0.400000006, 0.6000000238, -0.200000003, 1.200000048, 0.5, 0.8000000119, 0.400000006, 0.6000000238, 1.200000048, 0.200000003, 0.8000000119, 0.400000006, -0.200000003, 0.200000003, 1.200000048, -0.200000003, 0.6999999881, 1.200000048, 1.200000048, 0.200000003, 0.200000003, 1.200000048, 0.8000000119, 0.8000000119, 0.200000003, 0.400000006, 0.400000006, 0.8000000119, 0.6000000238, -0.200000003, 0.400000006, 0.400000006, 0.400000006, 0.8000000119, 0.400000006, 0.8000000119, 1.200000048, 1.200000048, 0.5, -0.1000000015, 0.8000000119, 0.200000003, -0.1000000015, 1.200000048, 0.200000003, 0.200000003, 0.400000006, 0.8000000119, 0.200000003, 0.200000003, -0.1000000015, 0.200000003, -0.200000003, 1.200000048, 0.6000000238, -0.1000000015, 0.400000006, 0.6000000238, 0.8000000119, 0.400000006, 0.8000000119, 1.200000048, 0.5, -0.200000003, 1.200000048, 0.200000003, 1.200000048, -0.200000003, 0.400000006, -0.200000003, 0.6000000238, 0.8000000119, 0.200000003, 0.400000006, 0.200000003, 0.8000000119, 0.6999999881, )


    # s        @37    -3.900000;   -4.900000;     0.500000    @ 0.200000
    # s        @223(-3.6 -4.4 0.400001) => (-3.6 -4.4 0.4) for size 0.8
    # s        @411 3.400000; -5.000000; -0.200000 @ 0.400000
    # start center    -11.300000; -4.900000;      0.500000    @ 0.200000
    #end point      -1.3,       -4.9,           0.5  => the distance takes into account the final point, not it's center

    start_coordinates               = (-11.30000, -4.900000,  0.500000)
    s37                             = (-3.900000, -4.900000,  0.500000)  
    s223                            = (-3.600000, -4.400000,  0.400000)  
    s39                             = (-3.400000, -5.000000,  0.600000) 
    s39_neighbor                    = (-3.1, -4.9,  0.5) 
    s39_neighbor_center             = (-3, -5,  0.6) 
    s40                             = (-3, -5,  0.6) 
    s41                             = (-2.600000, -5.000000, 0.600000) 
    s220                            = (-2.000000, -5.200000, 0.400000) 
    goal_point_coordinates          = (-1.3,      -4.9,       0.5)

    s = (s37, s223, s39, s40, s41)
    prev_s_coord                    = s41 
    current_s_coord                 = s220 
    # correct_neighbor_coordinates    = {'point': s39_neighbor, 
    #                                    'center': s39_neighbor_center} # unknown
    plot_pointVisualization_longDebug(X, Y, Z, X_A, Y_A, Z_A, prev_s_coord, current_s_coord, 
        start_coordinates, goal_point_coordinates, s)
                                                                                
                                                                       
# RUN RUN RUN
line_size = 10
resolution = 0.2
regularGrid_iterations = line_size/resolution
groups_highToLow_back = [1000000000, 250, 101,  regularGrid_iterations+1, regularGrid_iterations-1,  regularGrid_iterations/2, 0]
analyzeIterationCsvFile('10m/iterations_cellDistributionoffShoreOil_2Obst_10m', groups_highToLow_back, 'simulated offshore oil platform')



# variables= {}
# execfile( "iterations_vs_voxelSize.py", variables )