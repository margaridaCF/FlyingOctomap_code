import csv
import plotly
import plotly.graph_objs as go
from plotly.graph_objs import Bar, Layout
from collections import defaultdict
import math
import itertools

def get_integral_digit_count (number):
    return len(str(math.modf(number)[1]))

def average_column_horizontal (csv_filepath):

    averages = []
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            # Here is where the sampling times can be filtered
            averages[len(averages):] = [sum(row) / len(row)]
    return averages

def plot_bar_chart (averages):
    # print averages
    plotly.offline.plot({
        "data": [Bar(x=['Run 1', 'Run 2'], y=averages)],
        "layout": Layout(title="Number of times neighbour generation was calculated")
    })

def plot_time_vs_count_bar (count, time_avg, run):
    plotly.offline.plot({
        "data": [Bar(x=count, y=time_avg)],
        "layout": Layout(title="Time in average spent by lazy theta on neighbors by total neighbor count @ "+run)
        
    }, filename='plots/countVStime_'+run)

def plot_stacked_bar(volume, remaining_time, neighbor_time, title):
    trace1 = go.Bar(
        x=volume,
        y=remaining_time,
        name='Remaining time'
    )
    trace2 = go.Bar(
        x=volume,
        y=neighbor_time,
        name='Neighbor time'
    )

    data = [trace1, trace2]
    layout = go.Layout(
        barmode='stack',
        title='Time distribution according to explored space volume, ' + title,
        xaxis=dict(
            title='Octree volume in square meters',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        ),
        yaxis=dict(
            title='Time spent in milliseconds',
            titlefont=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        )
    )

    # fig = go.Figure(data=data, layout=layout)
    # py.plot(fig, filename='stacked-bar')

    plotly.offline.plot({
        "data": data,
        "layout": layout
    }, filename='plots/stackedTime_byVolume_'+title)

def extract_data_from_file_vertical(csv_filepath):
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        x = []
        y = []
        for row in reader:
            x.append(row[0])
            y.append(row[1])

        return [sum(y) / len(y)]

def extract_data_from_file_vertical_merged(csv_filepath):
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        timestamp = []
        count = []
        elapsed_time = []
        for row in reader:
            timestamp.append(row[0])
            count.append(row[1])
            elapsed_time.append(row[2])
        return [sum(count) / len(count), sum(elapsed_time) / len(elapsed_time)]

def extract_data_separate_by_neighbor(csv_filepath):
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        timestamp = []
        count = []
        elapsed_time = []
        volume = []
        group_by_neighbor = defaultdict(list)
        group_by_volume = defaultdict(list)
        for row in reader:
            timestamp.append(row[0])
            count.append(row[1])
            elapsed_time.append(row[2])
            group_by_neighbor[row[1]].append(row[2])
            group_by_volume[row[3]].append(row[2])
            volume.append(row[3])

        return {'neighbor_count':count, 'avg_time': elapsed_time, 'histogram': group_by_neighbor, 'volume' : volume, 'group_by_volume' : group_by_volume}

def extract_data_separate_by_volume(csv_filepath):
    with open(csv_filepath,"rb") as f:
        reader  = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        elapsed_time = []
        volume = []
        group_by_volume = defaultdict(list)
        measurement_count = 0
        for row in reader:
            group_by_volume[row[3]].append(row[4]) # dict[volume] = [time1, time2]
            measurement_count = measurement_count+1

        return {'group_by_volume': group_by_volume, 'measurement_count' : measurement_count}
        




def analyze_organised_data(organised_data, data_name):
    print " === ", data_name, " ==="
    print "Bag has ", len(organised_data['neighbor_count']), " calls to lazy theta star and ", sum(organised_data['neighbor_count']), " calls to neighbor"
    counts = []
    time_avg = []
    for count, measurements in organised_data['histogram'].iteritems():
        avg = sum(measurements) / len(measurements)
        print count, " call to getNeighbors happened ", len(measurements), " times. It's Mean execution time was ", avg#, "\n  Values: ", measurements
        counts.append(count)
        time_avg.append(avg/1000000)
    plot_time_vs_count_bar(counts, time_avg, data_name)


def histogram_gen(organised_data, data_name):
    print " === ", data_name, " ==="
    counts = []
    time_avg = []
    for count, measurements in organised_data.iteritems():
        avg = sum(measurements) / len(measurements)
        print count, " has ", len(measurements), " measurements. Mean was ", avg#, "\n  Values: ", measurements
        counts.append(count)
        time_avg.append(avg)
    plot_time_vs_count_bar(counts, time_avg, data_name)




def check_assumptions_for_filtering(original_data, filtered_data):
    # Filteres bag has less or same number of calls to lazy theta star
    if len(original_data['neighbor_count']) >= len(filtered_data['neighbor_count']) :
        print "Filtered bag indeed has less calls to Lazy Theta Star ( exactly ", len(original_data['neighbor_count']) - len(filtered_data['neighbor_count']), " fewer). Which means that the filter of the bag avoided duplicated calls."
    else:
        print "CHECK ASSUMPTIONS! The filtering added more calls to Lazy Theta Star - Red Alert!"
    

def compare_filtered_vs_original_v1():
    # Vertical
    run_1 = extract_data_from_file_vertical_merged("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run1_prev.csv")
    run_1_filtered = extract_data_from_file_vertical_merged("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run1_filtered_prev.csv")

    print run_1
    print run_1_filtered

    count_avg = [run_1[0], run_1_filtered[0]]
    time_avg = [run_1[1], run_1_filtered[1]]

    # # Horizontal
    # averages_out = average_column("/home/mfaria/ws_mavlink_grvcHal/src/generate_plots/example_eachRunMeasurementPerRow.csv")

    plot_bar_chart(count_avg)

def compare_filtered_vs_original_v2(original_csv, filtered_csv, data_name):
    run_1 = extract_data_separate_by_neighbor(original_csv)
    run_1_filtered = extract_data_separate_by_neighbor(filtered_csv)

    check_assumptions_for_filtering(run_1, run_1_filtered)

    analyze_organised_data(run_1_filtered, data_name+" filtered")


def compare_for_e_byVolume(total_csv, neighbor_csv, data_name):
    f_total    = open(total_csv,"rb")
    f_neighbor = open(neighbor_csv,"rb")

    csv_total    = csv.reader(f_total, quoting=csv.QUOTE_NONNUMERIC)
    csv_neighbor = csv.reader(f_neighbor, quoting=csv.QUOTE_NONNUMERIC)

    measurement_count = 0
    time_remaining = []
    time_neighbor = []
    volume_avg = []

    for row_total, row_neighbor in itertools.izip(csv_total, csv_neighbor):
        if (row_neighbor[2] > row_total[4]):
            print "RED ALERT! Neighbour time is higher that overall time"
        time_remaining.append( row_total[4]/1000000 - row_neighbor[2]/1000000 )
        time_neighbor.append ( row_neighbor[2]/1000000 )
        volume_avg.append    ( (row_total[3]+row_neighbor[3])/2 )

    print volume_avg
    print time_remaining
    print time_neighbor
    plot_stacked_bar(volume_avg, time_remaining, time_neighbor, data_name)

def compare_for_e_byCallCount(total_csv, neighbor_csv, data_name):
    f_total    = open(total_csv,"rb")
    f_neighbor = open(neighbor_csv,"rb")

    csv_total    = csv.reader(f_total, quoting=csv.QUOTE_NONNUMERIC)
    csv_neighbor = csv.reader(f_neighbor, quoting=csv.QUOTE_NONNUMERIC)

    measurement_count = 0
    time_remaining = []
    time_neighbor = []
    call_count = []

    for row_total, row_neighbor in itertools.izip(csv_total, csv_neighbor):
        if (row_neighbor[2] > row_total[4]):
            print "RED ALERT! Neighbour time is higher that overall time"
        time_remaining.append( row_total[4]/1000000 - row_neighbor[2]/1000000 )
        time_neighbor.append ( row_neighbor[2]/1000000 )
        call_count.append    ( row_neighbor[1] )

    print call_count
    print time_remaining
    print time_neighbor
    plot_stacked_bar(call_count, time_remaining, time_neighbor, data_name)



def compare_for_e(total_csv, neighbor_csv, data_name):
    total_dict     = extract_data_separate_by_volume  (total_csv)
    neighbour_dict = extract_data_separate_by_neighbor(neighbor_csv)
    analyze_organised_data(neighbour_dict, "neighbor time measurements")
    histogram_gen(total_dict['group_by_volume'], data_name)

    # Extract remaining time
    group_neighborTime_by_volume = neighbour_dict['group_by_volume']
    group_totalTime_by_volume    = total_dict['group_by_volume']

    # x_label = []
    # digest_neighborTime_by_volume = defaultdict(list)
    # for v in neighbour_dict['volume']:
    #     print "volume ", v
    #     order_of_greatness = get_integral_digit_count(v)
    #     digest_neighborTime_by_volume [order_of_greatness].append(group_neighborTime_by_volume[v])
    #     print group_neighborTime_by_volume[v]
    #     x_label.append(order_of_greatness)
        
    print neighbour_dict['volume']
    print digest_neighborTime_by_volume
    print digest_neighborTime_by_volume

    plot_stacked_bar(neighbour_dict['volume'], digest_neighborTime_by_volume, digest_neighborTime_by_volume, data_name)

def compare_all_runs_overall_time_vs_neighbor():
    compare_for_e_byCallCount("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/timeH_run_1_filtered_prev_overallLazy_Time.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/timeH_run1_filtered_prev_neighbors_time.csv", "Run 1 overall time vs neighbor time byt call count of getNeighbor")
    # compare_for_e("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/run_2_filtered_prev_overallLazy_Time.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run2_filtered_prev.csv", "Run 2 overall time vs neighbor time")
    # compare_for_e("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/run_3_filtered_prev_overallLazy_Time.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run3_filtered_prev.csv", "Run 3 overall time vs neighbor time")

def compare_all_runs_filteredOriginal():
    compare_filtered_vs_original_v2("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run1_prev.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run1_filtered_prev.csv", "Run 1")

    compare_filtered_vs_original_v2("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run2_prev.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run2_filtered_prev.csv", "Run 2")

    compare_filtered_vs_original_v2("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run3_prev.csv", "/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/count_file_timeH_run3_filtered_prev.csv", "Run 3")



# RUN RUN RUN

# Increases with call count
run_1_filtered = extract_data_separate_by_neighbor("/home/mfaria/Margarida/20170717_RedUas/experimental data/measurements/timeH_run1_filtered_prev_neighbors_time.csv")
analyze_organised_data(run_1_filtered, "Filtered getNeighbor execution time by call count")

# getNeighbor is a significant part of total execution time
compare_all_runs_overall_time_vs_neighbor()

# With new implementation execution time goes down

# variables= {}
# execfile( "plots.py", variables )