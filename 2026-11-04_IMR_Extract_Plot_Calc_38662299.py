# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 15:55:31 2026

@author: Daniel
"""

import csv
import json
import math
from matplotlib import pyplot as plt
from scipy.ndimage import gaussian_filter1d
import numpy as np
import os

#Initialise
year = "2026"
month = "11"
day = "02"
arena = "2"
robot_tag = "22"

main_path = f"C:/Users/XXXXXXXXX/{year}-{month}-{day}_Practical"


#Specify if there is Ground Truth available?
GROUND_TRUTH = True
GT_path = f"{main_path}/{year}-{month}-{day}_Aruco_Data_Arena{arena}/{robot_tag}_Tag_{robot_tag}.csv"


#Specify whether there is a shift in time between the observer (Aruco camera) ground truth and the laptop-logged data.
account_for_time_shift = True


#Specify which sort of parameters you want to evaluate
# param2eval = "Control"
param2eval = "EKF"
# param2eval = "All"


if param2eval == "Control":
    total_path = f"{main_path}/XXXXXXXXXXX"
    #                    North, East
    target_waypoints = [(0,     0),
                        (0.5,   0),
                        (0.5,   1),
                        (1.5,   1),
                        (1.5,   0)]
    initial_heading = 0.0 #[deg]
    final_heading = -90.0 #[deg]
    
elif param2eval == "EKF":
    total_path = f"{main_path}/XXXXXXXXXXXX"
    #                    North, East
    target_waypoints = [(0,     0),
                        (1,     0),
                        (1,     1),
                        (0,     1),
                        (0,     0),
                        (1,     0),
                        (1,     1),
                        (0,     1),
                        (0,     0)]
    initial_heading    = 0.0 #[deg]
    final_heading    = -90.0 #[deg]
    
elif param2eval == "All":
    total_path = f"{main_path}/XXXXXXXXXXXXX"
#                        North, East
    target_waypoints = [(0,     0),
                        (1.5,   0),
                        (1.5,   1),
                        (0,     1),
                        (0,     0),
                        (1.5,   0),
                        (1.5,   1),
                        (0,     1),
                        (0,     0)]
    initial_heading    = 0.0 #[deg]
    final_heading    = -90.0 #[deg]
    
    #For STATIONARY case:
    # target_waypoints = [(0,     0)]
    # initial_heading  = 0.0 #[deg]
    # final_heading    = 0.0 #[deg]




###############################################################################
#Functions
def calibrate(array_):
    """
    This function calibrates an array to a zero initial value by subtracting the initial value from the entire array.

    Parameters
    ----------
    array_ : array_like
        An array containing numeric values to calibrate to initial value of zero.

    Returns
    -------
    calibrated_array : array_like
        Returns calibrated array, where first element is zero.

    """
    #nested numpy commands do not affect array, e.g. np.array(np.array([1,2,3])) == np.array([1,2,3])
    numpy_array = np.array(array_, dtype=float)
    calibrated_array = numpy_array - numpy_array[0]
    
    return np.array(calibrated_array, dtype=float)


def rounding(array_, digits_, rule_="standard"):
    """
    This function round an array as desired (up, down, standard) to the provided number of digits.

    Parameters
    ----------
    array_ : array_like
        An array containing numeric values to round to the specified digits.
    digits_ : integer
        The number of digits to round to.
    rule_:
        Specifies if rounding always up/down/standard

    Returns
    -------
    rounded_array : array_like
        Returns array rouded as desired.

    """
    #nested numpy commands do not affect array, e.g. np.array(np.array([1,2,3])) == np.array([1,2,3])
    numpy_array = np.array(array_, dtype=float)
    if rule_ == "up":
        rounded_array = np.ceil((10**digits_) * numpy_array) / (10**digits_)
    elif rule_ == "down":
        rounded_array = np.floor((10**digits_) * numpy_array) / (10**digits_)
    else:
        rounded_array = np.round(numpy_array, digits_)

    return np.array(rounded_array, dtype=float)


def extractControlParameters(path_):    
    #Control Law parameters tau & L are logged and thus extracted from file/path
    tau_ = False
    L_ = False
    
    with open(path_, "r") as file:
        while not (tau_ and L_):
            for row in file:
                line = json.loads(row)
                topic = line.get("topic_name")
                if topic == "/tau_s_log":
                    tau_ = line["message"]["pose"]["covariance"]
                if topic == "/L_log":
                    L_ = line["message"]["pose"]["covariance"]
                    
    return tau_, L_
    
       
def extractEKFParameters(path_):    
    #EKF noise matrices Q & R_ne & R_g are logged and thus extracted from file/path
    Q_ = False
    R_ne_ = False
    R_g_ = False
    
    with open(path_, "r") as file:
        while not (Q_ and R_ne_ and R_g_):
            for row in file:
                line = json.loads(row)
                topic = line.get("topic_name")
                if topic == "/init_Q":
                    Q_ = line["message"]["pose"]["covariance"]
                if topic == "/init_R_ne":
                    R_ne_ = line["message"]["pose"]["covariance"]
                if topic == "/init_R_g":
                    R_g_ = line["message"]["pose"]["covariance"]
                    
    return Q_, R_ne_, R_g_


def extractCSV(data_, headers_=None, skip_=0):
    """
    This function extracts values from desired columns of CSV file data.
    If no columns are specified, by default all columns' values are extracted.

    Parameters
    ----------
    data_ : data
        File data opened using command: with open() as data.
    headers_ : array_like
        An array containing strings, listing all column names to search for in file data for extracting its values.
    skip_ : integer
        Number of rows to skip before extracting data (excluding header row). Default = 0

    Returns
    -------
    columns_ : dict
        A dictionary containing one key per specified column, listing its extracted values in an array.

    """    
    content = csv.DictReader(data_)
    if headers_ == None: headers_ = content.fieldnames
    
    #Following 5 lines generated with help of AI
    columns_ = {c:[] for c in headers_}
    
    for i, line in enumerate(content):
        if i < skip_: continue
        
        for h in headers_:
            columns_[h].append(line[h])

    return columns_


def extractData(data_, topic_):
    """
    This function extracts the timestamps, orientation/heading, and positions from either the logged data or ground truth data.

    Parameters
    ----------
    data_ : data
        File data opened using command: with open() as data.
    topic_ : string
        Topic name to search for in file data for extracting its values.

    Returns
    -------
    timestamps_ : array_like
        Array of timestamp values.
    headings_ : array_like
        Array of orientation/heading values.
    positions_ : array_like
        Array of position values.

    """
    timestamps_ = []
    headings_   = []
    positions_  = []
    
    #Following 4 lines generated with help of AI
    for row in data_:
        line = json.loads(row)  
        topic = line.get("topic_name")
        
        if topic == str(topic_):
            t, position, gamma = extractLineData(line)
            
            timestamps_.append(t)
            headings_.append(gamma)
            positions_.append(position)
                
    return np.array(timestamps_, dtype=float), np.array(headings_, dtype=float), np.array(positions_, dtype=float)


def extractLineData(line_):
    
    #Following 3 lines generated with help of AI
    timestamp_ = line_.get("timestamp")
    position_ = line_.get("message", {}).get("pose", {}).get("position")
    orientation_ = line_.get("message", {}).get("pose", {}).get("orientation")
        
    x_traj = position_["x"]
    y_traj = position_["y"]
    z_traj = position_["z"]
    
    point_ = np.array([x_traj, y_traj, z_traj])
            
    x_rot = orientation_["x"]
    y_rot = orientation_["y"]
    z_rot = orientation_["z"]
    w = orientation_["w"]
    
    #3D Formula simplifies in 2D as x_rot = y_rot = 0 #This line has been looked up using AI
    yaw_ = math.degrees( math.atan2(2*(w*z_rot + x_rot*y_rot), 1-2*(y_rot**2 + z_rot**2)) )
    
    return timestamp_, point_, yaw_


def findMatchingData(t_ref_, t_match_, data_match_):
    
    t_ref_start, t_ref_end = t_ref_[0], t_ref_[-1]
    
    i = 0
    t1 = t_match_[i]
    while t1 < t_ref_start:
        i += 1
        if i >= len(t_match_)-1: break
        t1 = t_match_[i]
    
    j = i
    t2 = t_match_[j]
    while t2 < t_ref_end:
        j += 1
        if j >= len(t_match_)-1: break
        t2 = t_match_[j]
    
    assert j > i, "ERROR: time arrays do not have common overlap and thus cannot be matched! \
        \nThere might be a shift in time or recorded in another arena. Try setting account_for_time_shift to True or change arena."
    t_matched = t_match_[i:j]
    data_matched = []
    for series in data_match_:
        series_matched = series[i:j]
        data_matched.append( np.array(series_matched, dtype=float) )
    
    return np.array(t_matched, dtype=float), data_matched


def filterData(series2filter_, sigma_level_):
    
    sigma = np.sqrt( np.array(series2filter_).var() )
    
    filtered_series = gaussian_filter1d(np.array(series2filter_), sigma_level_*sigma) #radius=range included in kernel, default = 4*sigma
    
    return np.array(filtered_series)


def rootMeanSquare(t_est_, estimates_, t_ref_, reference_):
    """
    This function calculates the root mean square (RMS) between an array of estimated and observer values.
    These araays do not need to have the same length.

    Parameters
    ----------
    estimate : array_like
        Estimate values.
    t_est : array_like
        Timestamps of estimates.
    observation : array_like
        Observation values.
    t_obs : array_like
        Timestamps of obserations.

    Returns
    -------
    RMS_ : Integer
        Root Mean Squared.
    """

    assert len(t_est_)==len(estimates_) and len(t_ref_)==len(reference_) , "ERROR: array lengths do not match!"
    
    SQERR_ = []
    i = 0
    j = 0
    
    if len(t_est_) >= len(t_ref_):
        while i < len(t_est_) and j < len(t_ref_):
            if t_est_[i] <= t_ref_[j]:
                sqerr = (reference_[j] - estimates_[i])**2
                SQERR_.append(sqerr)
                i += 1
                #print(f"{i} /",len(t_est))
            elif t_est_[i] > t_ref_[j]:
                j += 1
                #print(f"{j} /",len(t_obs))
            
    elif len(t_est_) < len(t_ref_):
        while i < len(t_est_)-1 and j < len(t_ref_):
            if t_est_[i] <= t_ref_[j] and t_est_[i+1] > t_ref_[j]:
                sqerr = (reference_[j] - estimates_[i])**2
                SQERR_.append(sqerr)
                j += 1
            elif t_est_[i] > t_ref_[j]:
                j += 1
            elif t_est_[i+1] < t_ref_[j]:
                i += 1
    
    #print("Number of values for MSE:\n",len(SQERR_),"\n")
    MSE_ = (1/len(SQERR_))*np.sum(SQERR_)
    RMS_ = np.sqrt(MSE_)
    
    return RMS_




###############################################################################
#Run Code
file_names = os.listdir(total_path)

#Specify Process Noise Matrix Values [R]
R_x = 0.01986119071239008 #[m]
R_y = 0.02335564849035042  #[m]
R_heading = 12.4792465984863 #[deg]


for k, file_name in enumerate(file_names):
    logfile = f"{total_path}/{file_name}"
    
    if param2eval == "Control":
        print("\n\n==============================================")
        print(f"File ({k+1}): {file_name}")
        print("++++++++ Evaluating: Control Law Data ++++++++")
        
        #IF CONTROL LAW PARAMETERS ARE LOGGED, UNCOMMENT AND USE FUNCTION:
        tau, L = extractControlParameters(logfile)
        print(f" tau \t= {tau} \n L \t\t= {L}")
        print("======================================\n")

    elif param2eval == "EKF":
        print("\n\n======================================")
        print(f"File ({k+1}): {file_name}")
        print("++++++++ Evaluating: EKF Data ++++++++")
                
        Q, R_ne, R_g = extractEKFParameters(logfile)
        print(f" Q \t\t= {Q} \n R_ne \t= {R_ne} \n R_g \t= {R_g}")
        print("======================================\n")

    elif param2eval == "All":
        print("\n\n======================================")
        print(f"File ({k+1}): {file_name}")
        print("++++++++ Evaluating Data ++++++++")
        
        tau, L = extractControlParameters(logfile)
        Q, R_ne, R_g = extractEKFParameters(logfile)
        print(f" tau \t= {tau} \n L \t\t= {L} \n Q \t\t= {Q} \n R_ne \t= {R_ne} \n R_g \t= {R_g}")
        print("======================================\n")
    
    
    
    #Extract Logged Data
    with open(logfile, "r") as logdata:
        timestamps_observer, headings_observer, positions_observer = extractData(logdata, "/aruco")
        logdata.seek(0) #This line added after learning from AI that any json/csv reader maintains position when not resetting to read file from beginning.
        timestamps_est0, headings_est0, positions_est0 = extractData(logdata, "/est_pose")
    
    if account_for_time_shift == True:
        # +T=3600 seconds DUE TO TIME SHIFT BETWEEN observer CAMERA AND LAPTOP LOGS (automatically connects and adapts time)
        timestamps_observer = np.array(timestamps_observer, dtype=float)+3600
        timestamps_est0 = np.array(timestamps_est0, dtype=float)+3600
        
    
    
    #Extract Ground Truth
    if GROUND_TRUTH == True:
        with open(f"{GT_path}", "r", encoding="utf-8-sig") as GTdata:
            GT = extractCSV(GTdata)
        timestamps_gt0 = np.array(GT["epoch [s]"], dtype=float).tolist()
        positions_gt0 = [ np.array([x, y, z]) for x,y,z in zip(GT["x [m]"], GT["y [m]"], GT["z [m]"]) ]
        headings_gt0 = GT["yaw [deg]"]

    
    
    #Find matching data    
    timestamps_estimated, [headings_estimated, positions_estimated] = \
        findMatchingData(timestamps_observer, timestamps_est0, [headings_est0, positions_est0])
    # print("\n++++++++++\n Matched time array boundaries:")
    # print("t_start_estimated =", timestamps_estimated[0],  "\t t_start_observer =", timestamps_observer[0])
    # print("t_end_estimated =", timestamps_estimated[-1], "\t t_end_observer =", timestamps_observer[-1])
    # print("\n++++++++++\n Discrepancy between matched time arrays:")
    # print("t_start_estimated - t_start_observer =\t", timestamps_estimated[0] - timestamps_observer[0], "[s]")
    # print("t_end_estimated - t_end_observer =\t\t", timestamps_estimated[-1] - timestamps_observer[-1], "[s]")
    # print("Estimated timestamps array length =\t\t", len(timestamps_estimated))
    # print("Observer timestamps array length =\t\t", len(timestamps_observer))
    
    if GROUND_TRUTH == True:
        timestamps_GT, [headings_GT, positions_GT] = \
            findMatchingData(timestamps_observer, timestamps_gt0, [headings_gt0, positions_gt0])
        # print("\n++++++++++\n Matched time array boundaries:")
        # print("t_start_observer =", timestamps_observer[0],  "\t t_start_GT =", timestamps_GT[0])
        # print("t_end_observer =", timestamps_observer[-1], "\t t_end_GT =", timestamps_GT[-1])
        # print("\n++++++++++\n Discrepancy between matched time arrays:")
        # print("t_start_observer - t_start_GT =\t", timestamps_observer[0] - timestamps_GT[0], "[s]")
        # print("t_end_observer - t_end_GT =\t\t", timestamps_observer[-1] - timestamps_GT[-1], "[s]")
        # print("Observer timestamps array length =\t\t", len(timestamps_observer))
        # print("Ground Truth timestamps array length =\t", len(timestamps_GT))
        

    
    #Extract coordinates
    if GROUND_TRUTH == True:
        x_GT = [p[0] for p in positions_GT]
        y_GT = [p[1] for p in positions_GT]
        z_GT = [p[2] for p in positions_GT]
    
    x_observer = [p[0] for p in positions_observer]
    y_observer = [p[1] for p in positions_observer]
    z_observer = [p[2] for p in positions_observer]

    x_estimated = [p[0] for p in positions_estimated]
    y_estimated = [p[1] for p in positions_estimated]
    z_estimated = [p[2] for p in positions_estimated]
    
    
    
    #Calibrate to zero initial value
    if GROUND_TRUTH == True:
        x_GT_cal = rounding( calibrate(x_GT), 9, "standard" )
        y_GT_cal = rounding( calibrate(y_GT), 9, "standard" )
        z_GT_cal = rounding( calibrate(z_GT), 9, "standard" )
    
    x_observer_cal = rounding( calibrate(x_observer), 9, "standard" )
    y_observer_cal = rounding( calibrate(y_observer), 9, "standard" )
    z_observer_cal = rounding( calibrate(z_observer), 9, "standard" )

    x_estimated_cal = rounding( calibrate(x_estimated), 9, "standard" )
    y_estimated_cal = rounding( calibrate(y_estimated), 9, "standard" )
    z_estimated_cal = rounding( calibrate(z_estimated), 9, "standard" )
    # print("\nOriginal vs. Calibrated positions:\n",x_observer, x_observer_cal)
    
    if GROUND_TRUTH == True:
        timestamps_GT_cal = rounding( calibrate(timestamps_GT), 9, "down" )
    timestamps_observer_cal = rounding( calibrate(timestamps_observer), 9, "down" )
    timestamps_estimated_cal = rounding( calibrate(timestamps_estimated), 9, "down" )

    if GROUND_TRUTH == True:
        headings_GT_cal = rounding( calibrate(headings_GT), 9, "standard" )
    headings_observer_cal = rounding( calibrate(headings_observer), 9, "standard" )
    headings_estimated_cal = rounding( calibrate(headings_estimated), 9, "standard" )
    
    
    
    #Filter Ground Truth to smoothened out noise for better plotting inly
    if GROUND_TRUTH == True:
        headings_GT_filtered = filterData(headings_GT_cal, 0.2)
        x_GT_filtered = filterData(x_GT_cal, 3)
        y_GT_filtered = filterData(y_GT_cal, 3)
        z_GT_filtered = filterData(z_GT_cal, 3)

    
    
    #Calculate Error Variance
    if GROUND_TRUTH == True:
        errors_x_segments = []
        errors_y_segments = []
        errors_heading_segments = []
        
        i = 1
        while i < (len(timestamps_observer_cal)-1):
            timestamps_observer_cal_segment = np.array([timestamps_observer_cal[i-1], timestamps_observer_cal[i]])
            
            timestamps_estimated_cal_segment, [headings_estimated_cal_segment, x_estimated_cal_segment, y_estimated_cal_segment] = \
                findMatchingData(timestamps_observer_cal_segment, timestamps_estimated_cal, [headings_estimated_cal, x_estimated_cal, y_estimated_cal])
            
            timestamps_GT_cal_segment, [headings_GT_cal_segment, x_GT_cal_segment, y_GT_cal_segment] = \
                findMatchingData(timestamps_observer_cal_segment, timestamps_GT_cal, [headings_GT_cal, x_GT_cal, y_GT_cal])
            
            errors_x_segments.append( x_estimated_cal_segment[-1] - x_GT_cal_segment[-1] )
            errors_y_segments.append( y_estimated_cal_segment[-1] - y_GT_cal_segment[-1] )
            errors_heading_segments.append( headings_estimated_cal_segment[-1] - headings_GT_cal_segment[-1] )
            
            i += 1
            
        variance_error_x = np.array(errors_x_segments).var()
        variance_error_y = np.array(errors_y_segments).var()
        variance_error_headings = np.array(errors_heading_segments).var()
        print("\n++++++++++")
        print("Variance x-position error (estimate vs. ground truth):\n",variance_error_x,"[m]")
        print("Variance y-position error (estimate vs. ground truth):\n",variance_error_y,"[m]")
        print("Variance heading error (estimate vs. ground truth):\n",variance_error_headings,"[deg]")

    

    #Calculate Errors
    E_x_observer_target = x_observer_cal[-1] - target_waypoints[-1][0]
    E_y_observer_target = y_observer_cal[-1] - target_waypoints[-1][1]
    E_heading_observer_target = headings_observer_cal[-1] - final_heading
    # print(target_waypoints[-1][0], x_GT[0], x_GT[-1], len(x_GT))
    print("\n++++++++++")
    print("Final Error x-position (observer vs. target):\n",E_x_observer_target,"[m]")
    print("Final Error y-position (observer vs. target):\n",E_y_observer_target,"[m]")
    print("Final Error heading (observer vs. target):\n",E_heading_observer_target,"[deg]")
        
    if GROUND_TRUTH == True:
        E_x_GT_target = x_GT_cal[-1] - target_waypoints[-1][0]
        E_y_GT_target = y_GT_cal[-1] - target_waypoints[-1][1]
        E_heading_GT_target = headings_GT_cal[-1] - final_heading
        # print(target_waypoints[-1][0], x_GT[0], x_GT[-1], len(x_GT))
        print("\n++++++++++")
        print("Final Error x-position (ground truth vs. target):\n",E_x_GT_target,"[m]")
        print("Final Error y-position (ground truth vs. target):\n",E_y_GT_target,"[m]")
        print("Final Error heading (ground truth vs. target):\n",E_heading_GT_target,"[deg]")
    
    
    
    #Calculate Error growth rate
    observer_timespan = (timestamps_observer_cal[-1] - timestamps_observer_cal[0])
    e_x_observer_target = E_x_observer_target / observer_timespan
    e_y_observer_target = E_y_observer_target / observer_timespan
    e_heading_observer_target = E_heading_observer_target / observer_timespan
    print("\n++++++++++")
    print("Covered timespan:\n",observer_timespan,"[s]")
    print("Error growth rate x-position (using final error between target and observed):\n",e_x_observer_target,"[m/s]")
    print("Error growth rate y-position (using final error between target and observed):\n",e_y_observer_target,"[m/s]")
    print("Error growth rate heading (using final error between target and observed):\n",e_heading_observer_target,"[deg/s]")
        
    if GROUND_TRUTH == True:
        GT_timespan = (timestamps_GT_cal[-1] - timestamps_GT_cal[0])
        e_x_GT_target = E_x_GT_target / GT_timespan
        e_y_GT_target = E_y_GT_target / GT_timespan
        e_heading_GT_target = E_heading_GT_target / GT_timespan
        print("\n++++++++++")
        print("Covered timespan:\n",GT_timespan,"[s]")
        print("Error growth rate x-position (using final error between target and ground truth):\n",e_x_GT_target,"[m/s]")
        print("Error growth rate y-position (using final error between target and ground truth):\n",e_y_GT_target,"[m/s]")
        print("Error growth rate heading (using final error between target and ground truth):\n",e_heading_GT_target,"[deg/s]")
    
    
    
    #Calculate RMS
    if GROUND_TRUTH == True:
        RMS_x_est_GT = rootMeanSquare(timestamps_estimated_cal, x_estimated_cal, timestamps_GT_cal, x_GT_cal)
        RMS_y_est_GT = rootMeanSquare(timestamps_estimated_cal, y_estimated_cal, timestamps_GT_cal, y_GT_cal)
        RMS_heading_est_GT = rootMeanSquare(timestamps_estimated_cal, headings_estimated_cal, timestamps_GT_cal, headings_GT_cal)
    RMS_x_est_observer = rootMeanSquare(timestamps_estimated_cal, x_estimated_cal, timestamps_observer_cal, x_observer_cal)
    RMS_y_est_observer = rootMeanSquare(timestamps_estimated_cal, y_estimated_cal, timestamps_observer_cal, y_observer_cal)
    RMS_heading_est_observer = rootMeanSquare(timestamps_estimated_cal, headings_estimated_cal, timestamps_observer_cal, headings_observer_cal)

    print("\n++++++++++")
    if GROUND_TRUTH == True:
        print("Length of Headings_GT (calibrated to 0 initial value):\n",len(headings_GT_cal))
    print("Length of Headings_observer (calibrated to 0 initial value):\n",len(headings_observer_cal))
    print("Length of Headings_estimated (calibrated to 0 initial value):\n",len(headings_estimated_cal))
    
    print("\n++++++++++")
    if GROUND_TRUTH == True:
        print("RMS x-position (estimated vs. ground truth):\n",RMS_x_est_GT,"[m]")
        print("RMS y-position (estimated vs. ground truth):\n",RMS_y_est_GT,"[m]")
        print("RMS Heading (ground truth vs. estimated):\n",RMS_heading_est_GT,"[deg]")
    print("RMS x-position (estimated vs. observer):\n",RMS_x_est_observer,"[m]")
    print("RMS y-position (estimated vs. observer):\n",RMS_y_est_observer,"[m]")
    print("RMS Heading (observer vs. estimated):\n",RMS_heading_est_observer,"[deg]")
    
    
    
    #PLOTTING
    # #Position
    fig = plt.figure(figsize=(4,3), dpi=350)
    ax = fig.add_axes([0.1, 0.1, 0.7, 0.9]) #This line created with help of AI
    ax.scatter([p[0] for p in target_waypoints], [p[1] for p in target_waypoints], label=f"{len(target_waypoints)} Target Waypoints", color="red")
    if GROUND_TRUTH == True:
        ax.plot(x_GT_cal, y_GT_cal, label="Position ground truth", color="lightgreen")
    ax.plot(x_observer_cal, y_observer_cal, label="Position observer", color="blue")
    ax.plot(x_estimated_cal, y_estimated_cal, label="Position estimated", color="orange")
    if GROUND_TRUTH == True:
        ax.plot(x_GT_filtered, y_GT_filtered, label="Position filtered ground truth", color="green", linestyle="-")
    gridlines = 10
    gridline_spacing = ( (np.ceil(10*max(plt.xlim()[1], plt.ylim()[1]))/10) - (np.floor(10*min(plt.xlim()[0], plt.ylim()[0]))/10) ) / gridlines
    ax.set_xticks(np.arange(np.floor(10*min(plt.xlim()[0], plt.ylim()[0]))/10, np.ceil(10*max(plt.xlim()[1], plt.ylim()[1]))/10, gridline_spacing))
    ax.set_yticks(np.arange(np.floor(10*min(plt.xlim()[0], plt.ylim()[0]))/10, np.ceil(10*max(plt.xlim()[1], plt.ylim()[1]))/10, gridline_spacing))
    ax.set_aspect("equal")
    ax.set_title(f"Log-File {k+1}", fontsize=10)#" \n $\\tau$ = {tau}, L = {L}")
    ax.set_xlabel("x-Position [m]", fontsize=8)
    ax.set_ylabel("y-Position [m]", fontsize=8)
    ax.tick_params(axis="both", labelsize=6) #This line created with help of AI
    ax.grid(True)
    ax.legend(loc="center left", bbox_to_anchor=(1.05,0.5), ncol=1, fontsize=7) #This line created with help of AI
    # ax.legend(loc="upper center", bbox_to_anchor=(0.5,-0.2), ncol=2, fontsize=7) #This line created with help of AI
    plt.tight_layout()
    plt.show()
    
    #Heading
    fig = plt.figure(figsize=(4,3), dpi=350)
    ax = fig.add_axes([0.1, 0.1, 0.7, 0.9]) #This line created with help of AI
    if GROUND_TRUTH == True: ax.plot(timestamps_GT_cal, headings_GT_cal, label="Heading ground truth", color="lightgreen")
    ax.plot(timestamps_observer_cal, headings_observer_cal, label="Heading observer", color="blue")
    ax.plot(timestamps_estimated_cal, headings_estimated_cal, label="Heading estimated", color="orange")
    if GROUND_TRUTH == True: ax.plot(timestamps_GT_cal, headings_GT_filtered, label="Heading filtered ground truth", color="green", linestyle="-")
    ax.set_title(f"Log-File {k+1}", fontsize=10)#" \n $\\tau$ = {tau}, L = {L}")
    ax.set_xlabel("timestamp [s]", fontsize=8)
    ax.set_ylabel("Heading [deg]", fontsize=8)
    ax.tick_params(axis="both", labelsize=6) #This line created with help of AI
    ax.grid(True)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5,-0.2), ncol=2, fontsize=7) #This line created with help of AI
    plt.tight_layout()
    plt.show()
    
    
#Part of this code has been written with support of Generative AI, where suggestions were critically examined and adapted. Lines are indicated in the code.