"""
Copyright
David Speck
david.speck@pluto.uni-freiburg.de
"""

from sys import argv
from matplotlib import pyplot as plt
import os
import numpy as np

USAGE = """
python eval_data.py i [o]

i = path to top input folder (e.g. [...]/shakey_quickscenario/eval/example)
    - if more then one run is located in this folder it will process all
        and builts plots over the average
    
o = path to top output folder saving plots (default: 'i'/plots)

This script generates relevant evaluation plots for a collection of
shakey 2016 runs or a single shakey 2016 run.
"""

def parseActionTimes(fileName='action.times'):
    doc_id = -1
    files = dict()
    
    # Hack for checking if collection of folders
    if os.path.isfile(path + fileName):
        folders = [""]
    else:
        folders = os.listdir(path)
    
    for folder in folders:
        if "plot" in folder:
            #print ("Skipped " + folder + "...")
            continue
        cur_path = path + folder + "/" + fileName
        print("Parsing " + folder + "/" + fileName + "...")
        try:
            with open(cur_path) as file:
                doc_id += 1
                files[doc_id] = []
                for line in file:
                    entries = line.replace("\n", "")
                    entries = entries.split(" ")
                    files[doc_id].append(entries)
                file.close()
        except IOError:
            print("No such file found " + path + folder + "/" + fileName)
    return files

def parsePlanningTimes(path, filename='plan.times'):
    run_id = -1
    plan_times = [.0, .0, .0, .0, .0]
    
    # Hack for checking if collection of folders
    if os.path.isfile(path + "action.times"):
        run_folders = [""]
    else:
        run_folders = listdirs(path)
	run_folders = [x + "/" for x in run_folders]     
    #print(run_folders)
    num_all = 0
    for run in run_folders:
        if "plot" in run:
            #print ("Skipped " + run + "...")
            continue
        run_id += 1
        cur_path = path + run
        plan_step_folders = listdirs(cur_path)
        num = 0
        for plan_step in plan_step_folders:
            if "plot" in plan_step:
                #print ("Skipped " + run + "...")
                continue
            plan_path = cur_path + plan_step + "/" + "lazy_eval_partial_caching/" + filename
            #print(plan_path)
            try:
                with open(plan_path) as file:
                    for line in file:
                        cur_vals = line.replace("\n", "").split(" ")
                        if cur_vals[0] == "-1.000000":
                            #print("search: ", cur_vals)
                            for i in range(5):
                                plan_times[i] += float(cur_vals[i])
                    num += 1
                    """for line in file:
                        if line_nr == 3:
                            cur_vals = line.replace("\n", "").split(" ")
                            print("search: ", cur_vals)
                            for i in range(5):
                                runs_search[i] += float(cur_vals[i])
                        if line_nr == 4:
                            cur_vals = line.replace("\n", "").split(" ")
                            print("total: ", cur_vals)
                            for i in range(5):
                                runs_total[i] += float(cur_vals[i])"""
            except IOError:
                print("No such file found " + plan_path)
        #print(num)
        num_all += num
    av_plan_times = plan_times[:]
    for i in range(len(av_plan_times)):    
        av_plan_times[i] /= run_id+1
    return plan_times[-1], av_plan_times[-1]

def listdirs(folder):
    return [d for d in os.listdir(folder) if os.path.isdir(os.path.join(folder, d))]



def numberRunSuccess(data):
    success = failure = 0
    for run in data:
        if int(data[run][-1][-2]) == 1:
            success += 1
        else:
            failure += 1
    return success, failure, success / (success + failure + 0.)

def numberActionSuccess(data):
    success = failure = 0
    for run in data:
        for action in data[run]:
            if int(action[-2]) == 1:
                success += 1
            else:
                failure += 1
    return success, failure, success / (success + failure + 0.)

def dataActions(data):
    actions = dict()
    for run in data:
        for action in data[run]:
            name = action[1][1:]
            if name not in actions:
                # nr, time, success
                actions[name] = [0, 0, 0]
            actions[name][0] += 1
            actions[name][1] += float(action[-1])
            actions[name][2] += int(action[-2])
    return actions

def createTimePlot(data, dataAc):
    fig, ax = plt.subplots(figsize=(20, 10))
    ind = np.arange(len(dataAc) + 1)
    res = [0]
    avg = [0]
    label = ["Actions in Runs"]
    for run in data:
        for ac in data[run]:
            res[0] += float(ac[-1])
    avg[0] = res[0] / (len(data) + 0.)
    for ac in dataAc:
        label.append(ac)
        res.append(dataAc[ac][1] / (60 + 0.))
        avg.append((dataAc[ac][1] / (dataAc[ac][0] + 0.)) / 60.0)
    width = 0.35
    rects1 = ax.bar(ind, tuple(res), width, color='r')
    rects2 = ax.bar(ind + width, tuple(avg), width, color='y')
    # add some text for labels, title and axes ticks
    ax.set_title('Time of executing actions')
    ax.set_ylabel('Time in min')
    ax.set_xticks(ind + width)
    ax.set_xticklabels(tuple(label), rotation=90)
    ax.set_yscale('symlog')

    ax.legend((rects1[0], rects2[0]), ('Overall', 'Average'))
    fig.savefig(output_path + 'actionTime.jpg', transparent=False, bbox_inches='tight', pad_inches=1)
    return (res[0], avg[0])

def createActionCallPlot(data, dataAc):
    fig, ax = plt.subplots(figsize=(20, 10))
    ind = np.arange(len(dataAc) + 1)
    nr = [0]
    avg = [0]
    label = ["Runs"]
    for ac in dataAc:
        label.append(ac)
        nr[0] += dataAc[ac][0]
        nr.append(dataAc[ac][0])
        avg.append(dataAc[ac][0] / (len(data) + 0.0))
    avg[0] = nr[0] / len(data)
    width = 0.35
    rects1 = ax.bar(ind, tuple(nr), width, color='r')
    rects2 = ax.bar(ind + width, tuple(avg), width, color='y')
    # add some text for labels, title and axes ticks
    ax.set_ylabel('Action calls')
    ax.set_title('Amount of action calls')
    ax.set_xticks(ind + width)
    ax.set_xticklabels(tuple(label), rotation=90)
    #ax.set_yscale('symlog')

    ax.legend((rects1[0], rects2[0]), ('Overall', 'Average per Run'))
    fig.savefig(output_path + 'actionCalls.jpg', transparent=False, bbox_inches='tight', pad_inches=1)

def createSuccessPlot(data, dataAc):
    fig, ax = plt.subplots(figsize=(20, 10))
    ind = np.arange(2+ len(dataAc))
    success = []
    failure = []
    successRun, failureRun, percentageRun = numberRunSuccess(data)
    successAc, failureAc, percentageAc = numberActionSuccess(data)
    success.append(percentageRun)
    success.append(percentageAc)
    failure.append(1 - percentageRun)
    failure.append(1 - percentageAc)
    label = ["Runs", "Actions"]
    for ac in dataAc:
        label.append(ac)
        percentage = dataAc[ac][2] / (dataAc[ac][0] + 0.)
        success.append(percentage)
        failure.append(1 - percentage)
    width = 0.35
    #print(tuple(success))
    rects1 = ax.bar(ind, tuple(success), width, color='g')
    rects2 = ax.bar(ind + width, tuple(failure), width, color='r')
    ax.set_ylabel('Percentage')
    ax.set_title('Success ratio of runs and actions')
    ax.set_xticks(ind + width)
    ax.set_xticklabels(tuple(label), rotation=90)
    ax.legend((rects1[0], rects2[0]), ('Success', 'Failure'))
    plt.ylim([0,1.2])
    fig.savefig(output_path + 'success.jpg', transparent=False, bbox_inches='tight', pad_inches=1)
    
def createRunTimePlot(avg_action_times, avg_plan_times):
    run_time = (avg_action_times + avg_plan_times) / 60.0
    action_time = avg_action_times / 60.0
    plan_time = avg_plan_times / 60.0
    fig, ax = plt.subplots(figsize=(20, 10))
    ind = np.arange(3)
    width = 0.35
    rects1 = ax.bar(ind, tuple([run_time, action_time, plan_time]), width, color=['b', 'r', 'g'])
    label = ["Run", "Action Execution", "Planning"]
    ax.set_xticks(ind + width/2.0)
    ax.set_xticklabels(tuple(label), rotation=90)
    ax.set_ylabel('Time in min')
    ax.set_title('Average Runtimes')
    #ax.legend('Average')
    ax.set_yscale('symlog')
    fig.savefig(output_path + 'runtime.jpg', transparent=False, bbox_inches='tight', pad_inches=1)
      

if __name__ == '__main__':
    if len(argv) < 2 or len(argv) > 3:
        print(USAGE)
        quit()
    global path
    global output_path
    
    path = argv[1]
    if (path[-1] != "/"):
        path += "/"
    
    # Create default output folder
    if len(argv) == 3:
        output_path = argv[2]
        if output_path[-1] != "/":
            output_path += "/"
    else:
        if path[-1] == "/":
            output_path = path[:-1] + "_plots/"
        else:
            output_path = path + "_plots/"
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    print("---------------------------------------------------------------------------")
    data = parseActionTimes()
    successRun, failureRun, percentageRun = numberRunSuccess(data)
    successAc, failureAc, percentageAc = numberActionSuccess(data)
    print("Runs - Success: %d, Failure: %d, Percentage: %.3f" % (successRun, failureRun, percentageRun))
    print("Actions - Success: %d, Failure: %d, Percentage: %.3f" % (successAc, failureAc, percentageAc))
    dataAc = dataActions(data)
    """for run in data:
        for ac in data[run]:
            print(ac)
        print("\n")"""
    plt.rcParams.update({'font.size': 40})
    #print(dataAc)
    createSuccessPlot(data, dataAc)
    createActionCallPlot(data, dataAc)
    overallActionTime, avgActionTime = createTimePlot(data, dataAc)
    overallPlanTimes, avgPlanTtime = parsePlanningTimes(path)
    createRunTimePlot(avgActionTime, avgPlanTtime)
    print("Number of runs: ", len(data))
    print("AVG action times (sec): ", avgActionTime)
    print("AVG plan times (sec): ", avgPlanTtime)
    print("AVG overall time (sec): ", avgActionTime + avgPlanTtime)
    print("Saved all plots in: ", output_path)
    print("---------------------------------------------------------------------------")
    #createActionCallPlot(data, dataAc)


