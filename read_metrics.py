#!/usr/bin/env python

import sys
import os
import rospy
import json
import threading 
import time
import math
import argparse
import re
import pandas as pd

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import numpy as np

import tkinter as tk
from tkinter import ttk
from tkinter import Tk, Text, TOP, BOTH, X, Y, N, LEFT
from tkinter.ttk import Frame, Label, Entry

from cartographer_ros_msgs.srv import *
from cartographer_ros_msgs.msg import *

parser = argparse.ArgumentParser(description='Plot Cartographer Metrics')
parser.add_argument('--window_size', '-s', type=float, default=100, help='How long the monitor window will be.')
parser.add_argument('--lookup_interval', '-i', type=float, default=1, help='The period between two service call, unit: sec')
parser.add_argument('--dump_dir', '-d', type=str, default='/tmp/read_metrics/', help='Folder to save plots to')

args = parser.parse_args()

if not os.path.exists(args.dump_dir):
    os.mkdir(args.dump_dir)

LARGE_FONT= ("Verdana", 12)

MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY = "mapping_2d_pose_graph_work_queue_delay"

class CartoMetricsShowApp(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.iconbitmap(self)
        tk.Tk.wm_title(self, "Cartographer metrics plot")
        
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand = True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frame = ShowScalarGraph(container, self)
        self.frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(ShowScalarGraph)

    def show_frame(self, cont):

        self.frame.tkraise()

class ShowScalarGraph(tk.Frame):

    def __init__(self, parent, controller):
        # State variables
        self.label_width_default = 10
        self.paused = False
        self.graph_clear_requested = False
       
        self.time_range = float(args.window_size)
        self.refresh_interval = float(args.lookup_interval)
        self.counts_limit = args.window_size / args.lookup_interval
        self.metrics_list_updated = False

        self.scalar_metrics = {MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY: []}
        self.metrics_description = {MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY: 'Age of the oldest entry in the work queue in seconds'}
        self.stamps = []

        tk.Frame.__init__(self, parent)

        # Create a metric selection option menu
        frame1 = Frame(self)
        frame1.pack(fill=X)

        self.scalar_metric_selection = tk.StringVar(self)
        choices = self.scalar_metrics.keys()
        self.scalar_metric_selection.set(MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY) # set the default option

        choose_label = tk.Label(frame1, text="Metric", width=self.label_width_default, underline=0)
        choose_label.pack(side = LEFT, pady=5,padx=5)
        self.scalar_popup_menu = tk.OptionMenu(frame1, self.scalar_metric_selection, *choices)
        self.scalar_popup_menu.pack(fill = X, pady=5, padx=5)
         
        # link function to change dropdown
        self.scalar_metric_selection.trace('w', self.onChangeMetric)

        # Create a metric label selection option menu
        frame2 = Frame(self)
        frame2.pack(fill=X)

        self.metric_label_selection = tk.StringVar(self)
        label_choices = ['[Gauge/Counter]']
        self.metric_label_selection.set('[Gauge/Counter]') # set the default option

        choose_label = tk.Label(frame2, text="Label", width=self.label_width_default, underline=0)
        choose_label.pack(side = LEFT, pady=5, padx=5)
        self.metric_label_popup_menu = tk.OptionMenu(frame2, self.metric_label_selection, *label_choices)
        self.metric_label_popup_menu.pack(fill = X, pady=5, padx=5)

        # Create a description text label
        frame3 = Frame(self)
        frame3.pack(fill=X)

        description_label = tk.Label(frame3, text="Description", width=self.label_width_default, underline=0)
        description_label.pack(side = LEFT, pady=5, padx=5)

        self.description_label_var = tk.StringVar(self)
        description_content_label = tk.Label(frame3, textvariable=self.description_label_var)
        description_content_label.pack(pady=5, padx=5, expand = True)
        self.description_label_var.set(self.metrics_description[MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY])

        # Create a filter entry
        frame5 = Frame(self)
        frame5.pack(fill=X)

        filter_label = tk.Label(frame5, text="Filter", width=self.label_width_default, underline=0)
        filter_label.pack(side = LEFT, pady=5, padx=5)

        self.filter_text = tk.StringVar(self)
        self.filter_entry = tk.Entry(frame5, textvariable=self.filter_text, bd=5)
        self.filter_entry.bind('<Key-Return>', self.onFilter)
        self.filter_entry.pack(side=LEFT, fill=X, pady=5, padx=5, expand=True)

        clear_filter_btn = tk.Button(frame5, text='X', command=self.onClearFilterText)
        clear_filter_btn.pack(side = LEFT, padx = 5, pady = 5)

        # Create a scale widget to set the watching window and time interval dynamically.
        frame6 = Frame(self)
        frame6.pack(fill=X)
        time_range_scale_widget = tk.Scale(frame6, label='Time range in seconds', command=self.onSetTimeRange, orient=tk.HORIZONTAL, from_=5, to=200, length=300)
        time_range_scale_widget.set(self.time_range)
        time_range_scale_widget.pack(side=LEFT, pady=5, padx=5)
        interval_scale_widget = tk.Scale(frame6, label='Refresh interval in seconds', command=self.onSetRefreshInterval, orient=tk.HORIZONTAL, from_=0.1, to=2.0, resolution=0.1, length=300)
        interval_scale_widget.set(self.refresh_interval)
        interval_scale_widget.pack(side=LEFT, pady=5, padx=5)

        # Create operation button
        frame4 = Frame(self)
        frame4.pack(fill=Y)
        btn_clear = tk.Button(frame4, text='Clear', command=self.onClearGraph)
        btn_clear.pack(side = LEFT, padx = 5, pady = 5)

        self.btn_pause_label_var = tk.StringVar(self)
        self.btn_pause_label_var.set('Pause')
        btn_pause = tk.Button(frame4, textvariable=self.btn_pause_label_var, command=self.onPauseOrResumeGraph)
        btn_pause.pack(side = LEFT, padx = 5, pady = 5)

        btn_save_plot = tk.Button(frame4, text='Save', command=self.onSaveCurrentPlot)
        btn_save_plot.pack(side = LEFT, padx = 5, pady = 5)

        btn_save_all_plot = tk.Button(frame4, text='SaveAll', command=self.onSaveAllPlot)
        btn_save_all_plot.pack(side = LEFT, padx = 5, pady = 5)

        self.f = Figure(figsize=(5,5), dpi=100)
        self.a = self.f.add_subplot(111)
        self.a.axis((0, self.counts_limit - 1, 0, 10))
        self.a.set_autoscaley_on(True)

        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        self.toolbar.update()
        self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # initial time display
        self.onUpdate()

    def onSetTimeRange(self, value):
        self.time_range = float(value)
        self.counts_limit = self.time_range / self.refresh_interval
        
    def onSetRefreshInterval(self, value):
        self.refresh_interval = float(value)
        self.counts_limit = self.time_range / self.refresh_interval

    def onFilter(self, event):
        print('Filter with [', self.filter_text.get(), ']')
        filters = self.filter_text.get().split(';')
        filtered_keys = {k for k, v in self.scalar_metrics.items() if all(f in k for f in filters)}

        menu = self.scalar_popup_menu["menu"]
        menu.delete(0, "end")
        for m in filtered_keys:
            menu.add_command(label=m, 
                             command=lambda value=m: self.scalar_metric_selection.set(value))

    def onClearFilterText(self):
        self.filter_entry.delete(0, tk.END)

        menu = self.scalar_popup_menu["menu"]
        menu.delete(0, "end")
        for m in self.scalar_metrics.keys():
            menu.add_command(label=m, 
                             command=lambda value=m: self.scalar_metric_selection.set(value))

    def onClearGraph(self):
        self.graph_clear_requested = True

    def onPauseOrResumeGraph(self):
        self.paused = not self.paused
        if(not self.paused):
            # When switch from pause state, clear history graph
            self.graph_clear_requested = True

        self.btn_pause_label_var.set('Pause' if not self.paused else 'Resume')

    def SavePlot(self, metric_name, metric_label):
        counts = len(self.scalar_metrics[metric_name])
        is_histogram = True if metric_label.startswith('[Histogram]') else False
        # Only support Gauge/Count metric to save
        if not is_histogram: 
            records = []
            idx = 0
            for m in self.scalar_metrics[metric_name]:
                if not self.stamps[idx] == 0.0:
                    # stamp with '0.0' value should be ignored
                    records.append([self.stamps[idx], m[metric_label]])
                idx = idx + 1

            history = pd.DataFrame(records, columns=['Stamp', 'Value'])
            name = metric_name + '_' + metric_label
            name = name.replace("[Gauge/Counter]", "")
            name = name.replace("[", "_")
            name = name.replace("]", "_")

            path = os.path.join(args.dump_dir, name + '.csv')
            history.to_csv(path, index=False)
            print('Dump plot to path [', path, '] finished.')
    def onSaveCurrentPlot(self):
        self.SavePlot(self.scalar_metric_selection.get(), self.metric_label_selection.get())
        print('Finished saving current plot.')

    def onSaveAllPlot(self):
        all_metrics = self.scalar_metrics.keys()
        for metric in self.scalar_metrics:
           values = self.scalar_metrics[metric][-1] 
           for label in values.keys():
            self.SavePlot(metric, label)
        print('Finished saving all plots.')

    # on change dropdown value
    def onChangeMetric(self, *args):
        label_menu = self.metric_label_popup_menu["menu"]
        label_menu.delete(0, "end")

        latest_metrics = self.scalar_metrics[self.scalar_metric_selection.get()][-1]
        for l in latest_metrics.keys():
            # Update the default selection
            self.metric_label_selection.set(l)

            label_menu.add_command(label=l, 
                             command=lambda value=l: self.metric_label_selection.set(value))

        self.description_label_var.set(self.metrics_description[self.scalar_metric_selection.get()])

    def readScalarMetrics(self):
      rospy.wait_for_service('/read_metrics', timeout=3.0)

      new_metric_added = False
      metric_updated = False

      try:
        metrics_proxy = rospy.ServiceProxy('/read_metrics', ReadMetrics)
        resp = metrics_proxy()

        for mf in resp.metric_families:
            if len(mf.metrics) == 0:
                pass
            else:
                if(mf.name not in self.scalar_metrics):
                    self.scalar_metrics[mf.name] = []
                    self.metrics_description[mf.name] = mf.description
                    new_metric_added = True
                # Counter or Gauge type, these two are both scalar types, and can be showed with 2d graph

                metrics = {}

                for m in mf.metrics:
                    key = '[Histogram]' if m.type == 2 else '[Gauge/Counter]'
                    is_histogram = True if m.type == 2 else False

                    if(len(m.labels) > 0):
                        for l in m.labels:
                            key = key + '[' + l.key + ']' + l.value
                    
                    if(is_histogram):
                        bucket_count_pair = {}
                        for c in m.counts_by_bucket:
                            bucket_count_pair[c.bucket_boundary] = c.count
                        metrics[key] = bucket_count_pair
                    else:
                        metrics[key] = m.value


                self.scalar_metrics[mf.name].append(metrics)
                metric_updated = True

                # Histogram metric does not need to be buffered.
                max_metric_buffered = 1 if is_histogram else self.counts_limit
                if len(self.scalar_metrics[mf.name]) > max_metric_buffered:
                    self.scalar_metrics[mf.name].pop(0)

        self.metrics_list_updated = new_metric_added
        if metric_updated:
            self.stamps.append(resp.timestamp.to_sec())            
            if len(self.stamps) > self.counts_limit:
                self.stamps.pop(0)
 
      except rospy.ServiceException as e:
        print ("Service call failed: ", e)

    def onUpdate(self):
        if(self.graph_clear_requested):
            self.scalar_metrics = {}
            self.stamps = []
            self.graph_clear_requested = False
            self.a.cla()

        if(not self.paused):
            self.readScalarMetrics()

        if(self.metrics_list_updated):
            menu = self.scalar_popup_menu["menu"]
            menu.delete(0, "end")
            for m in self.scalar_metrics.keys():
                menu.add_command(label=m, 
                                 command=lambda value=m: self.scalar_metric_selection.set(value))

        counts = len(self.scalar_metrics[self.scalar_metric_selection.get()])
        if(counts == 0):
            return

        is_histogram = True if self.metric_label_selection.get().startswith('[Histogram]') else False
        # print('label: ', self.metric_label_selection.get(), 'is Histogram' if is_histogram else 'is not Histogram')

        self.a.cla()

        if not is_histogram: 
            x_range = np.arange(self.counts_limit - counts, self.counts_limit) * self.refresh_interval
            y_range = []
            for m in self.scalar_metrics[self.scalar_metric_selection.get()]:
                y_range.append(m[self.metric_label_selection.get()])

            self.a.axis((0, (self.counts_limit-1)*self.refresh_interval, 0, 10))
            self.a.set_xscale('linear')
            self.a.set_autoscaley_on(True)
            self.a.plot(x_range, y_range)
        else:
            # Ensure that only one record exists.
            counts_by_bucket = self.scalar_metrics[self.scalar_metric_selection.get()][0][self.metric_label_selection.get()]

            x_range = []
            y_range = []
            for (k, v) in counts_by_bucket.items():
                x_range.append(k)
                y_range.append(v)

            self.a.set_xscale('log')
            self.a.plot(x_range, y_range, '.')
            for i_x, i_y in zip(x_range, y_range):
                self.a.annotate('({:.2f}, {:.2f})'.format(i_x, i_y), xy=(i_x, i_y))
            self.canvas.draw()

        self.canvas.draw()
        self.after(int(self.refresh_interval * 1000), self.onUpdate)
        

app = CartoMetricsShowApp()
app.mainloop()
