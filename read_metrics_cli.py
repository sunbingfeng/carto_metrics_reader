#!/usr/bin/env python

import sys
import os
import rospy
import json
import logging
import threading 
import time
import math
import argparse
import re
import pandas as pd

import numpy as np
from tabulate import tabulate

from cartographer_ros_msgs.srv import *
from cartographer_ros_msgs.msg import *

parser = argparse.ArgumentParser(description='Plot Cartographer Metrics')
parser.add_argument('--window_size', '-s', type=int, default=100, help='How long the monitor window will be.')
parser.add_argument('--lookup_interval', '-i', type=float, default=1, help='The period between two service call, unit: sec')
parser.add_argument('--dump_dir', '-d', type=str, default='/tmp/read_metrics/', help='Folder to save plots to')
parser.add_argument('--filters', '-f', type=str, nargs='+', help='filters through metrics name')

args = parser.parse_args()

print('window_size: ', args.window_size, ', lookup_interval: ', args.lookup_interval)

if not os.path.exists(args.dump_dir):
    os.mkdir(args.dump_dir)

MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY = "mapping_2d_pose_graph_work_queue_delay"

class PeriodicThread(object):
    """
    Python periodic Thread using Timer with instant cancellation
    """

    def __init__(self, callback=None, period=1, name=None, *args, **kwargs):
        self.name = name
        self.args = args
        self.kwargs = kwargs
        self.callback = callback
        self.period = period
        self.stop = False
        self.current_timer = None
        self.schedule_lock = threading.Lock()

    def start(self):
        """
        Mimics Thread standard start method
        """
        self.schedule_timer()

    def run(self):
        """
        By default run callback. Override it if you want to use inheritance
        """
        if self.callback is not None:
            self.callback(*self.args, **self.kwargs)

    def _run(self):
        """
        Run desired callback and then reschedule Timer (if thread is not stopped)
        """
        try:
            self.run()
        except Exception, e:
            logging.exception("Exception in running periodic thread")
        finally:
            with self.schedule_lock:
                if not self.stop:
                    self.schedule_timer()

    def schedule_timer(self):
        """
        Schedules next Timer run
        """
        self.current_timer = threading.Timer(self.period, self._run, *self.args, **self.kwargs)
        if self.name:
            self.current_timer.name = self.name
        self.current_timer.start()

    def cancel(self):
        """
        Mimics Timer standard cancel method
        """
        with self.schedule_lock:
            self.stop = True
            if self.current_timer is not None:
                self.current_timer.cancel()

    def join(self):
        """
        Mimics Thread standard join method
        """
        self.current_timer.join()

class MetricsManager:
    def __init__(self):
        # State variables
        self.paused = False
        self.graph_clear_requested = False

        self.counts_limit = args.window_size
        self.metrics_list_updated = False

        self.scalar_metrics = {MAPPING_2D_POSE_GRAPH_WORK_QUEUE_DELAY: {'stamp': 0, 'description': 'bla'}}
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
                    self.scalar_metrics[mf.name] = {}
                    self.scalar_metrics[mf.name]['description'] = mf.description
                    new_metric_added = True
                self.scalar_metrics[mf.name]['stamp'] = resp.timestamp.to_sec()
                # Counter or Gauge type, these two are both scalar types, and can be showed with 2d graph
                metrics = {}

                for m in mf.metrics:
                    is_histogram = True if m.type == 2 else False
                    if is_histogram:
                        # histogram metrics are not supported in cli.
                        continue
                    key = ''

                    if(len(m.labels) > 0):
                        for l in m.labels:
                            key = key + '[' + l.key + ']' + l.value
                    
                    metrics[key] = m.value

                self.scalar_metrics[mf.name]['labels'] = metrics
                metric_updated = True
        self.metrics_list_updated = new_metric_added
      except rospy.ServiceException as e:
        print ("Service call failed: ", e)

    def run(self):
        self.readScalarMetrics()
       
        if(self.scalar_metrics is None):
            return

        if args.filters:
            filtered_ = {k:v for k, v in self.scalar_metrics.items() if all(f in k for f in args.filters)}
        else:
            filtered_ = self.scalar_metrics
        data = []
        for k, v in filtered_.items():
            for l, lv in v['labels'].items():
				data.append([v['stamp'], k, l, lv])
        print(tabulate(data, headers=["time", "metric", "label", "value"], numalign="right", floatfmt=".2f"))

metrics_manager = MetricsManager()

timer = PeriodicThread(metrics_manager.run, args.lookup_interval)
timer.start()
