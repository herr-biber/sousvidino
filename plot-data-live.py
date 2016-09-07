#!/usr/bin/env python2

import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import json
import time
import numpy as np
from collections import defaultdict

plt.ion()

p = None

needed_keys = set([u'loop_time', u'ki', u'sp', u't2', u't0', u't1', u'p', u'autotune', u'kd', u'output', u'kp'])


while True:
    with open('current-data.json') as fp:
        try:
            data = json.load(fp)
        except:
            continue
    ts = []
    vals = defaultdict(list)
    for t, d in data:
        if not needed_keys.issubset(set(d.keys())):
#            print "skipping garbage: ", d
            continue
        ts.append(t)
        for k,v in d.iteritems():
            assert k != "", v
            assert k in needed_keys, k
            vals[k].append(v)

    # make timestamps relative
    ts_beg = ts[0]
    ts = [t - ts_beg for t in ts]

    t0 = vals['t0']
    output = vals['output']
    sp = vals['sp'][-1]
    print len(ts), np.mean(t0), np.std(t0)
    duration = (ts[-1] - ts[0]) / 60.
    print "duration: %.1f min" % duration
    
    if p is None:
#    if True:
        print '@', len(ts), len(t0)
        p = plt.plot(ts, t0, color='r')
        p2 = plt.plot(ts, output, color='g')
        p3 = plt.plot(ts, vals['sp'], color='y')
    else:
        p[0].set_data(ts, t0)
        p[0].get_axes().set_ylim(sp - 1.5, sp + 1.5)
#        p[0].get_axes().set_ylim(20.0, sp + 1.5)
        p[0].get_axes().set_xlim(ts[0], ts[-1])
        p2[0].set_data(ts, [0.25*o + sp -1.5 for o in output])
        p3[0].set_data(ts, vals['sp'])
    plt.pause(1.0)

