#! /usr/bin/env python

import numpy as np

loopsize = 48 # samples in full sine cycle

x = np.linspace(0, 1, loopsize, endpoint=False)
y = (10000 * np.sin(2*np.pi*x)).astype(int)

print(np.array2string(y, separator=", "))
