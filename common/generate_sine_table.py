#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse

# N = 8000

parser = argparse.ArgumentParser(description="generate a single-cycle sine lookup table, formatted as a C header file. Prints to stdout.")
parser.add_argument("--num_points", "-N", type=int, default=8000, help="Number of points in table (default: 8000)")
parser.add_argument("--plot", "-p", action="store_true", help="Display a plot of the generated sine wave")

args = parser.parse_args()
N = args.num_points
x = np.arange(0, N)
y = (1000 * np.sin(2*np.pi*x/N)).astype(int)

print(f"int16_t sine{N}[{N}] = " + "{")
with np.printoptions(threshold=np.inf):
    print(np.array2string(y, separator=", ").replace("[", " ").replace("]", ""))
print("};")


if args.plot:
    plt.plot(x, y)
    plt.show()
