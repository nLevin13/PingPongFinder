"""
This example creates a room with reverberation time specified by inverting Sabine's formula.
This results in a reverberation time slightly longer than desired.
The simulation is pure image source method.
The audio sample with the reverb added is saved back to `examples/samples/guitar_16k_reverb.wav`.
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile

import pyroomacoustics as pra
from detect_peaks import detect_peaks
import numdifftools as nd
import pandas as pd

methods = ["ism", "hybrid"]

def simroom(room_dim, src_loc, mic_locs):

    parser = argparse.ArgumentParser(
        description="Simulates and adds reverberation to a dry sound sample. Saves it into `./examples/samples`."
    )
    parser.add_argument(
        "--method",
        "-m",
        choices=methods,
        default=methods[0],
        help="Simulation method to use",
    )
    args = parser.parse_args()

    # The desired reverberation time and dimensions of the room
    rt60_tgt = 0.3  # seconds
      # meters

    # import a mono wavfile as the source signal
    # the sampling frequency should match that of the room
    fs, audio = wavfile.read("examples/samples/guitar_16k.wav")

    # We invert Sabine's formula to obtain the parameters for the ISM simulator
    e_absorption, max_order = pra.inverse_sabine(rt60_tgt, room_dim)

    # Create the room
    room = pra.ShoeBox(room_dim, fs=fs, materials=pra.Material(e_absorption), max_order=max_order)
    

    room.add_source(src_loc, signal=audio, delay=0.5)


    # finally place the array in the room
    room.add_microphone_array(mic_locs)

    # Run the simulation (this will also build the RIR automatically)
    room.simulate()

    room.mic_array.to_wav(
        "examples/samples/guitar_16k_reverb_{}.wav".format(args.method),
        norm=True,
        bitdepth=np.int16,
    )
    """
    detect_peaks(room.mic_array.signals[0, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[1, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[2, :], mph=0, mpd=1000, threshold=10, show=True)

    print(max(room.mic_array.signals[0, :]))
    print(max(room.mic_array.signals[1, :]))
    print(max(room.mic_array.signals[2, :]))
    """
    return np.array([max(room.mic_array.signals[0, :]), max(room.mic_array.signals[1, :]), max(room.mic_array.signals[2, :])])

room_dim = [3, 3, 2]
#src_loc = np.array([0.1, 0.1, 2.5])
mic1_loc = np.array([0.1, 0.1, 0])
mic2_loc = np.array([2.9, 0.1, 0])
mic3_loc = np.array([1.5, 2.9, 0])

mic_locs = np.c_[
    mic1_loc, mic2_loc, mic3_loc # mic 1  # mic 2 # mic 3
]
print(mic_locs)

#simroom(room_dim, src_loc, mic_locs)
#making sure there are no source points that are on a mic location
x_min = 0.2
x_max = 2.8 #0.3 for testing
y_min = 0.2
y_max = 2.8 #0.3 for testing
z_min = 0.1
z_max = 0.9 #3.3
step_size = 0.1
x_num = int((x_max-x_min)/step_size + 1)
y_num = int((y_max-y_min)/step_size + 1)
z_num = int((z_max-z_min)/step_size + 1)
print(x_num, y_num, z_num)
arr_len = x_num*y_num*z_num
print(arr_len)
#our y's
srcs = np.zeros((arr_len, 3))
n = 0
for i in np.linspace(x_min, x_max, num=x_num):
	for j in np.linspace(y_min, y_max, num=y_num):
		for k in np.linspace(z_min, z_max, num=z_num):
			srcs[n] = np.round(np.array([i, j, k]), decimals=1)
			n += 1
print(srcs[10])

#print(grid[0], grid [100], grid[arr_len-1])
#y = np.array()
#find x's
intensities = np.zeros((arr_len, 3))
n = 0
for src in srcs:
	print(src)
	intensities[n] = simroom(room_dim, src, mic_locs)
	print(n, intensities[n])
	n += 1
#print(intensities[0], intensities[100], intensities[arr_len-1])
simdata = np.concatenate((srcs, intensities), axis=1)
np.savetxt('simdata2.csv', simdata, delimiter=',', fmt='%10.5f')#, fmt='%d')
