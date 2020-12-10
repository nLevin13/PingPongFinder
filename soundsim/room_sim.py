"""
This example creates a room with reverberation time specified by inverting Sabine's formula.
This results in a reverberation time slightly longer than desired.
The simulation is pure image source method.
The audio sample with the reverb added is saved back to `examples/samples/guitar_16k_reverb.wav`.
"""
from __future__ import division
import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile

import pyroomacoustics as pra
from detect_peaks import detect_peaks
import numdifftools as nd

methods = ["ism", "hybrid"]

def sim(src_loc, mic_locs, noise=False):

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
    rt60_tgt = 0.166  # seconds, original was 0.3, IRL this would probably be closest by digitally removing reverb
    room_dim = [10, 10, 3.5]  # meters
    #room_dim = [5, 5, 3]

    # import a mono wavfile as the source signal
    # the sampling frequency should match that of the room
    fs, audio = wavfile.read("examples/samples/guitar_16k.wav")

    # We invert Sabine's formula to obtain the parameters for the ISM simulator
    e_absorption, max_order = pra.inverse_sabine(rt60_tgt, room_dim)

    # Create the room
    if args.method == "ism":
        room = pra.ShoeBox(
            room_dim, fs=fs, materials=pra.Material(e_absorption), max_order=max_order
        )
    elif args.method == "hybrid":
        room = pra.ShoeBox(
            room_dim,
            fs=fs,
            materials=pra.Material(e_absorption),
            max_order=3,
            ray_tracing=True,
            air_absorption=True,
        )

    # place the source in the room
    #src_loc = [5, 5, 1]
    

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

    
    print(fs)
    detect_peaks(room.mic_array.signals[0, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[1, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[2, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[3, :], mph=0, mpd=1000, threshold=10, show=True)
    
    sig1 = room.mic_array.signals[0, :]
    sig2 = room.mic_array.signals[1, :]
    sig3 = room.mic_array.signals[2, :]
    sig4 = room.mic_array.signals[3, :]

    div = 100 #10 is where things start to break down
    if noise:
    	sig1 = sig1 + np.random.normal(0, max(sig1)/div, sig1.shape)
    	sig2 = sig2 + np.random.normal(0, max(sig2)/div, sig2.shape)
    	sig3 = sig3 + np.random.normal(0, max(sig3)/div, sig3.shape)
    	sig4 = sig4 + np.random.normal(0, max(sig4)/div, sig4.shape)

    sigs = [sig1, sig2, sig3, sig4]
    """
    for sig in sigs:
    	detect_peaks(sig, mph=0, mpd=1000, threshold=10, show=True)
    """

    I1 = max(sig1)
    I2 = max(sig2)
    I3 = max(sig3)
    I4 = max(sig4)
    

    i1 = np.where(sig1 == I1)[0][0]
    i2 = np.where(sig2 == I2)[0][0]
    i3 = np.where(sig3 == I3)[0][0]
    i4 = np.where(sig4 == I4)[0][0]
    print("SRC's: " + str(src_loc))
    print(I1, I2, I3, I4)
    print(i1, i2, i3, i4)
    return fs, i1, i2, i3, i4
