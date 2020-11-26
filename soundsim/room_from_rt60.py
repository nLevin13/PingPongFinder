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

methods = ["ism", "hybrid"]

if __name__ == "__main__":

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
    room_dim = [10, 10, 3.5]  # meters

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
    src_loc = [5, 5, 1]
    room.add_source(src_loc, signal=audio, delay=0.5)

    # define the locations of the microphones
    mic1_loc = [0.5, 0.5, 1]
    mic2_loc = [9.5, 9.5, 1]
    mic3_loc = [5, 9.5, 1]
    mic_locs = np.c_[
        mic1_loc, mic2_loc, mic3_loc # mic 1  # mic 2 # mic 3
    ]

    # finally place the array in the room
    room.add_microphone_array(mic_locs)

    # Run the simulation (this will also build the RIR automatically)
    room.simulate()

    room.mic_array.to_wav(
        "examples/samples/guitar_16k_reverb_{}.wav".format(args.method),
        norm=True,
        bitdepth=np.int16,
    )

    # measure the reverberation time
    """rt60 = room.measure_rt60()
                print("The desired RT60 was {}".format(rt60_tgt))
                print("The measured RT60 is {}".format(rt60[1, 0]))"""
    """
    # Create a plot
    plt.figure()

    # plot one of the RIR. both can also be plotted using room.plot_rir()
    rir_1_0 = room.rir[1][0]
    plt.subplot(2, 1, 1)
    plt.plot(np.arange(len(rir_1_0)) / room.fs, rir_1_0)
    plt.title("The RIR from source 0 to mic 1")
    plt.xlabel("Time [s]")

    # plot signal at microphone 1
    plt.subplot(2, 1, 2)
    plt.plot(room.mic_array.signals[1, :])
    plt.title("Microphone 1 signal")
    plt.xlabel("Time [s]")

    plt.tight_layout()
    plt.show()
    """
    #print(len(room.mic_array.signals[1, :]))
    #print(len(detect_peaks(room.mic_array.signals[1, :])))
    print(max(detect_peaks(room.mic_array.signals[0, :], mph=0, mpd=1000, threshold=10, show=False)))
    print(max(detect_peaks(room.mic_array.signals[1, :], mph=0, mpd=1000, threshold=10, show=False)))
    print(max(detect_peaks(room.mic_array.signals[2, :], mph=0, mpd=1000, threshold=10, show=False)))
    print(max(room.mic_array.signals[0, :]))
    print(max(room.mic_array.signals[1, :]))
    print(max(room.mic_array.signals[2, :]))
    detect_peaks(room.mic_array.signals[0, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[1, :], mph=0, mpd=1000, threshold=10, show=True)
    detect_peaks(room.mic_array.signals[2, :], mph=0, mpd=1000, threshold=10, show=True)
    I1 = max(detect_peaks(room.mic_array.signals[0, :], mph=0, mpd=1000, threshold=10, show=False))
    I2 = max(detect_peaks(room.mic_array.signals[1, :], mph=0, mpd=1000, threshold=10, show=False))
    I3 = max(detect_peaks(room.mic_array.signals[2, :], mph=0, mpd=1000, threshold=10, show=False))

    def func12(x):
        return (((x[0]-mic1_loc[0])**2 + (x[1]-mic1_loc[1])**2) - I2*((x[0]-mic2_loc[0])**2 + (x[1]-mic2_loc[1])**2)/I1)

    def func23(x):
        return (((x[0]-mic2_loc[0])**2 + (x[1]-mic2_loc[1])**2) - I3*((x[0]-mic3_loc[0])**2 + (x[1]-mic3_loc[1])**2)/I2)

    def func31(x):
        return (((x[0]-mic3_loc[0])**2 + (x[1]-mic3_loc[1])**2) - I1*((x[0]-mic1_loc[0])**2 + (x[1]-mic1_loc[1])**2)/I3)

    #Here we are trying to find the intersection point of the two circles
    step_size = 0.0008
    x_curr = [5, 6] #starting point
    #Now iterating to find point of intersection of circles 1 and 2
    """
    f_curr = func12(x_curr)
    while abs(f_curr) > 0.01:
        print(1, x_curr, f_curr)
        unnormalized = nd.Gradient(func12)(x_curr)
        grad12 = unnormalized / np.sqrt(unnormalized[0]**2 + unnormalized[1]**2)
        print("grad", grad12)
        if f_curr > 0:
            x_curr -= step_size * grad12
        else:
            x_curr += step_size * grad12
        f_curr = func12(x_curr)

    
    f_curr = func23(x_curr)
    while abs(f_curr) > 0.1:
        print(2, f_curr)
        unnormalized = nd.Gradient(func23)(x_curr)
        grad23 = unnormalized / np.sqrt(unnormalized[0]**2 + unnormalized[1]**2)
        if f_curr > 0:
            x_curr -= step_size * grad23
        else:
            x_curr += step_size * grad23
        f_curr = func23(x_curr)
    

    
    f_curr = func31(x_curr)
    while abs(f_curr) > 0.1:
        print(3, f_curr)
        unnormalized = nd.Gradient(func12)(x_curr)
        grad31 = unnormalized / np.sqrt(unnormalized[0]**2 + unnormalized[1]**2)
        grad31 = nd.Gradient(func31)(x_curr)
        if f_curr > 0:
            x_curr -= step_size * grad31
        else:
            x_curr += step_size * grad31
        f_curr = func31(x_curr)
    """
    src_est = x_curr

    print("Source location: " + str(src_loc))
    print("Source estimate: " + str(src_est))
