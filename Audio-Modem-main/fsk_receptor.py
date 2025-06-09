import numpy as np
import sounddevice as sd
from scipy.fft import fft

sample_rate = 48000
duration = 0.1  # in seconds
chunk_size = int(sample_rate * duration)
freq_0 = 1800
freq_1 = 2000
tolerance = 80
header = '10101010'

bit_buffer = ''
received_bits = ''
receiving = False

def detect_frequency(signal):
    n = len(signal)
    fft_vals = fft(signal)
    freqs = np.fft.fftfreq(n, 1 / sample_rate)
    peak_freq = abs(freqs[np.argmax(np.abs(fft_vals))])
    return peak_freq

def frequency_to_bit(freq):
    if abs(freq - freq_0) < tolerance:
        return '0'
    elif abs(freq - freq_1) < tolerance:
        return '1'
    return None

def callback(indata, frames, time, status):
    global bit_buffer, receiving, received_bits

    # Flatten and process the input chunk
    chunk = indata[:, 0]
    freq = detect_frequency(chunk)
    bit = frequency_to_bit(freq)

    if bit:
        bit_buffer += bit

        if not receiving and bit_buffer.endswith(header):
            print("[+] Header detected")
            receiving = True
            received_bits = ''
        elif receiving:
            received_bits += bit

            # Once we collect a full byte, print character
            if len(received_bits) % 8 == 0:
                byte = received_bits[-8:]
                try:
                    char = chr(int(byte, 2))
                    print(char, end='', flush=True)
                except:
                    pass  # skip invalid byte

def listen():
    print("Listening...")
    with sd.InputStream(callback=callback, samplerate=sample_rate, channels=1, blocksize=chunk_size):
        input()  # Press Enter to stop

if __name__ == "__main__":
    listen()