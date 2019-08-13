import math

def calculate_steady_sate_throttle(speed):
    return -0.071175 + 6.42851e-18 * math.sqrt(1.62827e33 * speed + 7.24232e32)