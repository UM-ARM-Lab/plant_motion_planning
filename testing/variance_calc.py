import numpy as np


data = np.array([
    2.4806,
    1.8429,
    2.2920,
    2.3629,
    2.9596,
    1.9684,
    2.6094,
    2.2775,
    1.8997,
    1.8656,
    1.8821,
    1.5645,
    2.8373,
    2.0016,
    1.7445,
    2.3760,
    2.5040,
    2.0846,
    2.7660,
    1.9373
])

mean = np.mean(data)

var = np.mean(np.power(data - mean, 2))

print("mean: ",  mean)
print("variance: ", var)


