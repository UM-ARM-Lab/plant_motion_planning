import numpy as np


data = np.array([
    1.6633,
    1.9379,
    1.4036,
    1.6390,
    1.2213,
    1.6241,
    1.4652,
    2.6371,
    2.3294,
    1.3690,
    1.8346,
    1.6630,
    1.5484,
    1.2118,
    1.6497,
    1.5909,
    2.8990,
    1.8915,
    1.3358,
    2.3324
])

mean = np.mean(data)

var = np.mean(np.power(data - mean, 2))

print("mean: ",  mean)
print("variance: ", var)


