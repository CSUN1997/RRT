import numpy as np

def sample():
    x = round(np.random.rand() * 10, 2)
    y = round(np.random.rand() * 10, 2)
    return [x, y]

if __name__ == '__main__':
    print(sample())