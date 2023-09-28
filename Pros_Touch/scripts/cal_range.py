import numpy as np
import matplotlib.pyplot as plt
k = 10.0
b = 30000.0
data_type = 'float32'

data = np.linspace(-3000, 3000, 100000).astype(data_type)
data_int = (data*k+b).astype(np.uint16)
data_recovery = (data_int-b)/k

plt.plot(data, data_recovery,linewidth=5)
plt.plot(data, data)
plt.show()