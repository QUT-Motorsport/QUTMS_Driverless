import numpy as np

test = np.array([1, 2, 4, 7, 8, 4, 56, 567])
print(np.split(test, [0, 3, 6]))

import plotly.express.colors as pe_colors

print(pe_colors.sequential.Rainbow)
