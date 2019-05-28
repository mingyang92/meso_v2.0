import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

#PATH  = r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\result\outfile_lane_features_uniform.csv'

#df = pd.read_csv(PATH)
#print(df)

x = list(range(100))
y = np.random.randint(1,10,100)
y1 = np.random.randint(1,10,100)

fig1 = plt.figure('Figure1',figsize = (6,4)).add_subplot(122)
fig1.plot(x,y)
# 创建画布
fig2 = plt.figure('Figure2',figsize = (6,4)).add_subplot(122)
fig2.plot(x,y1)

# 创建标签
fig1.set_title('Figure1')
fig2.set_title('Figure2')
fig1.set_xlabel('This is x axis')
fig1.set_ylabel('This is y axis')

plt.show()



