#!/usr/bin/python3
# coding:utf-8

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei']  #用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  #用来正常显示负号

data = np.loadtxt('./../../data/relocalization_distance_100.txt')

data.sort()

delt = 1.0 / len(data)
proba = np.arange(0, 1.0, delt)

p99 = [0.99] * int(len(data) * 0.99)
p95 = [0.95] * int(len(data) * 0.95)
p90 = [0.90] * int(len(data) * 0.90)

plt.plot(data, proba, label=u"概率")
p99x = data[int(len(data) * 0.99)]
p95x = data[int(len(data) * 0.95)]
p90x = data[int(len(data) * 0.90)]

plt.hlines(0.99, 0, p99x, colors="r", linestyles="dashed", label="P99")
plt.vlines(p99x, 0, 0.99, colors="r", linestyles="dashed")

plt.hlines(0.95, 0, p95x, colors="g", linestyles="dashed", label="P95")
plt.vlines(p95x, 0, 0.95, colors="g", linestyles="dashed")

plt.hlines(0.90, 0, p90x, colors="b", linestyles="dashed", label="P90")
plt.vlines(p90x, 0, 0.90, colors="b", linestyles="dashed")

plt.ylabel(u'重定位成功概率')  # 横坐标轴的标题
plt.xlabel(u'无人车行驶距离/m')  # 纵坐标轴的标题
plt.xticks(np.arange(0, 60, 5))  # 设置横坐标轴的刻度为 0 到 10 的数组
plt.xlim([0, 60])  # 设置纵坐标轴范围为 -2 到 2
plt.ylim([0, 1.1])  # 设置纵坐标轴范围为 -2 到 2
plt.legend(loc='center right')
# plt.grid() # 显示网格
plt.title(u'无人车行驶一定距离重定位成功的概率')  # 图形的标题
plt.show()
