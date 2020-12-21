#!/usr/bin/python3
# coding:utf-8

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei']  #用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  #用来正常显示负号

data100 = np.loadtxt('./../../data/relocalization_distance_100.txt')
data80 = np.loadtxt('./../../data/relocalization_distance_80.txt')
data60 = np.loadtxt('./../../data/relocalization_distance_60.txt')

data100.sort()
data80.sort()
data60.sort()

delt = 1.0 / len(data100)
proba = np.arange(0, 1.0, delt)
plt.plot(data100, proba, label=u"0.59个/m")

delt = 1.0 / len(data80)
proba = np.arange(0, 1.0, delt)
plt.plot(data80, proba, label=u"0.473个/m")

delt = 1.0 / len(data60)
proba = np.arange(0, 1.0, delt)
plt.plot(data60, proba, label=u"0.354个/m")


plt.ylabel(u'重定位成功概率')  # 横坐标轴的标题
plt.xlabel(u'无人车行驶距离/m')  # 纵坐标轴的标题
# plt.xticks(np.arange(0, 60, 5))  # 设置横坐标轴的刻度为 0 到 10 的数组
# plt.xlim([0, 60])  # 设置纵坐标轴范围为 -2 到 2
plt.ylim([0, 1.1])  # 设置纵坐标轴范围为 -2 到 2

plt.legend(loc='center right')
plt.title(u'不同语义聚类密度情况下无人车行驶一定距离重定位成功的概率')  # 图形的标题
plt.show()
