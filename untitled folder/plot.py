import matplotlib.pyplot as plt

f = open ('path2.txt', 'r')
pos = []
vel = []
acc = []

for line in f:
    splitted = line.split()
    pos.append(float(splitted[2]))
    vel.append(float(splitted[5]))
    acc.append(float(splitted[8]))

plt.axis([0, 500, -1.5, 1.5])
plt.plot(pos)
plt.plot(vel)
plt.plot(acc)
plt.show()
