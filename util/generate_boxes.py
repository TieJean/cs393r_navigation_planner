import random

f = open("boxes.txt", "a")

# f.write("Lower lab area\n")
y_lower = 3.0
y_upper = 8.0
x_lower = -27.7
x_upper = -20.0

# f.write("Lab area\n")
# y_lower = 10.0
# y_upper = 22.0
# x_lower = -22.3
# x_upper = -16.0

# f.write("Rooms along bottom of hallway\n")
# y_lower = 2.7
# y_upper = 7.0
# x_lower = -19.7
# x_upper = -5.6


BOX_SIZE = 0.5

NUM_BOXES = 25

for i in range(NUM_BOXES):
	x = float("{:.6f}".format(random.uniform(x_lower, x_upper)))
	y = float("{:.6f}".format(random.uniform(y_lower, y_upper)))

	# top horizontal
	f.write(str(x) + ",\t" + str(y) + ",\t" + str(x + BOX_SIZE) + ",\t" + str(y) + "\n")
	# bottom horizontal
	f.write(str(x) + ",\t" + str(y - BOX_SIZE) + ",\t" + str(x + BOX_SIZE) + ",\t" + str(y - BOX_SIZE) + "\n")
	# left vertical
	f.write(str(x) + ",\t" + str(y) + ",\t" + str(x) + ",\t" + str(y - BOX_SIZE) + "\n")
	# right vertical
	f.write(str(x + BOX_SIZE) + ",\t" + str(y )+ ",\t" + str(x + BOX_SIZE) + ",\t" + str(y - BOX_SIZE) + "\n")

f.close()
