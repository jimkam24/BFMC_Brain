file = open("coordinates.log", "r")

for data in file:
    number1 = data[1:18]
    number2 = data[19:22]
    print(float(number2))