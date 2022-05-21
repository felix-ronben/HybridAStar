def yield_test():
    for i in range(10):
        for j in range(2):
            yield [i, j, i+j]


for k in yield_test():
    print(k)
