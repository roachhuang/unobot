def primary(n):
    sum = 0
    for i in range(2, n):
        for j in range(2, n):
            if (i != j):
                if (i % j == 0):
                    break
                elif (j == n-1):
                    sum = sum + i
                    print(i)
                
primary(10)