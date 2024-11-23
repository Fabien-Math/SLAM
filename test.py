for i in range(1, 10):
    for j in range(i+1, 10):
        for k in range(j+1, 10):
            for l in range(k+1, 10):
                if i%2 * j%2 * k%2 * l%2:
                    c_sum = [int(c) for c in str(i+j+k+l)]
                    if not(c_sum[0]%2 or c_sum[1]%2):
                        c_prod = [int(c) for c in str(i*j*k*l)]
                        if c_prod[0]%2 * c_prod[1]%2 * c_prod[2%len(c_prod)]%2 * c_prod[3%len(c_prod)]%2:
                            print(i*1000 + j*100 + k*10 + l)
                        
