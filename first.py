import os
a =46
b=12
def gcd(a, b):
    if b == 0:
     return a
    else:
        return gcd(b, a%b)

print(gcd(a,b))