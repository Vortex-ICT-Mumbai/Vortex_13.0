# Codathon 13.0 - Round 1  
## Hall of Solutions

> A curated collection of solutions selected for clarity, fundamentals,  
> creative framing, and elegance - not just correctness.

## Q1 - Fundamental Operations & Assumptions

**Ideal Solutions**
```python 
num = input("Enter number in R2 which you want to take square root of")
root = # (0, num]
for _ in range(20):
    root = (root + num / root) / 2
# This converges due to the fact that it provides upper and lower bound to roots which goes on converging as interations progress
```

### Solution A

**Author:** Varad G\
**Affiliation:** ICT Mumbai - TYBChemE\
**Language:** Python

**Why this stood out:**  

- Minimal but sufficient logic  
- Transparent reasoning

```python
x1 = int(input("Enter the number to take square root of: "))
def f(x):
    return x*x - x1

x=0.0001

for _ in range(0,10000):
    x = x-(f(x)/(2*x))
```

### Solution A

**Author:** Dhavall Raja\
**Affiliation:** ICT Mumbai - TYBChemE\
**Language:** Python

**Why this stood out:**  

- Well defined and Robust
- Well documented and compensations for Error

```python
def sqrt_function(x, guess_range, tol = 0.000001):
    # finding a value of sqrt(x) is as good as solving y2 - x = 0. for a value of y. This we can solve using secant method.
    # Thus we can start by defining a function for this equation.
    f = lambda y: y*y - x
    ## Now based on the given range we run the secant algorithm to find the value of y
    y1, y2 = guess_range
    f_y1, f_y2 = f(y1), f(y2)
    if f_y1 >= 0 or f_y2 <= 0:
        raise ValueError("Given range is not valid")
    error = y2 - y1
    while error > tol: # In case the error betwween the points goes lower than or equal to tolerance we stop
        c =  (f_y1*y2 - f_y2*y1)/(f_y1 - f_y2) # Formula comes from Secant Algorithm.
        f_c = f(c) # Here we evaluate the function value only once in entire loop to save space.
        if f_c > 0:
            y2 = c
            f_y2 = f_c
        elif f_c < 0:
            y1 = c
            f_y1 = f_c
        else:
            break # If by any chance we get 0 we stop immediately
        error = y2 - y1
    return (y1 + y2)/2
```

### Solution B

**Author:** Varad G\
**Affiliation:** ICT Mumbai - TYBChemE\
**Language:** Python

**Why this stood out:**  

- Within Constraints
- Built upon the previous root finding problem efficiently

```python
x1 = int(input("Enter the number to take absolute of: "))

x1 = x1 * x1

x=0.0001

for _ in range(0,10000):
    x = x-(f(x)/(2*x))
```

## Question 2

### Solution A

**Author:** Satyamesh Nathu Mali\
**Affiliation:** ICT Mumbai - FYBChemE\
**Language:** Python

**Why this stood out:**  

- Simple Logic NO assumptions
- Clean, sufficiently documentated and Understandable

```python
# integration is about adding small areas of the given function
DividingFactor = 10000 # the more the better area since it makes the deltax very less so better area comes

def CalculateDefIntegral(upperlimit,lowerlimit,func):
    Area = 0
    DeltaX = (upperlimit-lowerlimit)/DividingFactor # the more smaller the more better
    x = lowerlimit
    while(x<=upperlimit):
        y = func(x)
        SmallArea = y * DeltaX  
        Area += SmallArea
        x += DeltaX
    
    return Area
```


## Question 4

**Author:** Dhavall Raja\
**Affiliation:** ICT Mumbai - TYBChemE\
**Language:** Python

**Why this stood out:**  

- Sleek and Minimal
- Best use of resources

```python
string = # the string of numbers
n = len(string)
number_list = []
Joke = ""
for l in range(0, n/8):
    bin_number = string[l*8:(l+1)*8] # Convert to binary
    decimal = int(bin_number, 2) # To decimal
    letter = chr(decimal) # To characters
    Joke += letter
print('The joke is, \n\n' + Joke)
```

## Question 6

**Author:** Karthik Mahadevan\
**Affiliation:** ICT Mumbai - FinYBChemE\
**Language:** Python

**Why this stood out:**  

- Simple, Assumptions stated

```python
# I am assuming time step as 1
def simulate_system(init,final_time,r):
    curr = init
    vals=[init]
    time_arr = list(range(0,final_time+1))

    for element in time_arr:
        if element !=0:
            curr = r*curr*(1-curr)
            vals.append(curr)
    return time_arr,vals

import matplotlib.pyplot as plt
t,v = simulate_system(0.4,100,3.245) # r = 3.245 is the answer

plt.plot(t,v)
plt.grid();
```


## Question 6

**Author:** Satyamesh Nathu Mali\
**Affiliation:** ICT Mumbai - FYBChemE\
**Language:** Python

**Why this stood out:**  

- Superb and robust implementation
- Well documentated

```python
# note volume issueno serial number and issue year are suppose to be numbers not string
def GenerateDOI(JournamlName,Volume,IssueNo,AuthorFirstName,AuthorLastName,SerialNo,IssueYear):
    # JournamlName = "kdjf"
    # count number of space in the given name
    # make sure you don't put initial spaces in the journal name

    # well split the array as per the space present in the name and then later slice it as per our requirement
    Words = JournamlName.split(" ")
    JCode = ""

    # getting the jcode as per the given number of words in the full name
    if(len(Words)==1):
        JCode = Words[0][0:4]
    elif(len(Words)==2):
        JCode = Words[0][0:2]+Words[1][0:2]
    else:
        JCode = Words[0][0:1] + Words[1][0:1] + Words[2][0:1]+ Words[3][0:1]

    def GetTwoDigit(num):
        SerialNoString = str(num)
        if(SerialNo>9):
            SerialNoString = "0" + str(SerialNo)

        return SerialNoString

    SerialNoString = GetTwoDigit(SerialNo)
    IssueNoString = GetTwoDigit(IssueNo)
    VolumeNoString = GetTwoDigit(Volume)
    if(SerialNo>9):
        SerialNoString = "0" + str(SerialNo)


    # finally adding all the stuffs
    # their was no mention what is 10.5678
    DOI = "https://doi.org/10.5678/"+str(IssueYear)+"."+JCode+"."+VolumeNoString+"."+IssueNoString+"."+AuthorFirstName[0:2]+AuthorLastName[0:2]+SerialNoString

    return DOI
    

# sample 
print(GenerateDOI("Ultimate Detective",1,2,"satyam","mali",10,2100))
```

