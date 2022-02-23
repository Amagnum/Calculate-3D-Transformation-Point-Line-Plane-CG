# Calculate-3D-Transformation-Point-Line-Plane-CG
Calculate 3D Transformation | Point, Line, Plane | CG

# Available operations:
1. Rotation x, y, z (Rx, Ry, Rz)
2. Rotation along axis (RaA, RaA2)
3. Sheer x, y, z (Shx, Shy, Shz)
4. Scale (Scale)
5. Transform (Trans)
6. Projection on Plane (Proj)
7. Reflection (Refl)
8. Reflection on plane (ReflAP2, ReflAP)
9. Perspective CVV to Par CVV (PerCVVtoParCVV)
10. Npar
11. round off (Rnd)

# Run
```
g++ -I ./lib/eigen-3.3.9/Eigen 3dTransf.cpp -o e3ds
./e3ds < 3d.in
```

# Input format:
1. Enter No. of Points
2. Enter points in matrix format
  ```
  eg:
  1 (No of points)
  2 (x)
  0 (Y)
  0 (z)
  1 (w)

  2 (No of points)
  2 0 (x)
  0 2 (Y)
  0 0 (z)
  1 1 (w)

  // points go column wise
  ```
3. Round of to decimal places (eg: rnd 3)
4. No. of operations
5. Operation 1 
6. Operation 2
7. ...
```
Eg: 
3 (no. of operation)
Refl 0 0 1 1 (op 1)
Trans 0 -1 -1 0 (op 2)
Rz 0 (op 3)
```
---

Check 3d.in file for examples to use code

* Always remember +15 as ackw rot and -15 as ckw rotation

# Question solution (pakhira book):
// Success Q7.8
// Rotate ackw 30 in z axis
```
4
2 0 0 0
0 2 0 0
0 0 2 0
1 1 1 1
rnd 3
1
RaA 30 1 1 1
```

// Question 5.8
```
4
1 3 4 2
1 1 2 3
0 0 0 0
1 1 1 1
rnd 3
3
Refl 0 0 1 1
Trans 0 -1 -1 0
Rz 0 180
```

// Random Example
```
4
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1
rnd 3
1
Proj 5
plane 0 0 15
dir 1 1 1
```


// Success Q7.6 
```
4
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1
rnd 3
4
Trans -4 -7 -10
Rx 11.287
Ry 40.282
Trans 4 7 10
```

// Q7.5
```
8
0 2 2 0 0 2 2 0
0 0 2 2 0 0 2 0
0 0 0 0 2 2 2 2
1 1 1 1 1 1 1 1
rnd 3
3
Trans -2 0 0
RaA -45 -2 2 2
RaA2 -45 -0.577 0.577 0.577
Trans 2 0 0
```

// Success 5.6 C
```
4
-1 0 1 0
0 2 0 -2
0 0 0 0
1 1 1 1
rnd 3
3
Trans 0 -2 0
ReflAP2 0.707 -0.707 0
Trans 0 2 0
```
