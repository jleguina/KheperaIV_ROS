clear
clc

A = [1 -0.2094 1
    0.2094 1 0.3142
    0 0 0];

B = [1 0
    0 0
    0 1];

Q = diag([5.5 5.5 216.5]);
R =diag([1 1]);

[K,S,e] = lqr(A,B,Q,R)