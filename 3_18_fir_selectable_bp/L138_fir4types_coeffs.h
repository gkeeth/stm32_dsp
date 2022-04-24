#define N 81

float hbp[N] = {
    -5.8657E-017,-1.4811E-002,-5.6036E-003, 1.1286E-002, 8.4132E-003,-5.3567E-003,
    -6.3332E-003, 7.0426E-004,-4.6855E-018,-7.5341E-004, 7.2490E-003, 6.5616E-003,
    -1.1033E-002,-1.5854E-002, 8.4366E-003, 2.3919E-002,-3.3369E-017,-2.6095E-002,
    -1.0044E-002, 2.0609E-002, 1.5675E-002,-1.0201E-002,-1.2351E-002, 1.4097E-003,
    -4.8256E-018,-1.6016E-003, 1.5958E-002, 1.5019E-002,-2.6381E-002,-3.9828E-002,
     2.2423E-002, 6.7835E-002,-3.4026E-017,-8.7322E-002,-3.7462E-002, 8.7941E-002,
     7.9529E-002,-6.5477E-002,-1.1252E-001, 2.4229E-002, 1.2500E-001, 2.4229E-002,
    -1.1252E-001,-6.5477E-002, 7.9529E-002, 8.7941E-002,-3.7462E-002,-8.7322E-002,
    -3.4026E-017, 6.7835E-002, 2.2423E-002,-3.9828E-002,-2.6381E-002, 1.5019E-002,
     1.5958E-002,-1.6016E-003,-4.8256E-018, 1.4097E-003,-1.2351E-002,-1.0201E-002,
     1.5675E-002, 2.0609E-002,-1.0044E-002,-2.6095E-002,-3.3369E-017, 2.3919E-002,
     8.4366E-003,-1.5854E-002,-1.1033E-002, 6.5616E-003, 7.2490E-003,-7.5341E-004,
    -4.6855E-018, 7.0426E-004,-6.3332E-003,-5.3567E-003, 8.4132E-003, 1.1286E-002,
    -5.6036E-003,-1.4811E-002,-5.8657E-017
};

float hbs[N] = {
    -6.1030E-005,-6.1030E-005, 2.1360E-004, 8.5441E-004, 1.5868E-003, 1.8614E-003,
     1.1596E-003,-5.4927E-004,-2.6243E-003,-3.9059E-003,-3.5702E-003,-1.8004E-003,
     1.2206E-004, 7.0184E-004,-7.3235E-004,-2.8684E-003,-3.2346E-003, 2.4412E-004,
     7.2625E-003, 1.4495E-002, 1.7149E-002, 1.2267E-002, 1.1596E-003,-1.0528E-002,
    -1.6295E-002,-1.3274E-002,-5.0349E-003, 0.0000E+000,-5.4011E-003,-2.1116E-002,
    -3.7136E-002,-3.8205E-002,-1.3518E-002, 3.4085E-002, 8.5777E-002, 1.1437E-001,
     9.8715E-002, 3.7014E-002,-4.9159E-002,-1.2352E-001, 8.4712E-001,-1.2352E-001,
    -4.9159E-002, 3.7014E-002, 9.8715E-002, 1.1437E-001, 8.5777E-002, 3.4085E-002,
    -1.3518E-002,-3.8205E-002,-3.7136E-002,-2.1116E-002,-5.4011E-003, 0.0000E+000,
    -5.0349E-003,-1.3274E-002,-1.6295E-002,-1.0528E-002, 1.1596E-003, 1.2267E-002,
     1.7149E-002, 1.4495E-002, 7.2625E-003, 2.4412E-004,-3.2346E-003,-2.8684E-003,
    -7.3235E-004, 7.0184E-004, 1.2206E-004,-1.8004E-003,-3.5702E-003,-3.9059E-003,
    -2.6243E-003,-5.4927E-004, 1.1596E-003, 1.8614E-003, 1.5868E-003, 8.5441E-004,
     2.1360E-004,-6.1030E-005,-6.1030E-005
};

float hhp[N] = {
     3.6673E-017, 7.6035E-003,-2.4488E-003,-7.2729E-003, 4.9451E-003, 6.1359E-003,
    -7.2460E-003,-4.2003E-003, 9.0969E-003, 1.5483E-003,-1.0252E-002, 1.6628E-003,
     1.0493E-002,-5.2054E-003,-9.6525E-003, 8.7912E-003, 7.6265E-003,-1.2085E-002,
    -4.3894E-003, 1.4722E-002,-1.1520E-016,-1.6321E-002, 5.3977E-003, 1.6501E-002,
    -1.1580E-002,-1.4878E-002, 1.8258E-002, 1.1045E-002,-2.5090E-002,-4.5061E-003,
     3.1710E-002,-5.5158E-003,-3.7750E-002, 2.0606E-002, 4.2861E-002,-4.4973E-002,
    -4.6746E-002, 9.4506E-002, 4.9174E-002,-3.1438E-001, 4.5000E-001,-3.1438E-001,
     4.9174E-002, 9.4506E-002,-4.6746E-002,-4.4973E-002, 4.2861E-002, 2.0606E-002,
    -3.7750E-002,-5.5158E-003, 3.1710E-002,-4.5061E-003,-2.5090E-002, 1.1045E-002,
     1.8258E-002,-1.4878E-002,-1.1580E-002, 1.6501E-002, 5.3977E-003,-1.6321E-002,
    -1.1520E-016, 1.4722E-002,-4.3894E-003,-1.2085E-002, 7.6265E-003, 8.7912E-003,
    -9.6525E-003,-5.2054E-003, 1.0493E-002, 1.6628E-003,-1.0252E-002, 1.5483E-003,
     9.0969E-003,-4.2003E-003,-7.2460E-003, 6.1359E-003, 4.9451E-003,-7.2729E-003,
    -2.4488E-003, 7.6035E-003, 3.6673E-017
};

float hlp[N] = {
     4.0329E-017, 7.1123E-003, 5.6036E-003,-3.1237E-003,-8.4132E-003,-3.3207E-003,
     6.3332E-003, 8.5477E-003,-1.4057E-017,-9.1442E-003,-7.2490E-003, 4.0676E-003,
     1.1033E-002, 4.3878E-003,-8.4366E-003,-1.1486E-002, 1.4301E-017, 1.2531E-002,
     1.0044E-002,-5.7041E-003,-1.5675E-002,-6.3238E-003, 1.2351E-002, 1.7110E-002,
    -1.4477E-017,-1.9439E-002,-1.5958E-002, 9.3102E-003, 2.6381E-002, 1.1023E-002,
    -2.2423E-002,-3.2575E-002, 1.4583E-017, 4.1933E-002, 3.7462E-002,-2.4339E-002,
    -7.9529E-002,-4.0590E-002, 1.1252E-001, 2.9407E-001, 3.7500E-001, 2.9407E-001,
     1.1252E-001,-4.0590E-002,-7.9529E-002,-2.4339E-002, 3.7462E-002, 4.1933E-002,
     1.4583E-017,-3.2575E-002,-2.2423E-002, 1.1023E-002, 2.6381E-002, 9.3102E-003,
    -1.5958E-002,-1.9439E-002,-1.4477E-017, 1.7110E-002, 1.2351E-002,-6.3238E-003,
    -1.5675E-002,-5.7041E-003, 1.0044E-002, 1.2531E-002, 1.4301E-017,-1.1486E-002,
    -8.4366E-003, 4.3878E-003, 1.1033E-002, 4.0676E-003,-7.2490E-003,-9.1442E-003,
    -1.4057E-017, 8.5477E-003, 6.3332E-003,-3.3207E-003,-8.4132E-003,-3.1237E-003,
     5.6036E-003, 7.1123E-003, 4.0329E-017
};
