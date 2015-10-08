% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 922.450514890763657 ; 920.411923111604096 ];

%-- Principal point:
cc = [ 635.394539023621064 ; 354.208457614857480 ];

%-- Skew coefficient:
alpha_c = 0.001473023301789;

%-- Distortion coefficients:
kc = [ 0.106200123029192 ; -0.204643068419538 ; -0.000000000000000 ; -0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.192341093613377 ; 2.229173244693199 ];

%-- Principal point uncertainty:
cc_error = [ 1.256045038036933 ; 1.935841188911105 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000982684587066;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008726279492995 ; 0.026689649863030 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 21;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 1;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 0 ; 0 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.140043e+00 ; 2.139477e+00 ; -1.965050e-01 ];
Tc_1  = [ -5.504984e+01 ; -6.900937e+01 ; 3.916840e+02 ];
omc_error_1 = [ 4.182017e-03 ; 4.019959e-03 ; 8.367750e-03 ];
Tc_error_1  = [ 5.239337e-01 ; 8.068162e-01 ; 1.192956e+00 ];

%-- Image #2:
omc_2 = [ 1.938397e+00 ; 1.216166e+00 ; -4.296236e-01 ];
Tc_2  = [ -9.732184e+01 ; -7.294592e+01 ; 3.004505e+02 ];
omc_error_2 = [ 1.930227e-03 ; 1.680029e-03 ; 2.108508e-03 ];
Tc_error_2  = [ 3.864212e-01 ; 6.120820e-01 ; 7.694228e-01 ];

%-- Image #3:
omc_3 = [ -1.764474e+00 ; -1.524162e+00 ; 1.586663e+00 ];
Tc_3  = [ 1.925633e+01 ; 3.645581e-01 ; 4.604182e+02 ];
omc_error_3 = [ 2.194837e-03 ; 2.143307e-03 ; 3.258141e-03 ];
Tc_error_3  = [ 6.450700e-01 ; 9.489142e-01 ; 8.332109e-01 ];

%-- Image #4:
omc_4 = [ 2.157185e+00 ; 1.192033e+00 ; -4.819050e-01 ];
Tc_4  = [ -6.663938e+01 ; -5.833585e+01 ; 3.869960e+02 ];
omc_error_4 = [ 2.310928e-03 ; 1.953361e-03 ; 3.146888e-03 ];
Tc_error_4  = [ 5.133403e-01 ; 8.025193e-01 ; 1.041030e+00 ];

%-- Image #5:
omc_5 = [ 1.881189e+00 ; 1.719191e+00 ; -8.306771e-01 ];
Tc_5  = [ -8.131193e+01 ; -3.171137e+01 ; 4.430791e+02 ];
omc_error_5 = [ 2.429085e-03 ; 2.520681e-03 ; 3.886513e-03 ];
Tc_error_5  = [ 5.851695e-01 ; 9.326447e-01 ; 1.022748e+00 ];

%-- Image #6:
omc_6 = [ 1.513346e+00 ; 1.579723e+00 ; -9.761083e-01 ];
Tc_6  = [ -9.779261e+01 ; -8.215322e+01 ; 3.483281e+02 ];
omc_error_6 = [ 1.812628e-03 ; 2.000792e-03 ; 2.087298e-03 ];
Tc_error_6  = [ 4.358569e-01 ; 7.136669e-01 ; 6.960699e-01 ];

%-- Image #7:
omc_7 = [ -2.204324e+00 ; -1.856358e+00 ; 6.281800e-01 ];
Tc_7  = [ -5.671983e+01 ; -3.055380e+01 ; 4.351325e+02 ];
omc_error_7 = [ 3.262688e-03 ; 3.489800e-03 ; 7.400883e-03 ];
Tc_error_7  = [ 5.959877e-01 ; 9.006528e-01 ; 1.003684e+00 ];

%-- Image #8:
omc_8 = [ 1.982722e+00 ; 1.828878e+00 ; -1.032425e+00 ];
Tc_8  = [ -1.455820e+02 ; -4.852860e+01 ; 3.831369e+02 ];
omc_error_8 = [ 1.909183e-03 ; 2.166087e-03 ; 2.951144e-03 ];
Tc_error_8  = [ 4.897358e-01 ; 7.819072e-01 ; 8.097274e-01 ];

%-- Image #9:
omc_9 = [ 2.355471e+00 ; 7.040627e-01 ; -4.180751e-01 ];
Tc_9  = [ -1.563681e+01 ; 1.651481e+01 ; 3.326813e+02 ];
omc_error_9 = [ 2.683314e-03 ; 1.446573e-03 ; 3.493960e-03 ];
Tc_error_9  = [ 4.805653e-01 ; 7.169525e-01 ; 8.945940e-01 ];

%-- Image #10:
omc_10 = [ 1.467961e+00 ; 1.696582e+00 ; -9.111027e-01 ];
Tc_10  = [ -8.806837e+01 ; -6.309370e+01 ; 3.885021e+02 ];
omc_error_10 = [ 1.855710e-03 ; 2.104803e-03 ; 2.532296e-03 ];
Tc_error_10  = [ 4.994792e-01 ; 8.054554e-01 ; 8.152217e-01 ];

%-- Image #11:
omc_11 = [ 7.936600e-01 ; 2.355287e+00 ; -1.238159e+00 ];
Tc_11  = [ 3.217753e+01 ; -1.019348e+02 ; 4.482223e+02 ];
omc_error_11 = [ 1.763860e-03 ; 2.321469e-03 ; 2.908207e-03 ];
Tc_error_11  = [ 5.891394e-01 ; 9.181918e-01 ; 9.131136e-01 ];

%-- Image #12:
omc_12 = [ -1.549817e+00 ; -2.068846e+00 ; 1.180894e+00 ];
Tc_12  = [ -2.573989e+01 ; -8.643725e+01 ; 4.758636e+02 ];
omc_error_12 = [ 2.145948e-03 ; 2.370580e-03 ; 3.918623e-03 ];
Tc_error_12  = [ 6.395873e-01 ; 9.964168e-01 ; 9.985535e-01 ];

%-- Image #13:
omc_13 = [ -1.972794e+00 ; -1.844896e+00 ; 1.119378e-01 ];
Tc_13  = [ -9.936851e+01 ; -6.034986e+01 ; 4.407979e+02 ];
omc_error_13 = [ 3.159102e-03 ; 3.707736e-03 ; 6.297166e-03 ];
Tc_error_13  = [ 5.837509e-01 ; 9.224834e-01 ; 1.180625e+00 ];

%-- Image #14:
omc_14 = [ 2.167888e+00 ; 2.153025e+00 ; -2.066548e-01 ];
Tc_14  = [ -9.980143e+01 ; -8.315779e+01 ; 3.266891e+02 ];
omc_error_14 = [ 2.890917e-03 ; 3.060132e-03 ; 6.007167e-03 ];
Tc_error_14  = [ 4.220074e-01 ; 6.628505e-01 ; 8.769548e-01 ];

%-- Image #15:
omc_15 = [ 1.707637e+00 ; 1.876606e+00 ; 3.324127e-01 ];
Tc_15  = [ -4.818863e+01 ; -7.462792e+01 ; 3.062273e+02 ];
omc_error_15 = [ 2.238528e-03 ; 2.241955e-03 ; 3.679577e-03 ];
Tc_error_15  = [ 4.236127e-01 ; 6.338613e-01 ; 9.507083e-01 ];

%-- Image #16:
omc_16 = [ 1.425069e+00 ; 1.592968e+00 ; -9.890161e-01 ];
Tc_16  = [ -7.962515e+01 ; -6.363547e+01 ; 3.758052e+02 ];
omc_error_16 = [ 1.862745e-03 ; 2.071940e-03 ; 2.241903e-03 ];
Tc_error_16  = [ 4.816391e-01 ; 7.801984e-01 ; 7.557919e-01 ];

%-- Image #17:
omc_17 = [ 8.036646e-01 ; 1.809907e+00 ; -1.532346e+00 ];
Tc_17  = [ -3.267706e+01 ; -5.055777e+00 ; 4.491810e+02 ];
omc_error_17 = [ 1.891292e-03 ; 2.376925e-03 ; 2.263452e-03 ];
Tc_error_17  = [ 6.001813e-01 ; 9.512896e-01 ; 8.284664e-01 ];

%-- Image #18:
omc_18 = [ 1.752404e+00 ; 2.371227e+00 ; 7.763317e-01 ];
Tc_18  = [ 2.237172e+01 ; -5.611312e+01 ; 2.621898e+02 ];
omc_error_18 = [ 2.339399e-03 ; 1.906823e-03 ; 3.840546e-03 ];
Tc_error_18  = [ 3.777162e-01 ; 5.666645e-01 ; 8.619713e-01 ];

%-- Image #19:
omc_19 = [ -1.787231e+00 ; -2.151630e+00 ; -6.380164e-01 ];
Tc_19  = [ -6.386746e+01 ; -4.825766e+01 ; 2.785917e+02 ];
omc_error_19 = [ 2.102105e-03 ; 2.787958e-03 ; 4.281210e-03 ];
Tc_error_19  = [ 3.892101e-01 ; 6.026691e-01 ; 9.079093e-01 ];

%-- Image #20:
omc_20 = [ -1.656216e+00 ; -1.949753e+00 ; 6.572037e-02 ];
Tc_20  = [ -1.526313e+02 ; -9.764783e+01 ; 3.988108e+02 ];
omc_error_20 = [ 2.069492e-03 ; 2.651278e-03 ; 4.071593e-03 ];
Tc_error_20  = [ 5.264993e-01 ; 8.432743e-01 ; 1.046274e+00 ];

%-- Image #21:
omc_21 = [ 1.804748e+00 ; 1.835801e+00 ; 1.884094e-01 ];
Tc_21  = [ 1.470948e+00 ; -7.134532e+01 ; 2.907228e+02 ];
omc_error_21 = [ 1.994120e-03 ; 1.776580e-03 ; 3.195424e-03 ];
Tc_error_21  = [ 4.035028e-01 ; 5.918857e-01 ; 8.703629e-01 ];

