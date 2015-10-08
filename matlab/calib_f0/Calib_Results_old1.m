% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 921.864777171640299 ; 919.671913797009438 ];

%-- Principal point:
cc = [ 630.806211836263856 ; 353.579786536638949 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.106632956555959 ; -0.199727149624002 ; -0.000246769868870 ; -0.001953129523804 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.305695978092467 ; 2.329803280113678 ];

%-- Principal point uncertainty:
cc_error = [ 4.424712472019336 ; 3.419726127032182 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008689739929216 ; 0.026379457327496 ; 0.001443546548238 ; 0.002031746616130 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 21;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.140153e+00 ; 2.139455e+00 ; -2.012987e-01 ];
Tc_1  = [ -5.312929e+01 ; -6.870563e+01 ; 3.915549e+02 ];
omc_error_1 = [ 4.768022e-03 ; 4.661077e-03 ; 1.006615e-02 ];
Tc_error_1  = [ 1.888822e+00 ; 1.444216e+00 ; 1.260455e+00 ];

%-- Image #2:
omc_2 = [ 1.937745e+00 ; 1.218630e+00 ; -4.340949e-01 ];
Tc_2  = [ -9.587952e+01 ; -7.275070e+01 ; 3.006564e+02 ];
omc_error_2 = [ 3.448777e-03 ; 3.931525e-03 ; 5.375331e-03 ];
Tc_error_2  = [ 1.465811e+00 ; 1.115649e+00 ; 9.597334e-01 ];

%-- Image #3:
omc_3 = [ -1.760341e+00 ; -1.520452e+00 ; 1.589271e+00 ];
Tc_3  = [ 2.156636e+01 ; 6.965587e-01 ; 4.603756e+02 ];
omc_error_3 = [ 5.399117e-03 ; 3.148499e-03 ; 5.191224e-03 ];
Tc_error_3  = [ 2.217952e+00 ; 1.697484e+00 ; 8.607283e-01 ];

%-- Image #4:
omc_4 = [ 2.156460e+00 ; 1.193869e+00 ; -4.864316e-01 ];
Tc_4  = [ -6.478001e+01 ; -5.808056e+01 ; 3.869650e+02 ];
omc_error_4 = [ 3.778679e-03 ; 3.628743e-03 ; 6.277456e-03 ];
Tc_error_4  = [ 1.874882e+00 ; 1.423092e+00 ; 1.127679e+00 ];

%-- Image #5:
omc_5 = [ 1.880672e+00 ; 1.720963e+00 ; -8.346901e-01 ];
Tc_5  = [ -7.916244e+01 ; -3.141656e+01 ; 4.431962e+02 ];
omc_error_5 = [ 3.174512e-03 ; 4.230829e-03 ; 6.440079e-03 ];
Tc_error_5  = [ 2.125579e+00 ; 1.645224e+00 ; 1.126365e+00 ];

%-- Image #6:
omc_6 = [ NaN ; NaN ; NaN ];
Tc_6  = [ NaN ; NaN ; NaN ];
omc_error_6 = [ NaN ; NaN ; NaN ];
Tc_error_6  = [ NaN ; NaN ; NaN ];

%-- Image #7:
omc_7 = [ -2.201541e+00 ; -1.852445e+00 ; 6.292907e-01 ];
Tc_7  = [ -5.464068e+01 ; -3.022878e+01 ; 4.351202e+02 ];
omc_error_7 = [ 4.667324e-03 ; 3.910660e-03 ; 8.541622e-03 ];
Tc_error_7  = [ 2.086365e+00 ; 1.600313e+00 ; 1.072135e+00 ];

%-- Image #8:
omc_8 = [ 1.983100e+00 ; 1.831061e+00 ; -1.037079e+00 ];
Tc_8  = [ -1.437889e+02 ; -4.830888e+01 ; 3.837951e+02 ];
omc_error_8 = [ 2.410721e-03 ; 4.637817e-03 ; 6.703326e-03 ];
Tc_error_8  = [ 1.863894e+00 ; 1.469713e+00 ; 1.122643e+00 ];

%-- Image #9:
omc_9 = [ 2.354257e+00 ; 7.056068e-01 ; -4.227472e-01 ];
Tc_9  = [ -1.397961e+01 ; 1.672706e+01 ; 3.324061e+02 ];
omc_error_9 = [ 4.191621e-03 ; 2.673648e-03 ; 6.705436e-03 ];
Tc_error_9  = [ 1.596857e+00 ; 1.218065e+00 ; 9.097047e-01 ];

%-- Image #10:
omc_10 = [ 1.466616e+00 ; 1.699223e+00 ; -9.151713e-01 ];
Tc_10  = [ -8.620343e+01 ; -6.284110e+01 ; 3.886959e+02 ];
omc_error_10 = [ 2.933866e-03 ; 4.545068e-03 ; 5.190634e-03 ];
Tc_error_10  = [ 1.872201e+00 ; 1.450850e+00 ; 9.711524e-01 ];

%-- Image #11:
omc_11 = [ 7.914020e-01 ; 2.358025e+00 ; -1.241723e+00 ];
Tc_11  = [ 3.428940e+01 ; -1.016234e+02 ; 4.478762e+02 ];
omc_error_11 = [ 3.138156e-03 ; 5.086368e-03 ; 5.346665e-03 ];
Tc_error_11  = [ 2.174032e+00 ; 1.677983e+00 ; 1.038108e+00 ];

%-- Image #12:
omc_12 = [ -1.546659e+00 ; -2.065266e+00 ; 1.182486e+00 ];
Tc_12  = [ -2.351009e+01 ; -8.616825e+01 ; 4.759710e+02 ];
omc_error_12 = [ 4.907985e-03 ; 3.690791e-03 ; 5.816192e-03 ];
Tc_error_12  = [ 2.301088e+00 ; 1.769467e+00 ; 1.097374e+00 ];

%-- Image #13:
omc_13 = [ -1.970280e+00 ; -1.839734e+00 ; 1.164034e-01 ];
Tc_13  = [ -9.721664e+01 ; -5.998577e+01 ; 4.408539e+02 ];
omc_error_13 = [ 4.017424e-03 ; 4.371262e-03 ; 8.209135e-03 ];
Tc_error_13  = [ 2.124563e+00 ; 1.640202e+00 ; 1.319353e+00 ];

%-- Image #14:
omc_14 = [ 2.169493e+00 ; 2.154259e+00 ; -2.099190e-01 ];
Tc_14  = [ -9.821841e+01 ; -8.290282e+01 ; 3.268211e+02 ];
omc_error_14 = [ 3.664978e-03 ; 4.257796e-03 ; 8.167117e-03 ];
Tc_error_14  = [ 1.586104e+00 ; 1.214833e+00 ; 1.048700e+00 ];

%-- Image #15:
omc_15 = [ 1.709196e+00 ; 1.878298e+00 ; 3.261971e-01 ];
Tc_15  = [ -4.672896e+01 ; -7.439850e+01 ; 3.062590e+02 ];
omc_error_15 = [ 3.780480e-03 ; 3.751620e-03 ; 5.772717e-03 ];
Tc_error_15  = [ 1.485106e+00 ; 1.135356e+00 ; 1.025999e+00 ];

%-- Image #16:
omc_16 = [ 1.423319e+00 ; 1.595799e+00 ; -9.931172e-01 ];
Tc_16  = [ -7.782503e+01 ; -6.339814e+01 ; 3.759949e+02 ];
omc_error_16 = [ 3.181980e-03 ; 4.561824e-03 ; 4.902714e-03 ];
Tc_error_16  = [ 1.815636e+00 ; 1.401244e+00 ; 8.979584e-01 ];

%-- Image #17:
omc_17 = [ 8.003770e-01 ; 1.812680e+00 ; -1.536116e+00 ];
Tc_17  = [ -3.045218e+01 ; -4.742314e+00 ; 4.491138e+02 ];
omc_error_17 = [ 3.727555e-03 ; 4.946144e-03 ; 4.468956e-03 ];
Tc_error_17  = [ 2.156188e+00 ; 1.682165e+00 ; 8.739219e-01 ];

%-- Image #18:
omc_18 = [ 1.757155e+00 ; 2.374055e+00 ; 7.695260e-01 ];
Tc_18  = [ 2.365775e+01 ; -5.595542e+01 ; 2.623264e+02 ];
omc_error_18 = [ 4.958407e-03 ; 3.579452e-03 ; 6.687342e-03 ];
Tc_error_18  = [ 1.264452e+00 ; 9.777478e-01 ; 9.296512e-01 ];

%-- Image #19:
omc_19 = [ -1.787965e+00 ; -2.148009e+00 ; -6.302313e-01 ];
Tc_19  = [ -6.255701e+01 ; -4.805884e+01 ; 2.789812e+02 ];
omc_error_19 = [ 2.483054e-03 ; 4.572774e-03 ; 7.007654e-03 ];
Tc_error_19  = [ 1.346649e+00 ; 1.051446e+00 ; 9.950036e-01 ];

%-- Image #20:
omc_20 = [ -1.654567e+00 ; -1.944751e+00 ; 6.977414e-02 ];
Tc_20  = [ -1.506627e+02 ; -9.732393e+01 ; 3.990965e+02 ];
omc_error_20 = [ 3.546222e-03 ; 4.100130e-03 ; 6.826146e-03 ];
Tc_error_20  = [ 1.963892e+00 ; 1.520525e+00 ; 1.365941e+00 ];

%-- Image #21:
omc_21 = [ 1.805840e+00 ; 1.837719e+00 ; 1.818799e-01 ];
Tc_21  = [ 2.854920e+00 ; -7.112344e+01 ; 2.906081e+02 ];
omc_error_21 = [ 3.994230e-03 ; 3.602123e-03 ; 6.222573e-03 ];
Tc_error_21  = [ 1.404758e+00 ; 1.067100e+00 ; 9.239347e-01 ];

