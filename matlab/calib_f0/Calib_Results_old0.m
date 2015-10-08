% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 922.403095207614797 ; 920.205853597699615 ];

%-- Principal point:
cc = [ 630.900409704082904 ; 354.281927518635655 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.105108983123547 ; -0.198306929296617 ; 0.000134598787589 ; -0.001779352074282 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.225941433031558 ; 2.261939666040850 ];

%-- Principal point uncertainty:
cc_error = [ 4.473524877144240 ; 3.383144853911266 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008647881786616 ; 0.026738506508229 ; 0.001391109181630 ; 0.002020869738831 ; 0.000000000000000 ];

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
omc_1 = [ 2.140639e+00 ; 2.139811e+00 ; -2.005194e-01 ];
Tc_1  = [ -5.317626e+01 ; -6.900360e+01 ; 3.917068e+02 ];
omc_error_1 = [ 4.826862e-03 ; 4.728108e-03 ; 1.023616e-02 ];
Tc_error_1  = [ 1.909055e+00 ; 1.427898e+00 ; 1.251958e+00 ];

%-- Image #2:
omc_2 = [ 1.938397e+00 ; 1.218826e+00 ; -4.335260e-01 ];
Tc_2  = [ -9.591743e+01 ; -7.298178e+01 ; 3.007162e+02 ];
omc_error_2 = [ 3.401997e-03 ; 3.967734e-03 ; 5.413692e-03 ];
Tc_error_2  = [ 1.481244e+00 ; 1.102515e+00 ; 9.452239e-01 ];

%-- Image #3:
omc_3 = [ -1.760128e+00 ; -1.520893e+00 ; 1.588344e+00 ];
Tc_3  = [ 2.150455e+01 ; 3.371644e-01 ; 4.605202e+02 ];
omc_error_3 = [ 5.456523e-03 ; 3.159253e-03 ; 5.193432e-03 ];
Tc_error_3  = [ 2.241688e+00 ; 1.677273e+00 ; 8.399316e-01 ];

%-- Image #4:
omc_4 = [ 2.157204e+00 ; 1.194090e+00 ; -4.860282e-01 ];
Tc_4  = [ -6.482406e+01 ; -5.836901e+01 ; 3.871168e+02 ];
omc_error_4 = [ 3.737633e-03 ; 3.663946e-03 ; 6.359001e-03 ];
Tc_error_4  = [ 1.895370e+00 ; 1.408275e+00 ; 1.110892e+00 ];

%-- Image #5:
omc_5 = [ 1.881176e+00 ; 1.721390e+00 ; -8.341010e-01 ];
Tc_5  = [ -7.921256e+01 ; -3.175061e+01 ; 4.433573e+02 ];
omc_error_5 = [ 3.180458e-03 ; 4.262770e-03 ; 6.516373e-03 ];
Tc_error_5  = [ 2.148476e+00 ; 1.627661e+00 ; 1.107713e+00 ];

%-- Image #6:
omc_6 = [ 1.512179e+00 ; 1.582903e+00 ; -9.796085e-01 ];
Tc_6  = [ -9.618295e+01 ; -8.222847e+01 ; 3.487056e+02 ];
omc_error_6 = [ 3.097990e-03 ; 4.637605e-03 ; 5.043207e-03 ];
Tc_error_6  = [ 1.719330e+00 ; 1.297039e+00 ; 9.011555e-01 ];

%-- Image #7:
omc_7 = [ -2.201150e+00 ; -1.852381e+00 ; 6.287077e-01 ];
Tc_7  = [ -5.468150e+01 ; -3.055853e+01 ; 4.352906e+02 ];
omc_error_7 = [ 4.724059e-03 ; 3.978401e-03 ; 8.688973e-03 ];
Tc_error_7  = [ 2.109401e+00 ; 1.582300e+00 ; 1.050638e+00 ];

%-- Image #8:
omc_8 = [ 1.983303e+00 ; 1.831601e+00 ; -1.036334e+00 ];
Tc_8  = [ -1.438345e+02 ; -4.860882e+01 ; 3.838452e+02 ];
omc_error_8 = [ 2.411548e-03 ; 4.629177e-03 ; 6.733810e-03 ];
Tc_error_8  = [ 1.883186e+00 ; 1.451022e+00 ; 1.107287e+00 ];

%-- Image #9:
omc_9 = [ 2.355012e+00 ; 7.056114e-01 ; -4.226612e-01 ];
Tc_9  = [ -1.401716e+01 ; 1.648587e+01 ; 3.325798e+02 ];
omc_error_9 = [ 4.168661e-03 ; 2.704562e-03 ; 6.809287e-03 ];
Tc_error_9  = [ 1.614389e+00 ; 1.206802e+00 ; 8.960359e-01 ];

%-- Image #10:
omc_10 = [ 1.467093e+00 ; 1.699603e+00 ; -9.145418e-01 ];
Tc_10  = [ -8.625054e+01 ; -6.313570e+01 ; 3.887803e+02 ];
omc_error_10 = [ 2.921299e-03 ; 4.577142e-03 ; 5.218240e-03 ];
Tc_error_10  = [ 1.891851e+00 ; 1.434324e+00 ; 9.553668e-01 ];

%-- Image #11:
omc_11 = [ 7.918027e-01 ; 2.358514e+00 ; -1.240876e+00 ];
Tc_11  = [ 3.423361e+01 ; -1.019653e+02 ; 4.479794e+02 ];
omc_error_11 = [ 3.150397e-03 ; 5.112276e-03 ; 5.347789e-03 ];
Tc_error_11  = [ 2.197534e+00 ; 1.658179e+00 ; 1.015584e+00 ];

%-- Image #12:
omc_12 = [ -1.546411e+00 ; -2.065459e+00 ; 1.181563e+00 ];
Tc_12  = [ -2.355796e+01 ; -8.652817e+01 ; 4.760864e+02 ];
omc_error_12 = [ 4.961373e-03 ; 3.738345e-03 ; 5.825068e-03 ];
Tc_error_12  = [ 2.326302e+00 ; 1.750264e+00 ; 1.076276e+00 ];

%-- Image #13:
omc_13 = [ -1.969824e+00 ; -1.839584e+00 ; 1.155481e-01 ];
Tc_13  = [ -9.726467e+01 ; -6.032557e+01 ; 4.410148e+02 ];
omc_error_13 = [ 4.054701e-03 ; 4.436783e-03 ; 8.300083e-03 ];
Tc_error_13  = [ 2.147098e+00 ; 1.621454e+00 ; 1.301369e+00 ];

%-- Image #14:
omc_14 = [ 2.169769e+00 ; 2.154522e+00 ; -2.087466e-01 ];
Tc_14  = [ -9.825785e+01 ; -8.315803e+01 ; 3.268663e+02 ];
omc_error_14 = [ 3.695507e-03 ; 4.291770e-03 ; 8.224503e-03 ];
Tc_error_14  = [ 1.602522e+00 ; 1.199566e+00 ; 1.031595e+00 ];

%-- Image #15:
omc_15 = [ 1.709643e+00 ; 1.878254e+00 ; 3.271696e-01 ];
Tc_15  = [ -4.676361e+01 ; -7.463331e+01 ; 3.063488e+02 ];
omc_error_15 = [ 3.799501e-03 ; 3.807521e-03 ; 5.771375e-03 ];
Tc_error_15  = [ 1.501182e+00 ; 1.122243e+00 ; 1.018341e+00 ];

%-- Image #16:
omc_16 = [ 1.423842e+00 ; 1.596172e+00 ; -9.925445e-01 ];
Tc_16  = [ -7.787206e+01 ; -6.368047e+01 ; 3.760733e+02 ];
omc_error_16 = [ 3.164858e-03 ; 4.592709e-03 ; 4.930894e-03 ];
Tc_error_16  = [ 1.834736e+00 ; 1.385661e+00 ; 8.820591e-01 ];

%-- Image #17:
omc_17 = [ 8.008253e-01 ; 1.813387e+00 ; -1.535460e+00 ];
Tc_17  = [ -3.047471e+01 ; -5.090687e+00 ; 4.493261e+02 ];
omc_error_17 = [ 3.741087e-03 ; 4.965213e-03 ; 4.472022e-03 ];
Tc_error_17  = [ 2.179319e+00 ; 1.663862e+00 ; 8.529146e-01 ];

%-- Image #18:
omc_18 = [ 1.757401e+00 ; 2.373856e+00 ; 7.709018e-01 ];
Tc_18  = [ 2.362389e+01 ; -5.615400e+01 ; 2.624067e+02 ];
omc_error_18 = [ 5.004569e-03 ; 3.621971e-03 ; 6.617856e-03 ];
Tc_error_18  = [ 1.277738e+00 ; 9.678042e-01 ; 9.128994e-01 ];

%-- Image #19:
omc_19 = [ -1.787592e+00 ; -2.147549e+00 ; -6.312975e-01 ];
Tc_19  = [ -6.258597e+01 ; -4.827152e+01 ; 2.790735e+02 ];
omc_error_19 = [ 2.500125e-03 ; 4.607957e-03 ; 7.013265e-03 ];
Tc_error_19  = [ 1.361607e+00 ; 1.040315e+00 ; 9.799429e-01 ];

%-- Image #20:
omc_20 = [ -1.653980e+00 ; -1.944578e+00 ; 6.862326e-02 ];
Tc_20  = [ -1.507059e+02 ; -9.763656e+01 ; 3.991172e+02 ];
omc_error_20 = [ 3.537275e-03 ; 4.145075e-03 ; 6.807260e-03 ];
Tc_error_20  = [ 1.983423e+00 ; 1.502368e+00 ; 1.344517e+00 ];

%-- Image #21:
omc_21 = [ 1.806464e+00 ; 1.837716e+00 ; 1.828268e-01 ];
Tc_21  = [ 2.816312e+00 ; -7.134379e+01 ; 2.907033e+02 ];
omc_error_21 = [ 3.973159e-03 ; 3.644650e-03 ; 6.230155e-03 ];
Tc_error_21  = [ 1.419959e+00 ; 1.054627e+00 ; 9.153758e-01 ];

