//--------------------------------------------------//
// main program
//
// author Claire Dune
// date 07/07/2011
// 
// Given a real position cMo an estimated position cmMo
// Compute the feature s and sm
// Compute the interaction matrix L
// try to estimate the displacement between cMo and cmMo using the measured features
//--------------------------------------------------//


// sampling time
Te              = 0.01;

// Real pose in X Y Z Rx Ry Rz
posecMoRxyz     = [0.2 0.1 2 0 0 0 ]; 	
posecMoThetaU   = thetaUFromRxRyRz(posecMoRxyz); 
cMo             = homogeneousMatrixFromPos(posecMoRxyz); // convert pose vector into homogeneous matrix
oMc             = inv(cMo);


// Disturb the real position with random noise
//disturbance     = rand(posecMoRxyz,'normal')/100;
disturbance = [-0.0068342  -0.0072095 0.0081451 0.0032402 -0.0018848 0.0042416 ];
disp('Apply an error on the first pose estimation');
disp(disturbance);

// compute the disturbance in thetaU
distThetaU     = vThetaUFromVRxRyRz(disturbance); 
disp('Same disturbance but in thetaU')
disp(distThetaU);




// Model pose
cmMc            = homogeneousMatrixFromPos(disturbance); 

// object in measured camera
cmMo            = cmMc*cMo;
posecmMo        = pFromHomogeneousMatrix(cmMo);

// measured camera in real object
oMcm            = oMc * inv(cmMc)
poseoMcm        = pFromHomogeneousMatrix(oMcm);
poseoMcmThetaU  = vThetaUFromVRxRyRz(poseoMcm');

// camera in real obect
poseoMc         = pFromHomogeneousMatrix(oMc);
poseoMcThetaU   = vThetaUFromVRxRyRz(poseoMc');


// compute the velocity corresponding to this disturbance
vdist           = expMapInverseThetaU(cmMc,Te);
disp('the velocity Utheta corresponding to this motion is') 
disp(vdist);
vdistRxRyRz     = expMapInverseRxRyRz(cmMc,Te);
disp('the velocity Rxyz corresponding to this motion is') 
disp(vdistRxRyRz);

// difference of real pose and measured one 
disp('poseoMc-poseoMcm')
disp(poseoMc'-poseoMcm')

// difference between real pose and measured one
disp('poseoMcThetaU-poseoMcmThetaU')
disp(poseoMcThetaU-poseoMcmThetaU)


// Object
NbPts          = 4;
[oP cP s]      = mire4pointsInCam(0.20,cMo); // build the 3d points in the object and in the camera frame
Z              = cP(3:3:$); // compute the point depth
[oP cmP sm]    = mire4pointsInCam(0.20,cmMo);
Zm             = cmP(3:3:$);

//Desired position
global Zdes_global;
global sdes_global;
posecdesMo     = [0 0 1 0 0 0 ]; 	
cdesMo         = homogeneousMatrixFromPos(posecdesMo); // convert pose vector into homogeneous matrix
[oP cdesP sdes]= mire4pointsInCam(0.20,cdesMo); // build the 3d points in the object and in the camera frame
Zdes_global    = cdesP(3:3:$); // compute the point depth
sdes_global    = sdes;

Z = [2.5;2.2;2.4;2.1];

//----------------------------------------------------------------
//
//  Try to identify the error starting from the measure of s
//
//----------------------------------------------------------------

// Around s
Lc = matIntMireC(s,Z);
Ld = matIntMireD(s,Z);
Lm = matIntMireM(s,Z);
Lp = matIntMireP(s,Z);

// Around sm

//Lcm = matIntMireC(sm,Zm);
//Ldm = matIntMireD(sm,Zm);
//Lmm = matIntMireM(sm,Zm);
//Lpm = matIntMireP(sm,Zm);

// temptative
epsilon = pinv(Lc)*(s-sm);
epsilon = (abs(epsilon)>1E-8).*epsilon;

disp('epsilon')
disp(epsilon');




//cmMc = computeMotion(vCorr',Te);
cmMc2   = homogeneousMatrixFromPos(epsilon);
posecmMc= pFromHomogeneousMatrix(cmMc2);
posecMcm= pFromHomogeneousMatrix(inv(cmMc2));

disp('cmMc')
disp(posecmMc')

disp('cMcm')
disp(posecMcm')

oMc2 = oMcm*cmMc2;
posec2Mo = pFromHomogeneousMatrix(inv(oMc2));


disp('Pose estimee')
disp(posec2Mo')


//-----------------------------------------------------------------//
//             Finish the process
//-----------------------------------------------------------------//

disp('-------The End------')
xset("pixmap",0);
disp('pause before ending')
pause



