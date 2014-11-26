//---------------------------//
// Compute a cost function over a time horizon 
// With a free 3d camera
// December 11
// Dune 
// Note : 
//
// On etudie le lien entre dx/dt et v=(v, w)
//---------------------------//
function testEulerAngleRates()

//--- Create the camera pose and the object pose  
posecMo_m     = [0 0 1 0.3 0.2 0.1 ];								

//--- Build the corresponding homogeneous matrix
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
oMc_m			   = inv(cMo_m);
poseoMc_m     = pFromHomogeneousMatrix(oMc_m);

//--- Build the target
a_m           = 0.30;                              // dimension of the target  
oP_m          = mire5points(a_m);                  // create the Npbts Points Target
Nbpts_m       = length(oP_m)/3 ;

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;

// ------ Estimated First pose = first pose + perturbation
//disturbance     = rand(posecMo,'normal')/10;
//disturbance   = [-0.0068342  -0.0072095 0.0081451 0.0032402 -0.0018848 0.0042416 ];
DeltaX        = [0.00 0.00 0 0.001 0.001 0];
dt            = 0.01;
dotx          = DeltaX / dt;
dotu          = DeltaX (4:6)'/dt;
dott          = DeltaX (1:3)'/dt;

Lx            = computeLx(cMo_m);
vel           = Lx*dotx';

disp('vitesse estimee en a partir de dx et cMo_m')
disp(vel)

pause;

poseoMcm_m    = poseoMc_m+DeltaX';
oMcm_m        = homogeneousMatrixFromPos(poseoMcm_m);
cmP_m         = changeFrameMire(oP_m,inv(oMcm_m));
sm_m          = projectMireDirect(cmP_m);
Zm_m          = cmP_m(3:3:$)  ;


// --- Epsilon = L^+(sm-s) / X correction means the pose cXo is directly corrected
L_m = matIntMireC(s_m,Z_m);
DeltacXest = pinv(L_m*Lx)*(sm_m-s_m);///dt;
DeltacXest2= pinv(L_m)*(sm_m-s_m);///dt;

disp('DeltaX_m')
disp(DeltacXest)
disp('DeltacXest 2')
disp(DeltacXest2)


disp('dots=(sm_m-s_m)/dt')
disp(((sm_m-s_m))'/dt)

disp('dots=Lv')
disp((L_m*vel)')

pause;

//[DeltaX_m, cmMce_m]  = epsilonX(s0_m,sm0_m,L_m);




////-----------------------------------------------------------------//
////             Finish the process
////-----------------------------------------------------------------//
//
disp('-------The End------')

disp('pause before ending')
pause




 
endfunction




