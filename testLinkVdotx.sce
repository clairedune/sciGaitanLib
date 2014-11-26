//---------------------------//
// Compute a cost function over a time horizon 
// With a free 3d camera
// December 11
// Dune 
// Note : 
//
// On etudie le lien entre dx/dt et v=(v, w)
//---------------------------//
clear
getd("src/transformation")

//posecMo_m     = [0 0 1 0.3 0.2 0.1 ];								
//cMo_m         = homogeneousMatrixFromPos(posecMo_m);
//oMc_m			   = inv(cMo_m);
//poseoMc_m     = pFromHomogeneousMatrix(oMc_m)';

poseoMc_m     = [0 0 1 0.3 0.2 0.1 ];								
oMc_m         = homogeneousMatrixFromPos(posecMo_m);


// petit deplacement de la camera dans le repere objet
DeltaX        = [0.01 0.01 0 0.001 0.02 0];
dt            = 0.1;
dotx          = DeltaX / dt;
dotu          = DeltaX (4:6)'/dt;
dott          = DeltaX (1:3)'/dt;


disp('deltaX par definition')
disp(DeltaX)

// --- non car DeltaX n'est pas cm dans c mais bien DeltaX=oXcm-oXc
poseoMcm_m    = poseoMc_m + DeltaX;
oMcm_m        = homogeneousMatrixFromPos(posecmMo_m);
cmMc_m        = cmMo_m*inv(cMo_m);
vexpMap       = expMapInverse(cmMc_m,dt)

// trouver w
otc           = poseoMc_m(1:3);
oRc           = oMc_m(1:3,1:3);
cto           = posecMo_m(1:3);
cRo           = cMo_m(1:3,1:3);
R             = cRo;

r             = posecMo_m(4:6);
t             = cto;
T             = eulerAngleRatesPrime(r);
w             = (T*dotu)' 

v             = dott


//R             = oRc
//r             = poseoMc_m(4:6);
//t             = otc;
//T             = eulerAngleRatesMatrix(r);
//w             = (R'*T*dotu)'
//w             = (T*dotu)'
//w             = (R*T*dotu)'

//trouver v
cVo           = twistMatrix(cMo_m);





