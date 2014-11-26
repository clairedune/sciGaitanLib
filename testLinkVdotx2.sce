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


disp('test')
posecMo_m     = [0 0 0 0 0 0 ]								
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
oMc_m			   = inv(cMo_m);
poseoMc_m     = pFromHomogeneousMatrix(oMc_m)'

// petit deplacement de l'objet dans le repere de la camera
DeltaX        = [0.01 0.002 0.003 0.0001 0.0 0];
dt            = 0.1;
dotx          = DeltaX / dt;
dotu          = DeltaX (4:6)'/dt;
dott          = DeltaX (1:3)'/dt;

disp('deltaX par definition')
disp(DeltaX)

//cmMc_m        = homogeneousMatrixFromPos(DeltaX);
//vexpMap       = expMapInverse(cmMc_m,dt);
//disp('vitesse estimee en passant par homogeneousMatrixFromPos(DeltaX)')
//disp(vexpMap)

poseoMcm_m    = poseoMc_m + DeltaX;
oMcm_m        = homogeneousMatrixFromPos(poseoMcm_m)
cmMc_m        = inv(oMcm_m)*oMc_m;
posecmMc_m    = pFromHomogeneousMatrix(cmMc_m)'
cMcm_m        = inv(cmMc_m);
posecMcm_m    = pFromHomogeneousMatrix(cMcm_m)'

// si on met dr=0 et une rotation sur cMo alors w=0  
vexpMap       = expMapInverse(cMcm_m,dt)


// trouver w
otc           = poseoMc_m(1:3);
oRc           = oMc_m(1:3,1:3);
cto           = posecMo_m(1:3);
cRo           = cMo_m(1:3,1:3);
R             = cRo;
r             = posecMo_m(4:6);
T             = eulerAngleRatesPrime(r);
w1             = (R'*T*dotu)'
w2             = (T*dotu)'
w3             = (R*T*dotu)'
T             = eulerAngleRatesMatrix(r);
w4             = (R'*T*dotu)'
w5             = (T*dotu)'
w6             = (R*T*dotu)'

R             = oRc;
r             = poseoMc_m(4:6);
T             = eulerAngleRatesPrime(r);
w7             = (R'*T*dotu)'
w8             = (T*dotu)'
w9             = (R*T*dotu)'
T             = eulerAngleRatesMatrix(r);
w10             = (R'*T*dotu)'
w11             = (T*dotu)'
w12             = (R*T*dotu)'
//trouver v
cVo           = twistMatrix(cMo_m);






//------------------------------------------//
// J'ai vel et je veux trouver dx           //
//------------------------------------------//

//velo         = inv(cVo)*vel'

//dX           = dt*velo(1:3)

//dU           = dt*inv(T)*velo(4:6)



