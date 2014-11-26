//---------------------------//
// Compute a cost function over a time horizon 
// With a free 3d camera
// Created 11/12 updated 27 july 2012
// Etude pour ICRA 2012
// Dune 
// Note : 
// On etudie le lien entre dx/dt=(dotp dotu) et v=(v, w)
//---------------------------//

getd("src/transformation")

posecMo_m     = [0 0 1 0*%pi/180 0*%pi/180 0*%pi/180]								
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
oMc_m			   = inv(cMo_m);
poseoMc_m     = pFromHomogeneousMatrix(oMc_m)';

// petit deplacement de la camera dans le repere de l'objet
Delta         = [0 0. 0. 0.001 0.002 0.003];
dt            = 1/25;
dotx          = Delta / dt;
dotu          = Delta (4:6)'/dt;
dotp          = Delta (1:3)'/dt;

disp('deltaX par definition')
disp(Delta)

disp('dotX par definition')
disp(dotx)

//-----On cherche v qui produit le meme deplacement sur dt--------
// la matrice de rotation qui nous interesse ici est celle qui est construite
// parti des Euler Angle Rate 
cRo           = cMo_m(1:3,1:3)
r             = RxRyRzfromRotationMatrix(cRo);//camera dans objet

//a=r(3);
//b=r(2);
//c=r(1);
//R1= [cos(a)*cos(b) cos(a)*sin(b)*sin(c)-sin(a)*cos(c) cos(a)*sin(b)*cos(c)+sin(a)*sin(c)
//    sin(a)*cos(b) sin(a)*sin(b)*sin(c)+cos(a)*cos(c) sin(a)*sin(b)*cos(c)-cos(a)*sin(c)
//    -sin(b) cos(b)*sin(c) cos(b)*cos(c)];
//R=[cos(a)*cos(b) -sin(a)*cos(b) sin(b)
//   cos(c)*sin(a)+sin(c)*sin(b)*cos(a) cos(a)*cos(c)-sin(c)*sin(a)*sin(b) -sin(c)*cos(b)
//   sin(a)*sin(c)-cos(c)*sin(b)*cos(a) cos(a)*sin(c)+cos(c)*sin(a)*sin(b) cos(b)*cos(c)];
  
// on calcul la matrice E pour obtenir w dans le repere objet
oT             = eulerAngleRatesMatrix(r);
ow             = oT*dotu;
ow             = ow .*(ow >100*%eps);

//disp('Eijk');
//disp(oT);
//disp('w dans le repere objet');
//disp(ow);

// on calcul la matrice conjuguee pour obtenir w dans le repere camera
cT          = eulerAngleRatesConj(r);
cw          = cT*dotu;
cw          =cw .*(cw >100*%eps);

//Rtest=cT*inv(oT);
//disp('Rtest')
//disp(Rtest)

//Rtest2=oT*inv(cT);
//disp('Rtest2')
//disp(Rtest2)

//disp('Eijk conj');
//disp(cT);
//disp('w dans le repere camera');
//disp(cw);



//verif :
//winv2 = cRo*ow;
//disp ('verif on doit retrouver cw');
//disp (winv2);

//pause


// dans le repere objet : 

ov = [dotp ; ow];
disp('vitesse exprimee dans le repere objet')
disp(ov)

Lx = computeLxInObj(cMo_m);
ov = Lx*dotx';
disp('vitesse exprimee dans le repere objet en utilisant Lx')
disp(ov)

//translation = cMo_m(1:3,4);
//tx = skew(translation);
//cv = [cRo*dotp+tx*cw;cw];
//disp('vitesse exprimee dans le repere camera')
//disp(cv)

Lx = computeLxInCam(cMo_m);
cv = Lx*dotx';
disp('vitesse exprimee dans le repere camera en utilisant Lx')
disp(cv)

//cv3 = cameraVelFromDeltaX(cMo_m, Delta', dt);
//disp('vitesse exprimee dans le repere camera en utilisant Fonction')
//disp(cv3)

cVo_m=twistMatrix(cMo_m);
camv = cVo_m*ov;
disp('vitesse dans le repere de la camera en utilisant cVo');
disp(camv)

pause


//-----TESTS-------//

//--- Build the target
a_m           = 0.30;                              // dimension of the target  
oP_m          = mire5points(a_m);                  // create the Npbts Points Target
Nbpts_m       = length(oP_m)/3 ;

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;



// ------ Delta est un ajout a la pose courante de la camera ----//
poseoMcm_m    = poseoMc_m+Delta;
disp('Position d arrivee en utilisant ce petit dx')
disp('poseoMcm_m = poseoMc_m+Delta;')
disp(poseoMcm_m)

// ----- Appliquons la vitesse camera pendant dt
vitesse       = cv;
cMcm_m        = expMapDirectThetaU(vitesse,dt);
//cMcm_m = expMapDirectRxRyRz(vitesse',dt);
vexpMap       = expMapInverse(cMcm_m,dt);
//vexpMap       = expMapInverseRxRyRz(cMcm_m,dt);

disp('vitesse camera retrouvee')
disp(vexpMap)

posecMcm_m    = pFromHomogeneousMatrix(cMcm_m)';
disp('deplacement de la cam dans le repere cam si on applique cv')
disp(posecMcm_m)


oMcm_m        = oMc_m*cMcm_m;
poseoMcm_m    = pFromHomogeneousMatrix(oMcm_m)';
disp('pose d arrivee obtenu par deplacement de la camera')
disp(poseoMcm_m)

cmP_m         = changeFrameMire(oP_m,inv(oMcm_m));
sm_m          = projectMireDirect(cmP_m);
Zm_m          = cmP_m(3:3:$)  ;

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
disp((L_m*vitesseL)')





disp('----Idem en O----')

vitesse       = ov;
cMcm_m        = expMapDirectThetaU(vitesse,dt);
//cMcm_m = expMapDirectRxRyRz(vitesse',dt);
vexpMap       = expMapInverse(cMcm_m,dt);
//vexpMap       = expMapInverseRxRyRz(cMcm_m,dt);

disp('vitesse camera retrouvee')
disp(vexpMap)

posecMcm_m    = pFromHomogeneousMatrix(cMcm_m)';
disp('deplacement de la cam dans le repere cam si on applique cv')
disp(posecMcm_m)



oMcm_m        = oMc_m*cMcm_m;
poseoMcm_m    = pFromHomogeneousMatrix(oMcm_m)';
disp('pose d arrivee obtenu par deplacement de la camera')
disp(poseoMcm_m)

