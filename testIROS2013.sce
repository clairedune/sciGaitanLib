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


// pose reelle
disp('pose initiale')
poseoMc_m     = [-0.02 0.01 -1 0*%pi/180 0*%pi/180 30*%pi/180]								
oMc_m        = homogeneousMatrixFromPos(poseoMc_m);
cMo_m			 = inv(oMc_m);
posecMo_m     = pFromHomogeneousMatrix(cMo_m)';


// petit deplacement de la camera dans le repere de l'objet
Delta         = [0.01 0.1 0.02 0.0*%pi/180 0.1*%pi/180 0.3*%pi/180];
dt            = 1/10;
dotx          = Delta / dt;
dotu          = Delta (4:6)'/dt;
dotp          = Delta (1:3)'/dt;

disp('deltaX par definition')
disp(Delta)

disp('dotX par definition')
disp(dotx)


// pose modelisee
disp('pose decallee')
poseoMcm_m    = poseoMc_m+Delta;								
oMcm_m         = homogeneousMatrixFromPos(poseoMcm_m);
cmMo_m			   = inv(oMcm_m);
posecmMo_m     = pFromHomogeneousMatrix(cmMo_m)'

//-----On cherche v qui produit le meme deplacement sur dt--------
Lx_o = computeLxInObj(cMo_m);
ov = Lx_o*dotx';
disp('vitesse exprimee dans le repere objet en utilisant Lx')
disp(ov')

Lx_c = computeLxInCam(cMo_m);
cv = Lx_c*dotx';
disp('vitesse exprimee dans le repere camera en utilisant Lx')
disp(cv')
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

cmP_m          = changeFrameMire(oP_m,cmMo_m);
sm_m           = projectMireDirect(cmP_m);
Zm_m           = cmP_m(3:3:$)  ;

//matrice d'interaction
Ls_m = matIntMireC(s_m,Z_m);
disp('dots=(sm_m-s_m)/dt')
disp(((sm_m-s_m))'/dt)
//
disp('dots=Ls *Lx *dotx')
disp((Ls_m*Lx_c*dotx')')

disp('s sm sm-LsLxdotx*dt')
sest_m = sm_m-dt*Ls_m*Lx_c*dotx';
disp([s_m sm_m sest_m])

  disp('DeltaX ------ -Ls*Lx*(s_m-sm_m)')
DeltaX = pinv(Ls_m*Lx_c)*(-s_m+sm_m);
disp(DeltaX)
disp('Delta')
disp(Delta)
pause

//-------DRAW --------//
xu_m   = [  0.2 ;  0.2 ];                 // position max of the a 2D point in the image plane 
xl_m   = [ -1 ; -1 ];                 // position min of the a 2D point in the image plane 
cote_m = 0.03 ;

hf_13 = createPlanImage(13,xl_m,xu_m,"2D target projection");
mire2DDraw(s_m,cote_m,1); // initial target position
mire2DDraw(sm_m,cote_m,2); // initial target position
mire2DDraw(sest_m,cote_m,4); // initial target position
show_pixmap();

