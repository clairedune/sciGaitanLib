//---------------------------//
// Compute a cost function over a time horizon 
// With a free 3d camera
// December 11
// Dune 
// Note : 
//
// On construit un tour d'optimisation
// La fonction de cout de ne dépend que de Uc
// Uc est le vecteur de commande
// Nc (nb commande differentes) n'est pas forcement egal à Np le nombre de commandes
// Si Nc<Np alors tous les pas de l'horizon > Nc valent la derniere commande
// Il faudra donc toujours calculer Up a partir de Uc, Nc et Np
//---------------------------//
function compPred()

//EXP2
disturbance = [ -0.0373050    0.0704331  -0.0860551  -0.0002348   -0.0857879  0.0092117]
Uc_m          = [0; 0; -0.3; 0; 0; 1];     // init velocity







//--- Create the camera pose and the object pose  
posecMo_m     = [0 0 2 0 0 0 ];								 // pose target/object init  
posewMo_m     = [0 0 0 0 %pi/2 0 ];       // pose of the target in the World Frame

//--- Build the corresponding homogeneous matrix
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
wMo_m         = homogeneousMatrixFromPos(posewMo_m); 
oMc_m			   = inv(cMo_m);

//--- Build the target
a_m           = 0.40;                              // dimension of the target  
oP_m          = mire4points(a_m);                  // create the Npbts Points Target
NbPts_m       = length(oP_m)/3 ;
wP_m          = changeFrameMire(oP_m,wMo_m);

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;


// ------ Estimated First pose = first pose + perturbation
//disturbance     = rand(posecMo,'normal')/10;
//disturbance   = [-0.0068342  -0.0072095 0.0081451 0.0032402 -0.0018848 0.0042416 ];
//disturbance   = [0 0 0 0 0 %pi/2]




cmMc_m        = homogeneousMatrixFromPos(disturbance); 
cmMo_m        = cmMc_m*cMo_m;
oMcm_m        = inv(cmMo_m) ;

//--- compute the false projection on the view
cmP_m         = changeFrameMire(oP_m,cmMo_m);
sm_m          = projectMireDirect(cmP_m);
Zm_m          = cmP_m(3:3:$)  ;

// ------ Sampling time
Te_m          = 0.03;                              // to be consistant with the image frame rate
Te_simu			   = 0.03;

// ------ Visual Constraints definition
xu_m          = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl_m          = [ -0.22 ; -0.22 ];                 // position min of the a 2D point in the image plane 
cote_m         = 0.01 ;


//--- Predictive control parameters
thres_m       = 1e-2;                              // error threshold  
lambda        = 1; 

// --- repeat the control Np times on the control horizon
Np_m          = 100;                                // horizon lenght
Nc_m          = 1;                                 // command horizon length, as we have only one repeted control, it is 1  
//Uc_m          = [0; 0; -0.3; 0; 0; 1];     // init velocity

// 

Up_m          = computeControlOnHorizon(Uc_m,Nc_m,Np_m);


// --- compute the position of the camera over the time horizon / G stands for Global
[sHorG_m c1McN_m ZHorG_m] = preHorGlobalMirePos(oP_m,cMo_m,Up_m,Te_m,Np_m);
first_m       = 1;
last_m        = 2*NbPts_m;
// --- same for the estimated camera / sm means s measured
[smHorG_m cm1McmN_m ZmHorG_m] = preHorGlobalMirePos(oP_m,cmMo_m,Up_m,Te_m,Np_m);

disp('Compare les vraies features avec celles')
disp('obtenues par un model global mais une mauvaise position de depart')
hf_10 = createPlanImage(10,xl_m,xu_m,"Real features - / Predicted features -.");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-.') ; 
mireEvolutionMulticolor(Np_m,smHorG_m,'-') ; 
show_pixmap();
pause;


//--- DeltaS = sm-s / Linear correction : apply the same epsilon for all the s
s0_m          = sHorG_m(first_m:last_m);
sm0_m        = smHorG_m(first_m:last_m);
DeltaS_m     = sm0_m-s0_m;

// apply the computed epsilon on all the prediction si = si - epsilon_m
smHorGlinCorr_m  = linearCorrection(smHorG_m,DeltaS_m, Np_m);

disp('linear feature correction using 3 methods')
hf_12           = createPlanImage(12,xl_m,xu_m,"Correction of feature using DeltaS");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'..');
mireEvolutionMulticolor(Np_m,smHorGlinCorr_m,'-') ; // motion then projection
show_pixmap();


// --- Epsilon = L^+(sm-s) / X correction means the pose cXo is directly corrected
L_m = matIntMireC(s_m,Z_m);
[DeltaX_m, cmMce_m]  = epsilonX(s0_m,sm0_m,L_m);
ceMo_m						= inv(cmMce_m) * cmMo_m; 
oMce_m						= inv (ceMo_m);
[seHorG_m, ce1MceN_m, Ze_m] = preHorGlobalMirePos(oP_m,ceMo_m,Up_m,Te_m,Np_m);
ceP_m         = changeFrameMire(oP_m,ceMo_m);
se_m          = projectMireDirect(ceP_m);
Ze_m          = ceP_m(3:3:$)  ;


disp('Estimation of the correction feature using DeltaX')

hf_13           = createPlanImage(13,xl_m,xu_m,"Estimation of corrected features");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'..');
mireEvolutionMulticolor(Np_m,seHorG_m,'-') ; 
show_pixmap();



//---------Trajectory of 2D points
hf_1           = createPlanImage(5,xl_m,xu_m,"3 mire evolutions : real, mesured, estimated");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionDraw(Np_m,sHorG_m,1,'-k') ; // real target evolution
show_pixmap();
disp('real target evolution')
pause
mireEvolutionDraw(Np_m,smHorG_m,1,'-r') ; // estimated target evolution
disp('stimated target evolution')
show_pixmap();
mireEvolutionDraw(Np_m,seHorG_m,1,'-g') ; // corrected target evolution epsilon = L^+(s-sm)
disp('corrected target evolution epsilon = L^+(s-sm)')
show_pixmap();

mireEvolutionDraw(Np_m,smHorGlinCorr_m,1,'-b') ; // corrected target evolution epsilon = (s-sm)
disp('corrected target evolution epsilon = (s-sm)')
show_pixmap();


number=10;
titlefig="Camera Motion";
scale=1;

xset("window",number);
  xset("pixmap",1);
  xbasc()//effacement de la fenêtre
  clear_pixmap()//et buffer
  hf=scf(number);
  hf.figure_name = titlefig;
  ha=hf.children;
  ha.box="on"; 
  ha.view="3d";
  ha.thickness=1;
  ha.foreground=0;
  axe=gca(); // recupere un pointeur sur les axes
  axe.x_label.text="x"; // texte des label
  axe.y_label.text="y"; // y label
  axe.z_label.text="z"; // z label
  //axe.data_bounds=[-scale,-scale,-scale;scale,scale,scale]; // set the boundary values for the x y and z axis 
  axe.isoview="off"; // empeche le changement d'echelle
  axe.grid =[1 1 1];
  axe.auto_clear = "off" ;
  axe.data_bounds=[[-2,0.1]'; [-0.5,0.5]';[0,0.5]'];


Mire3DDraw4pts(wP_m);

show_pixmap();
for i=1:5:Np_m
    // This is the real 3D position
    c1Mc2_m        = c1McN_m(((i-1)*4+1:(i-1)*4+4),:)  ;//noir        
    wMc_display    = wMo_m * oMc_m * c1Mc2_m;
	Camera3DDrawColor(0.05,wMc_display,1);
		//show_pixmap();
		
		// This is the predicted 3D position starting from the current measurement
    cm1Mcm2_m      = cm1McmN_m(((i-1)*4+1:(i-1)*4+4),:)  ;         
    wMcm_display   = wMo_m * oMcm_m * cm1Mcm2_m;
    Camera3DDrawColor(0.05,wMcm_display,5); //rouge
    //show_pixmap(); 
		 
		
		// This is the estimated position.
    ce1Mce2_m      = ce1MceN_m (((i-1)*4+1:(i-1)*4+4),:);   
    wMce_display   = wMo_m * oMce_m*ce1Mce2_m;
	Camera3DDrawColor(0.05,wMce_display,3); //vert  
		//show_pixmap(); 
end
    show_pixmap





//-----------------------------------------------------------------//
//             Finish the process
//-----------------------------------------------------------------//

disp('-------The End------')
xset("pixmap",0);
disp('pause before ending')
pause




 
endfunction






