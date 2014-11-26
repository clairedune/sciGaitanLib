// Exp1 : erreur en translation et mouvement de translation
Delta         = [-0.04 0.07 0.0 0.00*%pi/180 0.0*%pi/180 0.00*%pi/180];
Uc_m          = [0.1; 0; 0.0; 0; 0; 0];     // init velocity


// Exp2 : erreur en translation et trans rot
Delta         = [-0.04 0.07 0.0 0.00*%pi/180 0.0*%pi/180 0.00*%pi/180];
Uc_m          = [0.1; 0; -0.3; 0; 0; 1];     // init velocity
//
//
//// Exp3 : erreur en translation et mouvement trans Z
//Delta         = [-0.04 0.07 0.0 0.00*%pi/180 0.0*%pi/180 0.00*%pi/180];
//Uc_m          = [0.1; 0; -0.3; 0; 0; 0];     // init velocity
//
//
//// Exp4 : erreur en translation et mouvement trans Z
//Delta         = [-0.04 0.07 0.0 0.01*%pi/180 0.1*%pi/180 0.03*%pi/180];
//Uc_m          = [0; 0.1; 0; 0; 0; 0];     // init velocity
//
////erreur en rotation et mouvement trans Z /// AIE
////Delta         = [0.0 0.0 0.0 10*%pi/180 0*%pi/180 0*%pi/180];
////Uc_m          = [0; 0.1; 0; 0; 0; 0];     // init velocity
//
////  erreur en rotation et mouvement trans Z /// AIE
////Delta         = [0.0 0.0 0.0 0*%pi/180 0*%pi/180 40*%pi/180];
////Uc_m          = [0; 0.1; 0; 0; 0; 0];     // init velocity
//
//// Exp4 : erreur en translation et mouvement trans Z
//Delta         = [-0.04 0.07 0.02 0.01*%pi/180 0.1*%pi/180 0.03*%pi/180];
//Uc_m          = [0.4; 0.1; 0.2; 0; 0; 0.5];     // init velocity


//--- Create the camera pose and the object pose  
posecMo_m     = [0 0 1 0 0 0 ];								 // pose target/object init  
posewMo_m     = [0 0 0 0 %pi/2 0 ];       // pose of the target in the World Frame

//--- Build the corresponding homogeneous matrix
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
wMo_m         = homogeneousMatrixFromPos(posewMo_m); 
oMc_m			   = inv(cMo_m);
posecMo_m     = pFromHomogeneousMatrix(cMo_m)';
poseoMc_m     = pFromHomogeneousMatrix(oMc_m)';

//--- Build the target
a_m           = 0.30;                              // dimension of the target  
oP_m          = mire4points(a_m);                  // create the Npbts Points Target
NbPts_m       = length(oP_m)/3 ;
wP_m          = changeFrameMire(oP_m,wMo_m);

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;

// ------ Estimated First pose = first pose + perturbation

//Delta         = [0.0 0.0 0.0 1*%pi/180 2*%pi/180 0.3*%pi/180]; 
//[0.01 0.01 0.002 0.0*%pi/180 0.*%pi/180 0.03*%pi/180];
//Delta = [0.0051151 -0.0516646 -0.0642930  0.0298558 0.0305389 -0.0528393 ];
poseoMcm_m    = poseoMc_m+Delta;								
oMcm_m        = homogeneousMatrixFromPos(poseoMcm_m);
cmMo_m			   = inv(oMcm_m);
posecmMo_m     = pFromHomogeneousMatrix(cmMo_m)'


//--- compute the false projection on the view
cmP_m         = changeFrameMire(oP_m,cmMo_m);
sm_m          = projectMireDirect(cmP_m);
Zm_m          = cmP_m(3:3:$)  ;

// ------ Sampling time
Te_m          =0.04;                              // to be consistant with the image frame rate

// ------ Visual Constraints definition
xu_m          = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl_m          = [ -0.22 ; -0.22 ];                 // position min of the a 2D point in the image plane 
cote_m         = 0.01 ;
                             // command bounds on the horizon

//-------------------------------------
//
// Compute prediction on Horizon
//
//-------------------------------------


// --- repeat the control Np times on the control horizon
Np_m          = 50;                                // horizon lenght
Nc_m          = 1;                                 // command horizon length, as we have only one repeted control, it is 1  
//Uc_m          = [0; 0; -0.3; 0; 0; 1];     // init velocity
Up_m          = computeControlOnHorizon(Uc_m,Nc_m,Np_m);


// --- compute the position of the camera over the time horizon / G stands for Global
// This function computes 
// 1. All the position on the time horizon
// 2. The positions of the object in the camera frame / deduced Z
// 3. The projections of the object in the camera frame
[sHorG_m c1McN_m ZHorG_m] = preHorGlobalMirePos(oP_m,cMo_m,Up_m,Te_m,Np_m);
first_m       = 1;
last_m        = 2*NbPts_m;

// --- same for the estimated camera / sm means s measured
[smHorG_m cm1McmN_m ZmHorG_m] = preHorGlobalMirePos(oP_m,cmMo_m,Up_m,Te_m,Np_m);

disp('Compare les vraies features avec celles')
disp('obtenues par un model global mais une mauvaise position de depart')
hf_10 = createPlanImage(10,xl_m,xu_m,"Real features - / Predicted features -.");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-') ; 
mireEvolutionMulticolor(Np_m,smHorG_m,'-.') ; 
show_pixmap();
pause;


//--- DeltaS = sm-s / Linear correction : apply the same epsilon for all the s
s0_m         = sHorG_m(first_m:last_m);
sm0_m        = smHorG_m(first_m:last_m);
DeltaS_m     = sm0_m-s0_m;

// apply the computed epsilon on all the prediction si = si - epsilon_m
smHorGlinCorr_m  = linearCorrection(smHorG_m,DeltaS_m, Np_m);

disp('linear feature correction using 3 methods')
hf_12           = createPlanImage(12,xl_m,xu_m,"Correction of feature using DeltaS");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-.');
mireEvolutionMulticolor(Np_m,smHorGlinCorr_m,'-') ; // motion then projection
show_pixmap();
//pause


// --- Epsilon = L^+(sm-s) / X correction means the pose cXo is directly corrected
Ls_m  = matIntMireC(s_m,Z_m);
Lx_c = computeLxInCam(cMo_m);

DeltaX_m = pinv(Ls_m*Lx_c)*(-s0_m+sm0_m);

//DeltaX_m = pinv(Ls_m)*(-s0_m+sm0_m);

poseoMce_m    = poseoMcm_m-DeltaX_m';
disp('----POSE ESTIMEE----')
disp(poseoMce_m);
//pause

oMce_m        = homogeneousMatrixFromPos(poseoMce_m);
ceMo_m			   = inv(oMce_m);
poseceMo_m     = pFromHomogeneousMatrix(ceMo_m)'


[seHorG_m, ce1MceN_m, Ze_m] = preHorGlobalMirePos(oP_m,ceMo_m,Up_m,Te_m,Np_m);
ceP_m         = changeFrameMire(oP_m,ceMo_m);
se_m          = projectMireDirect(ceP_m);
Ze_m          = ceP_m(3:3:$)  ;

//pause

disp('Estimation of the correction feature using DeltaX')

hf_13           = createPlanImage(13,xl_m,xu_m,"Estimation of corrected features");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-.');
show_pixmap();
mireEvolutionMulticolor(Np_m,seHorG_m,'-') ; // incremental
show_pixmap();
//pause



//---------Trajectory of 2D points
hf_1           = createPlanImage(5,xl_m,xu_m,"3 mire evolutions : real, mesured, estimated");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionDraw(Np_m,sHorG_m,1,'-k') ; // real target evolution
show_pixmap();
disp('real target evolution')
//pause
mireEvolutionDraw(Np_m,smHorG_m,1,'-r') ; // estimated target evolution
disp('stimated target evolution')
show_pixmap();
//pause
mireEvolutionDraw(Np_m,seHorG_m,1,'-g') ; // corrected target evolution epsilon = L^+(s-sm)
disp('corrected target evolution epsilon = L^+(s-sm)')
show_pixmap();
//pause
mireEvolutionDraw(Np_m,smHorGlinCorr_m,1,'-b') ; // corrected target evolution epsilon = (s-sm)
show_pixmap();

//hf_2            = createFigure3D(1,"Camera Motion",1);
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




 







// --- compute the feature position using the local model sk+1 = sk+ TeLv 
//global computeL_global;
//computeL_global		= matIntMireC; // Z and S current
//smHorLc_m         = ga_predHorLoc2dMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
//epsilon_m         = smHorLc_m((first_m:last_m))-s0_m;
//smHorLcCorr_m     = linearCorrection(smHorLc_m,epsilon_m, Np_m);

//computeL_global = matIntMireD; // Z and S desired
//smHorLd_m         = ga_predHorLoc2dMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
//epsilon_m         = smHorLd_m((first_m:last_m))-s0_m;
//smHorLdCorr_m     = linearCorrection(smHorLd_m,epsilon_m, Np_m);

//computeL_global   = matIntMireM; // Mixte Ld and Lc
//smHorLm_m           = ga_predHorLoc2dMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
//epsilon_m         = smHorLm_m((first_m:last_m))-s0_m;
//smHorLmCorr_m     = linearCorrection(smHorLm_m,epsilon_m, Np_m);

//computeL_global   = matIntMireP; // S current and Z desired
//disp('test');
//smHorLp_m         = ga_predHorLoc2dMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
//disp('test1');
//epsilon_m         = smHorLp_m((first_m:last_m))-s0_m;
//smHorLpCorr_m     = linearCorrection(smHorLp_m,epsilon_m, Np_m);

// --- compute the feature position using the global model s(x)
//smHorG_m = ga_predHorGlobalMire(sm_m,Zm_m,Up_m,Te_m,Np_m);

// --- keep L constant
//smHorConst_m      = predHorLoc2dMireConst(sm_m,Zm_m,Up_m,Te_m,Np_m);
//epsilon_m         = smHorConst_m((first_m:last_m))-s0_m;
//smHorConstCorr_m  = linearCorrection(smHorConst_m,epsilon_m, Np_m);

// --- compute a false representation by modifying Z : 
//Zmod = Z + rand(Z)/10;
//smHorGmod = ga_predHorGlobalMire(s,Zmod,Up,Te,Np);


//hf_1           = createPlanImage(4,xl,xu,"Point 2D trajectory global and noisy");
//mire2DDraw(s_m,cote,4);
//mireEvolutionDraw(Np_m,sHorG_m,1,'-k') ;
//mireEvolutionDraw(Np_m,smHorG_m,1,'-r') ;
////mireEvolutionDraw(Np_m, seHorG,1,'-g');
////mireEvolutionDraw(Np_m,smHorGmotionCorr,1,'-b');
//
//show_pixmap();
//
//pause;

// image plane
//xl             = [-0.3;-0.3] ;
//xu             = [0.3; 0.3] ;
//cote_m         = 0.01 ;
//hf_1           = createPlanImage(2,xl,xu,"Point 2D trajectory local");
//mire2DDraw(s_m,cote_m,4);
//mireEvolutionDraw(Np_m,smHorLc_m,1,'-b') ;
//mireEvolutionDraw(Np_m,smHorLd_m,1,'-g') ;
//mireEvolutionDraw(Np_m,smHorLm_m,1,'-r') ;
//mireEvolutionDraw(Np_m,smHorLp_m,1,'-c') ;
//mireEvolutionDraw(Np_m,smHorConst_m,1,'-m');
//mireEvolutionDraw(Np_m,smHorG_m,1,'-k') ;
//show_pixmap();


//---------Correction
//hf_5           = createPlanImage(5,xl,xu,"Corrected Point 2D trajectory local");
//mire2DDraw(s_m,cote_m,4);
//mireEvolutionDraw(Np_m,smHorLcCorr_m,1,'-.b') ;
//mireEvolutionDraw(Np_m,smHorLdCorr_m,1,'-.g') ;
//mireEvolutionDraw(Np_m,smHorLmCorr_m,1,'-.r') ;
//mireEvolutionDraw(Np_m,smHorLpCorr_m,1,'-.c') ;
//mireEvolutionDraw(Np_m,smHorConstCorr_m,1,'-.m')
//show_pixmap();

