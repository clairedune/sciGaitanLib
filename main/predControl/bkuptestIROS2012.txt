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

//--- Create the camera pose and the object pose  
posecDesMo_m  = [0 0 0.5 0 0 0 ];       // pose target/object desired
posecMo_m     = [0 0 2 0 0 0 ];								 // pose target/object init  
posewMo_m     = [0 0 0 0 %pi/2 0 ];       // pose of the target in the World Frame

//--- Build the corresponding homogeneous matrix
cDesMo_m      = homogeneousMatrixFromPos(posecDesMo_m);
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
wMo_m         = homogeneousMatrixFromPos(posewMo_m); 
oMc_m			   = inv(cMo_m);

//--- Build the target
a_m           = 0.30;                              // dimension of the target  
oP_m          = mire4points(a_m);                  // create the Npbts Points Target
NbPts_m       = length(oP_m)/3 ;
wP_m          = changeFrameMire(oP_m,wMo_m);

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;


//--- compute the desired projection on the view
cDesP_m       = changeFrameMire(oP_m,cDesMo_m);
sDes_m        = projectMireDirect(cDesP_m);
ZDes_m        = cDesP_m(3:3:$);    

// ------ Estimated First pose = first pose + perturbation
//disturbance     = rand(posecMo,'normal')/10;
//disturbance   = [-0.0068342  -0.0072095 0.0081451 0.0032402 -0.0018848 0.0042416 ];
//disturbance   = [0 0 0 0 0 %pi/2]
disturbance = [ -0.0373050    0.0704331  -0.0860551  -0.0002348   -0.0857879  0.0092117]

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
// ------ Control Constraints definition
bu_m          = 1e3*0.25*ones(6,1);                // command bounds
bl_m          = -bu_m;                             // command bounds on the horizon

//-------------------------------------
//
// Compute prediction on Horizon
//
//-------------------------------------

//--- Predictive control parameters
thres_m       = 1e-2;                              // error threshold  
lambda        = 1; 

// --- repeat the control Np times on the control horizon
Np_m          = 200;                                // horizon lenght
Nc_m          = 1;                                 // command horizon length, as we have only one repeted control, it is 1  
//Uc_m          = [0; 0; -0.3; 0; 0; 1];     // init velocity
Uc_m          = [0; 0; 0.1; 0; 0; 0.3];     // init velocity
Up_m          = computeControlOnHorizon(Uc_m,Nc_m,Np_m);
Q_m           = matWeightIdentity(Np_m,NbPts_m);

// --- build the desired feature vector. For a start it is constant
sDesHor_m			= constSdOnHor(sDes_m,Np_m);

// --- compute the position of the camera over the time horizon / G stands for Global
// This function computes 
// 1. All the position on the time horizon
// 2. The positions of the object in the camera frame / deduced Z
// 3. The projections of the object in the camera frame
[sHorG_m c1McN_m ZHorG_m] = preHorGlobalMirePos(oP_m,cMo_m,Up_m,Te_m,Np_m);
first_m       = 1;
last_m        = 2*NbPts_m;

//--- same starting from the feature position and estimation of Z
// 1 compute c0P from s0 and Z0
// 2 compute the motion between c0 and c1
// 3 deduce c1P
// 4 project C1 on the image plane and deduce s1 and Z1
// 5 compute c1P from s1 and Z1
// 6 etc ... 
//sHorG_incremental_m = ga_predHorGlobalMire(s_m,Z_m,Up_m,Te_m,Np_m);

//--- same but compute the positions
// 1 compute c0P from s0 and Z0
// 2 compute all the camera position
// 3 compute cP/ deduce Z
// 4 project cP on the image plane 
//sHorG_from2D_m = cld_predHorGlobalMire(s_m,Z_m,Up_m,Te_m,Np_m);

// --- same for the estimated camera / sm means s measured
[smHorG_m cm1McmN_m ZmHorG_m] = preHorGlobalMirePos(oP_m,cmMo_m,Up_m,Te_m,Np_m);
// 2nd method 
//smHorG_incremental_m = ga_predHorGlobalMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
// 3rd method
//smHorG_from2D_m = cld_predHorGlobalMire(sm_m,Zm_m,Up_m,Te_m,Np_m);
disp('Compare les vraies features avec celles')
disp('obtenues par un model global mais une mauvaise position de depart')
hf_10 = createPlanImage(10,xl_m,xu_m,"Real features - / Predicted features -.");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-') ; 
mireEvolutionMulticolor(Np_m,smHorG_m,'-.') ; 
show_pixmap();
pause;


//--- DeltaS = sm-s / Linear correction : apply the same epsilon for all the s
s0_m          = sHorG_m(first_m:last_m);
sm0_m        = smHorG_m(first_m:last_m);
DeltaS_m     = sm0_m-s0_m;

// apply the computed epsilon on all the prediction si = si - epsilon_m
smHorGlinCorr_m  = linearCorrection(smHorG_m,DeltaS_m, Np_m);
// 2nd method
smHorGlinCorr_incremental_m = ga_predHorGlobalMire((sm_m-DeltaS_m),Zm_m,Up_m,Te_m,Np_m);
// 3th method
smHorGlinCorr_from2D_m = cld_predHorGlobalMire((sm_m-DeltaS_m),Zm_m,Up_m,Te_m,Np_m);


disp('linear feature correction using 3 methods')
hf_12           = createPlanImage(12,xl_m,xu_m,"Correction of feature using DeltaS");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionMulticolor(Np_m,sHorG_m,'-');
mireEvolutionMulticolor(Np_m,smHorGlinCorr_m,'-.') ; // motion then projection
show_pixmap();
disp('motion then projection')
pause
mireEvolutionMulticolor(Np_m,smHorGlinCorr_incremental_m,'-') ; // incremental
show_pixmap();
disp('incremental')
pause
mireEvolutionMulticolor(Np_m,smHorGlinCorr_from2D_m,'o') ; // P estimation then motion then projection
disp('P estimation then motion then projection')
show_pixmap();
pause


// --- Epsilon = L^+(sm-s) / X correction means the pose cXo is directly corrected
L_m = matIntMireC(s_m,Z_m);

[DeltaX_m, cmMce_m]  = epsilonX(s0_m,sm0_m,L_m);

disp('cmMce_m directement comme une pause');
disp(cmMce_m);
//cmMce2_m = expMapDirectRxRyRz(epX_m',Te_m);
//disp('cmMce2_m directement comme une vitesse intgree');
//disp(cmMce2_m);

pause;

ceMo_m						= inv(cmMce_m) * cmMo_m; 
oMce_m						= inv (ceMo_m);

[seHorG_m, ce1MceN_m, Ze_m] = preHorGlobalMirePos(oP_m,ceMo_m,Up_m,Te_m,Np_m);
ceP_m         = changeFrameMire(oP_m,ceMo_m);
se_m          = projectMireDirect(ceP_m);
Ze_m          = ceP_m(3:3:$)  ;

// 2nd method
seHorG_incremental_m = ga_predHorGlobalMire(se_m,Ze_m,Up_m,Te_m,Np_m);
// 3th method
//seHorG_from2D_m = ga_predHorGlobalMire(se_m,Ze_m,Up_m,Te_m,Np_m);


pause

disp('Estimation of the correction feature using DeltaX')

hf_13           = createPlanImage(13,xl_m,xu_m,"Estimation of corrected features");
mire2DDraw(s_m,cote_m,1); // initial target position
mireEvolutionDraw(Np_m,sHorG_m,1,'-k');
//mireEvolutionDraw(Np_m,seHorG_m,1,'-k') ; // motion then projection
//show_pixmap();
//pause
mireEvolutionDraw(Np_m,seHorG_incremental_m,2,'-r') ; // incremental
//show_pixmap();
//pause
//mireEvolutionDraw(Np_m,seHorG_from2D_m,1,'-b') ; // P estimation then motion then projection
show_pixmap();
//pause



//---------Trajectory of 2D points
hf_1           = createPlanImage(5,xl_m,xu_m,"3 mire evolutions : real, mesured, estimated");
mire2DDraw(s_m,cote_m,1); // initial target position
mire2DDraw(sDes_m,cote_m,5); // desired target position
mireEvolutionDraw(Np_m,sHorG_m,1,'-k') ; // real target evolution
show_pixmap();
disp('real target evolution')
pause
mireEvolutionDraw(Np_m,smHorG_m,1,'-r') ; // estimated target evolution
disp('stimated target evolution')
show_pixmap();
pause
mireEvolutionDraw(Np_m,seHorG_m,1,'-g') ; // corrected target evolution epsilon = L^+(s-sm)
disp('corrected target evolution epsilon = L^+(s-sm)')
show_pixmap();
pause
mireEvolutionDraw(Np_m,smHorGlinCorr_from2D_m,1,'-b') ; // corrected target evolution epsilon = (s-sm)
disp('corrected target evolution epsilon = (s-sm)')
show_pixmap();
//pause
//mireEvolutionDraw(Np_m,smHorGlinCorr_m,2,'-.c') //static corrected
//show_pixmap();
//pause
//mireEvolutionDraw(Np_m,smHorGlinCorr_incremental_m,1,'.-m') ; //
//show_pixmap();
//pause


hf_2            = createFigure3D(1,"Camera Motion",1);
Mire3DDraw4pts(wP_m);

show_pixmap();
for i=1:40:Np_m
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
//mireEvolutionDraw(Np_m, seHorG,1,'-g');
//mireEvolutionDraw(Np_m,smHorGmotionCorr,1,'-b');

show_pixmap();

pause;

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

