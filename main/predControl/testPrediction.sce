//--------------------------------------------------//
// main program
//
// author Claire Dune
// date 07/07/2011
// 
// This test compares the prediction models
// We test several epsilon and compare the corrections.
//
//--------------------------------------------------//

//---------------------------------------------------------------------//
//        Problem Setting
//---------------------------------------------------------------------//

wMo        = homogeneousMatrixFromPos([0 0 0 0 %pi 0]); // convert pose vector into homogeneous matrix

// ------ First Camera Object Position
posecMo       = [0 0 4 0 0 0 ]; 	
cMo           = homogeneousMatrixFromPos(posecMo); // convert pose vector into homogeneous matrix
oMc           =inv(cMo);
NbPts         = 4;
[oP cP s]     = mire4pointsInCam(0.20,cMo); // build the 3d points in the object and in the camera frame
Z             = cP(3:3:$); // compute the point depth


// ------ Estimated First pose = first pose + perturbation
//disturbance     = rand(posecMo,'normal')/10;
//disturbance   = [-0.0068342  -0.0072095 0.0081451 0.0032402 -0.0018848 0.0042416 ];
disturbance   = [0.1 0.1 0.1 0 0 0]
cmMc          = homogeneousMatrixFromPos(disturbance); 
cmMo          = cmMc*cMo;
oMcm          = inv(cmMo)
[oP cmP sm]   = mire4pointsInCam(0.20,cmMo); // build the 3d points in the object and in the camera frame
Zm             = cmP(3:3:$); // compute the point depth


//------- desired position
global Zdes_global;
global sdes_global;

posecdesMo    = [0.1 0.1 1 0.3 0.5 0.6 ]; 	
cdesMo        = homogeneousMatrixFromPos(posecdesMo); // convert pose vector into homogeneous matrix
[oP cdesP sdes]  = mire4pointsInCam(0.20,cdesMo); // build the 3d points in the object and in the camera frame
Zdes_global   = cdesP(3:3:$); // compute the point depth
sdes_global   = sdes;
//-----------------------------------------------------------------//
//             Point trajectory test
//-----------------------------------------------------------------//
Nc = 1;
Np = 200;
v  = [0; 0; 0.1; 0; 0 ;0.3];
//v  = [0.01 0.01 0.1 0.02 0 0.03]';
Te = 0.03;

// --- Test de dplacement
cprecMcnext = computeMotion(v',Te);

// --- repeat the control Np times on the control horizon
Up          = computeControlOnHorizon(v,Nc,Np);

// --- compute the position of the camera over the time horizon
[sHorGmotion c1McN Zmotion] = preHorGlobalMirePos(oP,cMo,Up,Te,Np);
first       = 1;
last        = 2*NbPts;
s0          = sHorGmotion(first:last);


// --- same for the estimated camera
[smHorGmotion cm1McmN Zmmotion] = preHorGlobalMirePos(oP,cmMo,Up,Te,Np);
sm0         = smHorGmotion(first:last);
epsilon     = sm0-s0;
smHorGmotionCorr  = linearCorrection(smHorGmotion,epsilon, Np);
L           = matIntMireC(s,Z);
[epX cmMc]  = epsilonX(s0,sm0,L);

// ---- corrected 
ceMo          = inv(cmMc)*cmMo; 
[oP ceP se]   = mire4pointsInCam(0.20,ceMo); // build the 3d points in the object and in the camera frame
Ze            = ceP(3:3:$); // compute the point depth

[seHorG ce1MceN Ze] = preHorGlobalMirePos(oP,ceMo,Up,Te,Np);


// --- compute the feature position using the local model sk+1 = sk+ TeLv 
global computeL_global;
computeL_global = matIntMireC; // Z and S current
smHorLc         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
epsilon         = smHorLc((first:last))-s0;
smHorLcCorr     = linearCorrection(smHorLc,epsilon, Np);


computeL_global = matIntMireD; // Z and S desired
smHorLd         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
epsilon         = smHorLd((first:last))-s0;
smHorLdCorr     = linearCorrection(smHorLd,epsilon, Np);

computeL_global = matIntMireM; // Mixte Ld and Lc
smHorLm         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
epsilon         = smHorLm((first:last))-s0;
smHorLmCorr     = linearCorrection(smHorLm,epsilon, Np);

computeL_global = matIntMireP; // S current and Z desired
smHorLp         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
epsilon         = smHorLp((first:last))-s0;
smHorLpCorr     = linearCorrection(smHorLp,epsilon, Np);

// --- compute the feature position using the global model s(x)
smHorG = ga_predHorGlobalMire(sm,Zm,Up,Te,Np);

// --- keep L constant
smHorConst      = predHorLoc2dMireConst(sm,Zm,Up,Te,Np);
epsilon         = smHorConst((first:last))-s0;
smHorConstCorr  = linearCorrection(smHorConst,epsilon, Np);

// --- compute a false representation by modifying Z : 
//Zmod = Z + rand(Z)/10;
//smHorGmod = ga_predHorGlobalMire(s,Zmod,Up,Te,Np);



//------create  figures

// image plane
xl             = [-0.3;-0.3] ;
xu             = [0.3; 0.3] ;
cote           = 0.01 ;
hf_1           = createPlanImage(2,xl,xu,"Point 2D trajectory local");
mire2DDraw(s,cote,4);
mireEvolutionDraw(Np,smHorLc,1,'-b') ;
mireEvolutionDraw(Np,smHorLd,1,'-g') ;
mireEvolutionDraw(Np,smHorLm,1,'-r') ;
mireEvolutionDraw(Np,smHorLp,1,'-c') ;
mireEvolutionDraw(Np,smHorConst,1,'-m')
mireEvolutionDraw(Np,smHorG,1,'-k') ;
show_pixmap();


//---------Correction
//hf_5           = createPlanImage(5,xl,xu,"Corrected Point 2D trajectory local");
mire2DDraw(s,cote,4);
mireEvolutionDraw(Np,smHorLcCorr,1,'-.b') ;
mireEvolutionDraw(Np,smHorLdCorr,1,'-.g') ;
mireEvolutionDraw(Np,smHorLmCorr,1,'-.r') ;
mireEvolutionDraw(Np,smHorLpCorr,1,'-.c') ;
mireEvolutionDraw(Np,smHorConstCorr,1,'-.m')
show_pixmap();




//hf_1           = createPlanImage(3,xl,xu,"Point 2D trajectory global");
//mireEvolutionDraw(Np,smHorG,1,'-b') ;
//show_pixmap();

hf_2            = createFigure3D(3,"Camera Motion",1);
//CameraPredDraw(0.1,Up,Te,Np,cMo,3);
cMcm = inv(cmMc);
for i=1:Np/20:Np
    c1Mc2        = c1McN(((i-1)*4+1:(i-1)*4+4),:)  ;//noir         // resulting motion
    oMc_display  = oMc*c1Mc2;
    Camera3DDrawColor(0.05,inv(oMc_display),1);

    cm1Mcm2      = cm1McmN(((i-1)*4+1:(i-1)*4+4),:)  ;         // resulting motion
    oMcm_display  = oMcm*cm1Mcm2;
    Camera3DDrawColor(0.05,inv(oMcm_display),5); //rouge
     
        
    oMcest_display  = oMcm*inv(cMcm)*cm1Mcm2;
    Camera3DDrawColor(0.05,inv(oMcest_display),3); //vert  
end
    show_pixmap();

hf_1           = createPlanImage(4,xl,xu,"Point 2D trajectory global and noisy");
mire2DDraw(s,cote,4);
mireEvolutionDraw(Np,sHorGmotion,1,'-k') ;
mireEvolutionDraw(Np,smHorGmotion,1,'-r') ;
mireEvolutionDraw(Np, seHorG,1,'-g');
mireEvolutionDraw(Np,smHorGmotionCorr,1,'-b');

show_pixmap();



//-----------------------------------------------------------------//
//             Finish the process
//-----------------------------------------------------------------//

disp('-------The End------')
xset("pixmap",0);
disp('pause before ending')
pause



