//--------------------------------------------------//
// main program
//
// author Claire Dune
// 10/09/2013
// 
// This test compares the prediction models our/ Alliberts / Folios
//
//--------------------------------------------------//



//---------------------------------------------------------------------//
//        Problem Setting
//---------------------------------------------------------------------//
posecdesMo   = [0 0 1   0 0 0];             // desired pose target/object 
posecMo      = [0.5 0.5 2 0 -%pi/6 0];             // init pose target/object   
posewMo      = [0 0 0 %pi/2 0 0 ];          // pose of the target in the World Frame

Nbpts        = 4;
targetSize   = 0.2;                         // dimension of the target    
oP           = mire4points (targetSize);    // create the Npbts Points Target
      
lambda       = 0.1;                         // visual servo gain
iterMax      = 1000 ;                         // number of iteration of the loop  
Te           = 1/33;                        // time step
threshold    = 0.1;                         // visual servoing error threshold

//---------------------------------------------------------------------//
//        Create the associated homogeneous matrix
//---------------------------------------------------------------------//

// ------ Object pose
wMo         = homogeneousMatrixFromPos(posewMo);
wP          = changeFrameMire(oP,wMo);       // the Points in the World frame

// ------ First Camera Object Position
cMo         = homogeneousMatrixFromPos(posecMo);// pose target/object init   
wMc         = wMo*inv(cMo) ;                 // pose of the camera in the world frame
wMc         = wMc.*(abs(wMc)>1e-10);

// compute the init projection on the view
cP          = changeFrameMire(oP,cMo);       // target Points in the camera frame
s           = projectMireDirect(cP);           // projection of the target points in the image plane 
Z           = cP(3:3:$) ;                      // depth of the target points in the camera frame

// -------- Compute the desired pose
cDesMo      = (homogeneousMatrixFromPos(posecdesMo));
wMcDes      = wMo*inv(cDesMo);
wMcDes      = wMcDes.*(abs(wMcDes)>1e-10);
// compute the desired projection on the view
cDesP       = changeFrameMire(oP,cDesMo);   // desired target Points in the camera frame
sDes        = projectMireDirect(cDesP);       // desired target Points projection
ZDes        = cDesP(3:3:$) ;    // desired depth

//---------------------------------------------------------------------//
//        Create figures
//---------------------------------------------------------------------//
xl          = [-0.3;-0.3];
xu          = [0.3; 0.3];

//Image plane
cote        = 0.02;
hf_1        = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(s,cote,3);
show_pixmap()
mire2DDraw(sDes,cote,5);
show_pixmap()

// 3D view
wMcfirst    = wMc;
hf_2        = createFigure3D(2,"Camera Motion",2);
Camera3DDraw(0.3,wMcfirst);       // display the first camera
Camera3DDraw(0.3,wMcDes);         // display the desired camera
Mire3DDraw4pts(wP);
show_pixmap()


//-----------------------------------------------------------------//
//             Visual servoing
//-----------------------------------------------------------------//

// init norm to a great value
vsErrorNorm   = 100;
vsErrorThreshold = 0.001;
iter          = 0 ;
v             = zeros(1,6); // real camera montion
// store
velocity    = [];
norme       = [];
E           = [];
wMc         = wMo*inv(cMo); 
Xc          = [wMc(1,4)];
Yc          = [wMc(2,4)];
Zc          = [wMc(3,4)];
S           = [s];
SFolio      = [s];
S_VS        = [s];
Sb          = [s];
Sour        = [s];

Np          = 10;
t           = 1:Np;
t           = Te*t;
t0          = 0;
Xest        =0;

//INIT
s_vs = s ; // features courantes utilisees pour faire l'asservissement visuel
           // s est la feature reelle
Z_vs = Z; // profondeur utilisee pour calculer l'asservissement visuel
cm1Mcm2 = eye(4,4);
oMcm    = inv(cMo);


//Au choix
FlagFolio    = 0;
FlagAllibert = 0;
FlagOur      = 1;

// Option
FlagOcclusion=1;
FlagNoise=0;

while(iter<1000&vsErrorNorm>vsErrorThreshold)
  
  iter = iter+1;
  disp('------------')
  disp(iter)
  disp(vsErrorNorm')
  
  
  vsError     = s_vs-sDes; //s visual servoing
  vsErrorNorm = norm(vsError);
  L           = matIntMireC(s_vs,Z_vs);
  v           = -lambda * pinv(L) * (vsError)  ;
  
  
   // Dans tous les cas, on calcule quand meme la vraie position des features et de la camera
   cpMc        = computeMotion(v,Te);
   cMo         = inv(cpMc)*cMo ;
   oMc			   = inv(cMo);
   poseoMc     = pFromHomogeneousMatrix(oMc)';   
   wMc         = wMo*inv(cMo); 
   cP          = changeFrameMire(oP,cMo);
   s           = projectMireDirect(cP);
   Z           = cP(3:3:$) ;
   
  

   
   // if we want to had noise on the data
   if(FlagNoise)  
     // on ajoute individuellement des bruits sur s et Z
     sb        = s+rand(s, 'normal')/1000;
     Zb        = Z+rand(Z, 'normal')/1000;
   else
     sb        = s;
     Zb        = Z;
   end
   

  
  // from previous s, v and Z, estimate the new s
  //if(iter>10&iter<100)
    //FlagOcclusion=1; // on ne voit plus rien
  //else
    //FlagOcclusion=0; // on revient a du classique
  //end

  
  //------------- Folios-------------------//
  //on fait tout le temps l'estimation
   X          = [s_vs;Z_vs];
   //Xest =X;
   Xest        = ode("stiff",X,t0,t,list(vsOde,v));
   //Xest      = ode("adams",X,t0,t,list(vsOde,v));
   N = length(Xest(:,1))/3;
   sFolio      = Xest(1:2*N,1);
   ZFolio      = Xest(2*N+1:$,1);
  //---------------------------------------//
   
   
   if(FlagOcclusion==0)
     "On evalue ceMo"
     
     //----------- Our ---------------------//
     // on ajoute toujours un bruit sur la pause lue //on ajoute juste un bruit sur la pose 
     poseoMcb    = poseoMc+rand(poseoMc,'normal')/1000;	
     oMcb        = homogeneousMatrixFromPos(poseoMcb);   
     cbMo        = inv(oMcb);
     // on lit en entree la pose bruitee
     poseoMcm    = poseoMcb;	
     oMcm        = homogeneousMatrixFromPos(poseoMcm);
     cmMo        = inv(oMcm);
     cmP         = changeFrameMire(oP,cmMo);
     sm          = projectMireDirect(cmP);
     Zm          = cmP(3:3:$) ;

     // et on calcul la correction
     Ls          = matIntMireC(sb,Zb);
     Lx          = computeLxInCam(cmMo);
     DeltaX      = pinv(Ls*Lx)*(sm-sb);
     poseoMce      = poseoMcm - DeltaX';
     oMce          = homogeneousMatrixFromPos(poseoMce);

   end
   
   ceMo          = inv(oMce);
   Up            = computeControlOnHorizon(v,1,Np);
   [sHorG c1McN ZG] = preHorGlobalMirePos(oP,ceMo,Up,Te,Np);
   sG            = sHorG(1:2*Nbpts);

   // on fait evoluer la pose avec notre estimation
   cm1Mcm2       = c1McN(1:4,:);
   oMce          = oMce*cm1Mcm2;
  
  //---------------------------------------//
   if(FlagFolio)
    soc  = sFolio;
   elseif(FlagOur)
    soc  = sG;
   end
  
   // on a perdu s et on est oblige d'estimer les features.  
   if(FlagOcclusion)     
     s_vs       = soc;
     Z_vs       = Zoc ;
    else
     s_vs       = sb ;
     Z_vs       = Zb ; 
   end 
 
  
  // store value for display
  velocity    = [velocity;v'];
  norme       = [norme; vsErrorNorm];
  E           = [E vsError]; 
  Xc          = [Xc wMc(1,4)];
  Yc          = [Yc wMc(2,4)];
  Zc          = [Zc wMc(3,4)];
  S           = [S s]; 
  S_VS        = [S_VS s_vs];
  SFolio      = [SFolio  sFolio]; 
  Sb          = [Sb sb];

  // display
 
  
  
  //TODO : regarder la position finale obtenue.
  
end






xu          = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl          = [ -0.22 ; -0.22 ];                 // position min of the a 2D point in the image plane 
 // --- Displays--------------------------------------//
  xset("window",1); 
  hf_1      = createPlanImage(1,xl,xu,"Point 2D");
  if (iter>2)
      for i=1:Nbpts
        plot(S((i-1)*2+1,:),S((i-1)*2+2,:),'k')
        plot(S_VS((i-1)*2+1,:),S_VS((i-1)*2+2,:),'r')
        plot(SFolio((i-1)*2+1,:),SFolio((i-1)*2+2,:),'b')
        plot(Sb((i-1)*2+1,:),Sb((i-1)*2+2,:),'k-.')

      end  
  end
  show_pixmap();
  pause;
   
  hf_2            = createFigure3D(2,"Camera Motion",2);
  Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
  Camera3DDrawColor(0.1,wMc,3);// display the first camera
  Camera3DDrawColor(0.1,wMcDes,5);// display the first camera
  Mire3DDraw4pts(wP);
  if (iter>=2)
    plot3d(Xc,Yc,Zc);
  end
  Camera3DDraw(0.1,wMc);
  show_pixmap()
  
  
  xset("window",3);
  clf 
      hf=scf(3);
  hf.figure_name = "error";
  Thres = vsErrorThreshold*ones(length(norme),1);
  if(iter>2)
  plot(norme,'g-');
  plot(E','b');
  plot(Thres,'r-');

  end 
  
pause;
  xset("window",4);
    hf=scf(4);
  hf.figure_name = "velocity";
if(size(velocity,1)>1)
    plot(velocity(:,1),'r-');
    plot(velocity(:,2),'g-');
    plot(velocity(:,3),'b-');
    plot(velocity(:,4),'r-.');
    plot(velocity(:,5),'g-.');
    plot(velocity(:,6),'b-.');
    show_pixmap()
end





  
//-----------------------------------------------------------------//
//             FORMERIROS Code
//-----------------------------------------------------------------//
//Nc = 1;
//Np = 200;
//v  = [0; 0; 0.1; 0; 0 ;0.3];
//v  = [0.01 0.01 0.1 0.02 0 0.03]';
//Te = 0.03;

//// --- Test de dplacement
//cprecMcnext = computeMotion(v',Te);
//
//// --- repeat the control Np times on the control horizon
//Up          = computeControlOnHorizon(v,Nc,Np);
//
//// --- compute the position of the camera over the time horizon
//[sHorGmotion c1McN Zmotion] = preHorGlobalMirePos(oP,cMo,Up,Te,Np);
//first       = 1;
//last        = 2*NbPts;
//s0          = sHorGmotion(first:last);
//
//
//// --- same for the estimated camera
//[smHorGmotion cm1McmN Zmmotion] = preHorGlobalMirePos(oP,cmMo,Up,Te,Np);
//sm0         = smHorGmotion(first:last);
//epsilon     = sm0-s0;
//smHorGmotionCorr  = linearCorrection(smHorGmotion,epsilon, Np);
//L           = matIntMireC(s,Z);
//[epX cmMc]  = epsilonX(s0,sm0,L);
//
//// ---- corrected 
//ceMo          = inv(cmMc)*cmMo; 
//[oP ceP se]   = mire4pointsInCam(0.20,ceMo); // build the 3d points in the object and in the camera frame
//Ze            = ceP(3:3:$); // compute the point depth
//
//[seHorG ce1MceN Ze] = preHorGlobalMirePos(oP,ceMo,Up,Te,Np);
//
//
//// --- compute the feature position using the local model sk+1 = sk+ TeLv 
//global computeL_global;
//computeL_global = matIntMireC; // Z and S current
//smHorLc         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
//epsilon         = smHorLc((first:last))-s0;
//smHorLcCorr     = linearCorrection(smHorLc,epsilon, Np);
//
//
//computeL_global = matIntMireD; // Z and S desired
//smHorLd         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
//epsilon         = smHorLd((first:last))-s0;
//smHorLdCorr     = linearCorrection(smHorLd,epsilon, Np);
//
//computeL_global = matIntMireM; // Mixte Ld and Lc
//smHorLm         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
//epsilon         = smHorLm((first:last))-s0;
//smHorLmCorr     = linearCorrection(smHorLm,epsilon, Np);
//
//computeL_global = matIntMireP; // S current and Z desired
//smHorLp         = ga_predHorLoc2dMire(sm,Zm,Up,Te,Np);
//epsilon         = smHorLp((first:last))-s0;
//smHorLpCorr     = linearCorrection(smHorLp,epsilon, Np);
//
//// --- compute the feature position using the global model s(x)
//smHorG = ga_predHorGlobalMire(sm,Zm,Up,Te,Np);
//
//// --- keep L constant
//smHorConst      = predHorLoc2dMireConst(sm,Zm,Up,Te,Np);
//epsilon         = smHorConst((first:last))-s0;
//smHorConstCorr  = linearCorrection(smHorConst,epsilon, Np);
//
//// --- compute a false representation by modifying Z : 
////Zmod = Z + rand(Z)/10;
////smHorGmod = ga_predHorGlobalMire(s,Zmod,Up,Te,Np);
//
//
//
////------create  figures
//
//// image plane
//xl             = [-0.3;-0.3] ;
//xu             = [0.3; 0.3] ;
//cote           = 0.01 ;
//hf_1           = createPlanImage(2,xl,xu,"Point 2D trajectory local");
//mire2DDraw(s,cote,4);
//mireEvolutionDraw(Np,smHorLc,1,'-b') ;
//mireEvolutionDraw(Np,smHorLd,1,'-g') ;
//mireEvolutionDraw(Np,smHorLm,1,'-r') ;
//mireEvolutionDraw(Np,smHorLp,1,'-c') ;
//mireEvolutionDraw(Np,smHorConst,1,'-m')
//mireEvolutionDraw(Np,smHorG,1,'-k') ;
//show_pixmap();
//
//
////---------Correction
////hf_5           = createPlanImage(5,xl,xu,"Corrected Point 2D trajectory local");
//mire2DDraw(s,cote,4);
//mireEvolutionDraw(Np,smHorLcCorr,1,'-.b') ;
//mireEvolutionDraw(Np,smHorLdCorr,1,'-.g') ;
//mireEvolutionDraw(Np,smHorLmCorr,1,'-.r') ;
//mireEvolutionDraw(Np,smHorLpCorr,1,'-.c') ;
//mireEvolutionDraw(Np,smHorConstCorr,1,'-.m')
//show_pixmap();
//
//
//
//
////hf_1           = createPlanImage(3,xl,xu,"Point 2D trajectory global");
////mireEvolutionDraw(Np,smHorG,1,'-b') ;
////show_pixmap();
//
//hf_2            = createFigure3D(3,"Camera Motion",1);
////CameraPredDraw(0.1,Up,Te,Np,cMo,3);
//cMcm = inv(cmMc);
//for i=1:Np/20:Np
//    c1Mc2        = c1McN(((i-1)*4+1:(i-1)*4+4),:)  ;//noir         // resulting motion
//    oMc_display  = oMc*c1Mc2;
//    Camera3DDrawColor(0.05,inv(oMc_display),1);
//
//    cm1Mcm2      = cm1McmN(((i-1)*4+1:(i-1)*4+4),:)  ;         // resulting motion
//    oMcm_display  = oMcm*cm1Mcm2;
//    Camera3DDrawColor(0.05,inv(oMcm_display),5); //rouge
//     
//        
//    oMcest_display  = oMcm*inv(cMcm)*cm1Mcm2;
//    Camera3DDrawColor(0.05,inv(oMcest_display),3); //vert  
//end
//    show_pixmap();
//
//hf_1           = createPlanImage(4,xl,xu,"Point 2D trajectory global and noisy");
//mire2DDraw(s,cote,4);
//mireEvolutionDraw(Np,sHorGmotion,1,'-k') ;
//mireEvolutionDraw(Np,smHorGmotion,1,'-r') ;
//mireEvolutionDraw(Np, seHorG,1,'-g');
//mireEvolutionDraw(Np,smHorGmotionCorr,1,'-b');
//
//show_pixmap();



//-----------------------------------------------------------------//
//             Finish the process
//-----------------------------------------------------------------//

disp('-------The End------')
xset("pixmap",0);
disp('pause before ending')
pause



