// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');
function testExp()
  disp('Yes');
endfunction

function testSwayCorrection3DExpM()
  // -- Pose init and final of the robot in the world frame
  h              = 0.8;               // CoM height
  g              = 9.81;              // gravity
  maxiter        = 200;               //
  poserInit      = [0 0 h 0 0 0]; // CoM init pose in the world reference frame
  poserFinal     = [1 0.8 h 0 0 %pi/6]; // CoM final pose in the world reference frame
  poser          = poserInit;

  // ----- For Walking
  OPT_WALK_SIMU  = %T;               // sinusoidal addition
  OPT_WALK_HRP2  = %F;               // walk simulation 

  // -- For Simulation
  dt             = .1;
  cote           = .01;              // size of the target dot

  // -- For Visual Servoing
  lambda         = 0.4;               // aservissement gain
  threshold      = .001 ;             // threshold on the error to stop


 //---------AUTO-SET -----//

  if(OPT_WALK_HRP2)
    setActuationSamplingPeriod(2.5e-3);
    setActuationSamplingDelay(0.1e-3);
    setNbActuators(30);
    DOUBLE_SUPPORT = 2; 
    PERIOD=16;
  else
    DOUBLE_SUPPORT = 0;
    PERIOD=16;
  end

  //-------- Matrix Position -------------//

  wMrinit        = homogeneousMatrixFromPos(poserInit);
  wMr            = wMrinit; 
  wMrdes         = homogeneousMatrixFromPos(poserFinal);
  [rMc,rVc]      = loadHRP2camCom();
  wMc            = wMr*rMc;
  wMcdes         = wMrdes*rMc;

  // set to 0 the values that are < small
  tooSmall       = 1e-10;
  wMc            = wMc.*(abs(wMc)>tooSmall);
  wMcdes         = wMcdes.*(abs(wMcdes)>tooSmall);
  wMcfirst       = wMc;
  wMrfirst       = wMr;

  cdesMc         = inv(wMcdes)*wMc;
  p              = tUFromHomogeneousMatrix(cdesMc);
  pcorr          = p;   
  


  //----------------- VISUAL SERVOING LOOP -----------------//  
  Lfunction   = matIntposeThetaU;
  er          = 10; // error value init
  iter        = 0;
  vcam        = zeros(1,6);
  vcamReal    = zeros(1,6);
  mu          = 0.8;

  Xc          = [ wMc(1,4)];
  Yc          = [ wMc(2,4)];
  Zc          = [ wMc(3,4)];
  Xcv         = [ wMc(1,4)];
  Ycv         = [ wMc(2,4)];
  Zcv         = [ wMc(3,4)];
  Xr          = [ wMr(1,4)];
  Yr          = [ wMr(2,4)];
  Zr          = [ wMr(3,4)];
  zmpx        = [];
  zmpy        = [];
  // velocity
  VDes        = [];
  VReal       = [];
  VRobotReal  = [];
  VRobotDes   = [];
  // features
  P           = [];
  Pcorr       = [];
  E           = [];
  Ecorr       = [];
  normE       = [];
  normEcorr   = [];
  ncPvc       = [];
  INTEGRALE   = [];
  PSWAINT     = [];
  NCMVC       = [];
  
  //Andrei
  //state of the robot [x;dx;ddx;y;dy;ddy;theta;dtheta;ddtheta]
  RobotReal      = zeros(9,1);
  //we must enter in robot init the value of pose init
  RobotReal(1)   = poserInit(1) ;  
  RobotReal(4)   = poserInit(2) ;
  RobotReal(7)   = poserInit(6) ;
  k = 0;
  //Andrei
  warming        = PERIOD+DOUBLE_SUPPORT;
  L              = matIntposeThetaU(p);
  Lcourant       = L;
  STOP_CRITERION = %F;
  vcam = zeros(1,6);
  vrobotprevious = zeros(6,1);
  vrobotReal = zeros(6,1);
  wVr            = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
  ncMvc          = eye(4,4);
  pswayInt       = zeros(6,1);
  meanint        = pswayInt;
//----------------------------------------------------------------------//
// Servo LOOP
  while( iter < maxiter & STOP_CRITERION==%F)
    iter = iter+1;
    printf('------------------------%d\n',iter)
  
    wMvc         = wMc*ncMvc;
    ncpvc        = pFromHomogeneousMatrix(ncMvc);
    cdesMvc       = inv(wMcdes)*wMvc;
    pcorr         = tUFromHomogeneousMatrix(cdesMvc);
    Lcourant      = matIntposeThetaU(pcorr);       // compute the interaction matrix
       
    //------------------------------------------------------------//
    // compute the camera velocity
    lambda=0.2;
    pcorr = p -pswayInt+meanint;
    vcam  = (-lambda *pinv( Lcourant)*pcorr)';
    vcam  = vcam.*(abs(vcam) > 1e-6);
  
    //------------------------------------------------------------//
    // change velocity frame
    vrobot       = rVc*vcam';

    //---------------------------------------------------------//
    // Approximation of what andrei code should do
    // input is the robot relative velocity
    // output is the robot velocity in the world frame
    // vrobotReal = [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
    // the real velocity is not vcam but something based on vDes
  
    //---------- Add sinus -------------------------------------//
    if (OPT_WALK_SIMU)
      vrobotReal   = vrobot;
      vrobotReal(2)= vrobot(2)+0.1*sin(%pi/(PERIOD/2)*(iter-1)) ;
      vrobotRealW  = wVr*vrobotReal;
      
    //------------Andrei Code -------------------------------------//
    elseif (OPT_WALK_HRP2) 
      [RobotReal,wMr,vrobotReal] = runPG(vrobot,RobotReal,k,wMrinit);
       k = k + 1;
      wVr          = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
      vrobotRealW  = wVr*vrobotReal;
    
    else
      vrobotReal  = vrobot;
      vrobotRealW = wVr*vrobotReal;
    end
  
    //--------------------------------------------------------------//
    // deduce vcamReal 
    vcamReal   = inv(rVc)*vrobotReal;
    vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);

    // ---- Control error-----
    
    
    // compute the real motion and the virtual motion
    vcMc       = expMapDirectRxRyRz(vcam,dt);
    vcpc        = pFromHomogeneousMatrix(vcMc);
    disp('pose cpvc');
    disp(vcpc');
    wMvc       = wMc*cMvc;
    // compute the next expected motion 
    ncMc       = expMapDirectRxRyRz(vcamReal',dt);
    ncpc       = pFromHomogeneousMatrix(ncMc);
    disp('posencpc');
    disp(ncpc');
    
   
    wMnc       = wMc*inv(ncMc);
    // deduce the expected pose error
    ncMvc_new  = inv(cMnc)*cMvc;
    ncpvc_new  = pFromHomogeneousMatrix(ncMvc_new);
    disp('posencpvc courant');
    disp(ncpvc');
    ncMvc      = ncMvc*ncMvc_new;
    ncpvc  = pFromHomogeneousMatrix(ncMvc);
    disp('posencpvc cumul');
    disp(ncpvc');
    
    balancement = vcamReal'-vcam;
    disp('balancement courant')
    psway      = dt*Lcourant*balancement';// instant velocity of points
    disp(psway)
    
   
    pswayInt   = pswayInt + psway;
    disp('balancement cumul');
    disp(pswayInt);

    meanint =  mean(INTEGRALE,2);
    PSWAINT     = [PSWAINT pswayInt-meanint];
    INTEGRALE   = [INTEGRALE pswayInt];


    
    NCMVC       = [NCMVC ncpvc];


    // position reelle
    r1Mr2      = expMapDirectRxRyRz(vrobotReal',dt);
    wMr        = wMr*r1Mr2;
    wMr        = wMr.*(abs(wMr)>1e-10);
    wMc        = wMr*rMc;

    // feature
    cdesMc     = inv(wMcdes)*wMc;
    p          = tUFromHomogeneousMatrix(cdesMc);
    //pause;
    //----------------------------------------------------//
    // update the position    
    //save the 2D position
    ncPvc      = [ncPvc ncpvc];
    Ecorr      = [Ecorr pcorr];  
    normE      = [normE;norm(pcorr)];
    E          = [E p];  
    normE      = [normE;norm(p)];
    VDes       = [VDes;vcam];
    VReal      = [VReal;vcamReal'];
    VRobotDes  = [VRobotDes;vrobotReal([1;2;6])'];
    VRobotReal = [VRobotReal;vrobot([1;2;6])'];
    P          = [P p]; //preal
    Pcorr      = [P pcorr]; // pcorrected
    poser      = pFromHomogeneousMatrix(wMr);
    posec      = pFromHomogeneousMatrix(wMc); 
    posecv     = pFromHomogeneousMatrix(wMvc);
    Xcv        = [Xcv;posecv(1)];
    Ycv        = [Ycv;posecv(2)]; 
    Zcv        = [Zcv;posecv(3)];
    Xc         = [Xc;posec(1)];
    Yc         = [Yc;posec(2)]; 
    Zc         = [Zc;posec(3)];
    Xr         = [Xr;poser(1)];
    Yr         = [Yr;poser(2)];
    Zr         = [Zr;poser(3)];
    //XPGr       = [XPGr;RobotReal(1)];
    //YPGr       = [YPGr;RobotReal(4)];
    //ZPGr       = [ZPGr;RobotReal(7)];
    zmpx       = [zmpx;RobotReal(1)-h*RobotReal(3)/g];
    zmpy       = [zmpy;RobotReal(4)-h*RobotReal(6)/g];  

  end

  disp('End of the servo loop')
  hf=scf(10);
  hf.figure_name = "position with exp map" ;
  //plot(NCMVC(1,:)');
  plot(NCMVC');
  show_pixmap();    
  hf=scf(11);
  hf.figure_name = "position with difference" ;
  //plot(PSWAINT(1,:)');
  plot(PSWAINT');
  show_pixmap();   

  disp('Display the camera position')
  xset("window",2);
  clf 
  hf_2         = createFigure3D(20,"Camera Motion",2);
  Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
  if (iter>=2)
    plot3d(Xc,Yc,Zc);
    plot3d(Xr,Yr,Zr);
    plot3d(Xcv,Ycv,Zcv);
  end
  show_pixmap()
  
  
  disp('Display the velocities')
  xset("window",5);
  clf 
  hf=scf(5);
  hf.figure_name = "velocity";
  if(size(VDes,1)>1)
    plot(VDes(:,1),'r-.');
    plot(VDes(:,2),'g-.');
    plot(VDes(:,3),'b-.');
    plot(VDes(:,4),'c-.');
    plot(VDes(:,5),'m-.');
    plot(VDes(:,6),'k-.');
    
    plot(VReal(:,1),'r');
    plot(VReal(:,2),'g');
    plot(VReal(:,3),'b');
    plot(VReal(:,4),'c');
    plot(VReal(:,5),'m');
    plot(VReal(:,6),'k');
    show_pixmap()
  end



  disp('Pause before ending');
  pause;
endfunction

function isExpSet()
  disp('testExpMap is set');
  
  endfunction

