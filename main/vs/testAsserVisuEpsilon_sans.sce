//--------------------------------------------------//
// main program
//
// author Claire Dune
// date 10/04/2011
// ;exec('testAsserVisuEpsilon.sce');
//
//--------------------------------------------------//

//---------------------------------------------------------------------//
//        Problem Setting
//---------------------------------------------------------------------//

posecdesMo   = [0 0 1   0 0 0];             // desired pose target/object 
posecinitMo  = [0.5 0.5 2 0 -%pi/6 0];             // init pose target/object   
posewMo      = [0 0 0 %pi/2 0 0 ];          // pose of the target in the World Frame
posecMo      =	posecinitMo;

Nbpts        = 4;
targetSize   = 0.2;                         // dimension of the target    
oP           = mire4points (targetSize);    // create the Npbts Points Target
   
   
lambda       = 0.1;                         // visual servo gain
iterMax      = 1000 ;                         // number of iteration of the loop  
Te           = 1/33;                        // time step
threshold    = 0.1;                         // visual servoing error threshold

//---------------------------------------------------------------------//
//         Create the cameras and the object
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
p           = projectMireDirect(cP);
      




//------- Desired Camera Object Position 
cDesMo      = (homogeneousMatrixFromPos(posecdesMo));
wMcDes      = wMo*inv(cDesMo);
wMcDes      = wMcDes.*(abs(wMcDes)>1e-10);
// compute the desired projection on the view
cDesP       = changeFrameMire(oP,cDesMo);   // desired target Points in the camera frame
sDes        = projectMireDirect(cDesP);       // desired target Points projection
ZDes        = cDesP(3:3:$) ;    // desired depth



//------create  figures
xl             = [-0.3;-0.3]
xu             = [0.3; 0.3]

//Image plane
cote           = 0.1;
hf_1           = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(s,cote,3);
show_pixmap()
mire2DDraw(sDes,cote,5);
show_pixmap()

// 3D view
wMcfirst       = wMc;
hf_2           = createFigure3D(2,"Camera Motion",2);
Camera3DDraw(0.3,wMcfirst);       // display the first camera
Camera3DDraw(0.3,wMcDes);         // display the desired camera
Mire3DDraw4pts(wP);
show_pixmap()





//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction   = matIntMireC;
vcam        = zeros(1,6); // real camera montion
er          = 10; // error value init
iter        = 0;
mu          = 0.8;
L           = Lfunction(p,Z); 

Xc          = [ wMc(1,4)];
Yc          = [ wMc(2,4)];
Zc          = [ wMc(3,4)];
S           = [ s];
velocity    = [];
norme       = [];
E           = [];

// launch the servo
while( iter < iterMax)

  iter = iter+1;
  disp('--------')
  disp(iter)
  
  // compute feature
  cP         = changeFrameMire(oP,cMo);                    // compute the 3D points
  s          = projectMireDirect(cP);                      // compute the 2D points
  Z          = cP(3:3:$);                                  // compute Z
  
  // compute error
  e          = s-sDes;                                       
  norme      = [norme;norm(e)];  
  
  // compute the interaction matrix
  L          = Lfunction(s,Z);                             
  
  // compute the control law
  vcam       = - lambda *pinv(L)*e;
  vcam       = vcam';
  // compute displacement c1Mc2
  c1Mc2      = expMapDirectRxRyRz(vcam,Te);
  cMo        = inv(c1Mc2)*cMo;
  wMc        = wMo*inv(cMo);
      
 //save data for display
  S          = [S s]; 
  posec      = pFromHomogeneousMatrix(wMc);
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];
  E          = [E e];  
  velocity   = [velocity;vcam]   ;
    
end


  
  // --- Displays--------------------------------------//
  xset("window",1); 
  hf_1           = createPlanImage(1,xl,xu,"Point 2D");
  mire2DDraw(sDes,cote*2,5);
  if (iter>2)
      for i=1:Nbpts
        plot(S((i-1)*2+1,:),S((i-1)*2+2,:),'r')
      end  
  end
  show_pixmap();
  
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
  
  xset("window",4);
  clf 
  Thres = threshold*ones(length(norme),1);
  
  if(iter>2)
  plot(norme,'g');
  plot(E','g-.')
  end 
  
  

  
  xset("window",5);
    clf 

  
  if(size(velocity,1)>1)
    plot(velocity(:,1),'r-');
    plot(velocity(:,2),'g-');
    plot(velocity(:,3),'b-');
    plot(velocity(:,4),'r-.');
    plot(velocity(:,5),'g-.');
    plot(velocity(:,6),'b-.');
    show_pixmap()
  end



 

  xset("window",7);
    clf 
  if(size(E,1)>1)
    plot2d(E');
    show_pixmap()
  end 
 
  

disp('-------The End------')

xset("pixmap",0);
disp('pause before ending')
pause



