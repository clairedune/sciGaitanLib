// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touchent le sol.


// posture 
q1 = 10*%pi/180 ;
q2 = -40*%pi/180 ;
q3 = 40*%pi/180 ;
q4 = 110*%pi/180 ;
q5 = -30*%pi/180 ;
q6 = 90*%pi/180-(q1+q2+q3+q4+q5);//+5*%pi/180;
q = [q1,q2,q3,q4,q5,q6];

//-------- Parametres du robot ------- //

d = humanWalkerSagSeg();
masse = 70;



// ----------------- // 
// Calcul du MGD 
//-------------------//
M_base0 = homogeneousMatrixFromPos([0 0 0 0 -%pi/2 0]); // expression du premier repere dans le monde
[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);
M_02 = M_01*M_12;
M_03 = M_01*M_12*M_23;
M_04 = M_01*M_12*M_23*M_34;
M_05 = M_01*M_12*M_23*M_34*M_45;
M_06 = M_01*M_12*M_23*M_34*M_45*M_56;


// ----------------- // 
// Calcul des positions du centre d'inertie
//-------------------//
[PI,COM, m] = humanWalkerSagInertia(q,d,masse);

// ----------------- // 
// Affichage 
//-------------------//
  
// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);



//trac de la figure
xset("window",2);
xset("pixmap",1);
clear_pixmap()//et buffer
h1=scf(2);
h1.figure_name = "Sagital plane";
ha=h1.children;
ha.box="on"; 
ha.view="2d";
ha.thickness=1;
ha.foreground=0;
axe=gca(); // recupere un pointeur sur les axes
axe.data_bounds=[[-0.1 2]'; [-0.1 2]'];
axe.grid =[0.1 0.1 -0.1];
axe.auto_clear = "off" ;

plot(x,y);
displayInertiaCenter(PI)
displayCOM2d(COM)
show_pixmap()

pause


//---------------//
// MGD direct
//---------------//
[Mdirect_01, Mdirect_02, Mdirect_03, Mdirect_04, Mdirect_05, Mdirect_06, Adirect_0, Bdirect_0, Cdirect_0, Pdirect_0] = computeMGDsagitalManDir(d,q);


// ----------------- // 
// Calcul du Jacobien 
//-------------------//
J06 = computeJ06SagMan(d,q);

// -----------------//
// Loi de commande pour
// garantir que les pieds du deambulateur touchent par terre
//------------------//
tol = 0.001;
[qnew] = applyConstraintsWalker(q,d,tol);

xset('window',2)
[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
[PI, COM ,m] = humanWalkerSagInertia(qnew,d,masse);



// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);
plot(x,y,'g');
displayInertiaCenter(PI)
displayCOM2d(COM)
show_pixmap()

pause
//------------------//
// simulation du mouvement de l'effecteur
//------------------//
// soit la vitesse de l'effecteur
v = [0 1 0];
qprec = qnew;
dt =0.1;
xset('window',2)
for i=0:10
  dotq = pinv(J06)*v';
  qnew= qprec+dotq*dt;
  qnew = applyConstraintsWalker(qnew',d,tol);

  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
  [PI, COM, m] = humanWalkerSagInertia(qnew,d,masse);
  // le robot est un robot plan sur xy
  sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
  y = sommets (1,:);
  x = sommets (2,:);
  plot(x,y,'r');
  displayInertiaCenter(PI)
  displayCOM2d(COM)
  show_pixmap()

  J06 = computeJ06SagMan(d,qnew);
  qprec=qnew;
end



