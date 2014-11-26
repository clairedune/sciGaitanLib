// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touchent le sol.


// simuInit 
q1 = 10*%pi/180 ;
q2 = -40*%pi/180 ;
q3 = 40*%pi/180 ;
q4 = 110*%pi/180 ;
q5 =  -30*%pi/180 ;
q6 = -%pi/2;

q = [q1,q2,q3,q4,q5,q6];

//-------- Parametres du robot ------- //
d = humanWalkerSagSeg();
// articular position limit            
qlimit = humanWalkerSagJointLimit();          
masse = 70;
yDes =0.2;



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
plot(x,y);
//displayInertiaCenter(PI)
//displayCOM2d(COM)
show_pixmap()


// ----------------- // 
// Calcul du Jacobien 
//-------------------//
J06 = computeJ06SagMan(d,q);

// -----------------//
// Loi de commande pour
// garantir que les pieds du deambulateur touchent par terre
//------------------//
tol = 0.001;
dt = 0.1;

[qnew1] = applyConstraintsWalker(q,d,dt,tol);
[qnew2] = applyConsAndLimits(q,d,qlimit,dt,tol);
[qnew3] = applyLimitsandConst(q,d,qlimit,dt,tol);
[qnew4] = applyActivSet(q,d,qlimit,dt,tol);


// ----------------- // 
// Affichage 
//-------------------//
xset('window',2)
[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew1);
[PI, COM ,m] = humanWalkerSagInertia(qnew1,d,masse);
//// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);
plot(x,y,'g');
//displayInertiaCenter(PI)
//displayCOM2d(COM)

[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew2);
[PI, COM ,m] = humanWalkerSagInertia(qnew2,d,masse);
//// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);
plot(x,y,'c-.');
//pause
[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew3);
[PI, COM ,m] = humanWalkerSagInertia(qnew3,d,masse);
//// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);
plot(x,y,'m-.');

//displayInertiaCenter(PI)
//displayCOM2d(COM)


[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew4);
[PI, COM ,m] = humanWalkerSagInertia(qnew4,d,masse);
//// le robot est un robot plan sur xy
sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
y = sommets (1,:);
x = sommets (2,:);
plot(x,y,'r-.');
show_pixmap()


