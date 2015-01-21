// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touche

// read qualysis data

disp('chargement du fichier de données');
path = '/home/dune/Documents/data/AnalysisQualisys/MADN/';
pathres = 'results/essai1-';

pathMatFile = path+ 'KBM-essai1.mat';
loadmatfile(pathMatFile);

//--------------------------------------------------------------//
// Nouvelle methode pour le prise en compte facilité de 
// plusieurs pas de temps, les donnees sont en colonnes x,y,z
// et ordonnées telles que dans la construction du modele
//--------------------------------------------------------------//

disp('construction des points 3D issus des données mocap');
P = mocap10dofData(essai1);
global Ptest ;

Ptest = []
for index = 1000:1:1000
  Pi = P(:,index);
  Pi = Pi'/1000; //tout mettre en metre
  Ptest=[Ptest,Pi];
end


disp('affichage des données de test mocap')
// affichage

//humanMocap10dofPlot(Ptest);
xset("window",9);
humanMocapSet10dofPlot(Ptest);
xset("window",10);
//humanMocap10dofPlot(Ptest);
humanMocapSet10dofPlot(Ptest);
show_pixmap();


//------------------------------------------------------------------//
// le robot est un robot plan sur xy
// on construit une premiere position arbitrairement
//------------------------------------------------------------------//
disp('construction du modele du corps humain');
//q1  = 0 ;      // angle plante du pied/sol G
q2  = 0.72546 ;  // cheville G
q3  = -0.7 ;     // genou G
q4  = 0.4;       // hanche gauche 
q5  = %pi-.3;    // hanche droite
q6  = 0.2;       // genou D
q7  = 0.39 ;     // cheville D
//q8  = 0;         // pied D
q9  = 3.3 ;      // epaule gauche
q10 = -0.8 ;     // coude G
//q11 = -1.89 ;    // poignet G
q12 = 3;         // epaule droite
q13 = -0.4;      // coude droit
//q14 = 0.6577 ;   // poignet D
q0   = [q2,q3,q4,q5,q6, q7,q9,q10,q12,q13];


// dimensions des segments du corps humain
global d0;
d0   = H2LWSagSeg();

// adaptation a la main
d0(1)=d0(1)+0.1;
d0(5)=d0(1);
d0(2)=d0(2)+0.1;
d0(4)=d0(2);
d0(3)=d0(3)+0.15;

// position de la base du modele dans le repere du monde
tz0 = 0.0864232;//;Ptest(2,6); // position du pied dans le plan sagital 
tx0 = -0.0976375 ;//Ptest(3,6); // elevation du pied au sol 

x0  = [d0];

N = length(Ptest)/30;

for i=1:N
    x0 = [x0,q0,tz0,tx0];
end
x0  = x0';
// on construit le modele
//Pmodel = computePestCol(x0);// le robot est un robot plan sur xy

Pmodel = computePset(x0);

disp('affichage du modele')
// on affiche le model
//humanModel10dofPlot(Pmodel);
humanSet10dofPlot(Pmodel);



//-------------------------------------------------------------//
// Estimation des parametres par optimisation numerique 
//------------------------------------------------------------//
disp('Pause avant optimisation')
pause
//simplex nelder&mead
//x = fminsearch ( costfunction , x0 );
//moindre carre
[fopt,x]=leastsq(costfunctionSet, x0);
Pestime= computePset(x);
//humanModel10dofPlot(Pestime);
humanSet10dofPlot(Pestime);

xset("window",11);
humanSet10dofPlot(Pestime);

//for i=1:10
//    n = 20+i;
//    xset("window",n)
//    plot(Pestime(10+i-1:30:$),'r--');
//    plot(Ptest(10+i-1:30:$),'r');
//    plot(Pestime(11+i:30:$),'b--');
//    plot(Ptest(11+i:30:$),'b');
//show_pixmap();
end