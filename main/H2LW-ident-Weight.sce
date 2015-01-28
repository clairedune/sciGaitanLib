// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touche

// read qualysis data


clear all
xdel(winsid());

disp('chargement du fichier de données');
path = '/home/dune/Documents/data/AnalysisQualisys/MADN/';


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
disp(size(P));
Ptest = []
pasTest   = 1;
finTest   = 1;
debutTest = 1;

pathres = path+'results/essai-1-1-1-';

for index = debutTest:pasTest:finTest
  Pi = P(:,index);
  Pi = Pi'/1000; //tout mettre en metre
  Ptest=[Ptest,Pi];
end


disp('affichage des données de test mocap')
// affichage

graphique(1,4)
xtitle("Motion capture data", "x (mm)","z (mm)");
humanMocapSet10dofPlot(Ptest);
show_pixmap();
xs2eps(gcf(),pathres+"mocap.eps");

pause
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

Pmodel = computePset(x0);

disp('affichage du modele')
graphique(2,4)
xtitle("Motion capture data (plain line) vs initial model guess (dash line)", "x (mm)","z (mm)");
humanMocapSet10dofPlot(Ptest);
humanSet10dofPlot(Pmodel);
show_pixmap();
xs2eps(gcf(),pathres+"mocapVSinitmodel.eps");


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

graphique(3,4)
xtitle("Motion capture data (plain line) vs estimated model (dash line)", "z (mm)","x (mm)");
humanMocapSet10dofPlot(Pestime);
humanSet10dofPlot(Pmodel);
show_pixmap();
xs2eps(gcf(),pathres+"mocapVSestimation.eps");

graphique(4,4)
xtitle("Estimated model", "x (mm)","z (mm)");
humanSet10dofPlot(Pestime);
show_pixmap();
xs2eps(gcf(),pathres+"estimation.eps");


//for i=1:10
//    n = 20+i;
//    xset("window",n)
//    plot(Pestime(10+i-1:30:$),'r--');
//    plot(Ptest(10+i-1:30:$),'r');
//    plot(Pestime(11+i:30:$),'b--');
//    plot(Ptest(11+i:30:$),'b');
//show_pixmap();
//end

dvalidTest   =  [];
derrorTest   =  [];
dest     =  x(1:9);

for index = debutTest:pasTest:finTest
   Pvalid    =  P(:,index)/1000;
   Pvalid    =  Pvalid';
   d = H2LWScomputeSegLength(Pvalid);
   dvalidTest = [dvalidTest ; d];
   derrorTest = [derrorTest ;abs(d-dest')];
end

graphique(5,4)
xtitle("Segment length from test data ", "iteration","distance (mm)");
plot(dvalidTest,'o-')
show_pixmap();
xs2eps(gcf(),pathres+"distance-model-testdata.eps");

graphique(6,4)
xtitle("Segment-length error between test data and model", "iteration","distance (mm)");
plot(derrorTest,'o-')
show_pixmap();
xs2eps(gcf(),pathres+"distance-error-model-testdata.eps");

avSLTest = [];
sdSLTest = [];
avErrorTest = [];
sdErrorTest = [];
for index=1:9
    avSLTest = [avSLTest;mean(dvalidTest(:,index))];
    sdSLTest = [sdSLTest;st_deviation(dvalidTest(:,index))];
    avErrorTest = [avErrorTest;mean(derrorTest(:,index))];
    sdErrorTest = [sdErrorTest;st_deviation(derrorTest(:,index))];
end    

//------------------------------------------------------------
// Validation for all the data including the test data
//------------------------------------------------------------
dvalid = [];
derror = [];
dest   = x(1:9);

for index = finTest:1:1600
   Pvalid = P(:,index)/1000;
   Pvalid = Pvalid';
   d = H2LWScomputeSegLength(Pvalid);
   dvalid=[dvalid;d];
   derror=[derror;abs(d-dest')];
end

graphique(7,4)
xtitle("Segment length estimation", "iteration","distance (mm)");
plot(dvalid)
show_pixmap();
xs2eps(gcf(),pathres+"distance-mocap-alldata.eps");

graphique(8,4)
xtitle("Segment length error (all mocap data vs model) ", "iteration","distance (mm)");
plot(derror)
show_pixmap();
xs2eps(gcf(),pathres+"distance-error-mocap-alldata.eps");


avSL = [];
sdSL = [];
maxSL = [];
minSL = [];
avError = [];
sdError = [];
maxError = [];
minError = [];
for index=1:9
    avSL = [avSL;mean(dvalid(:,index))];
    sdSL = [sdSL;st_deviation(dvalid(:,index))];
    maxSL = [maxSL;max(dvalid(:,index))];
    minSL = [minSL;min(dvalid(:,index))];
    avError = [avError;mean(derror(:,index))];
    sdError = [sdError;st_deviation(derror(:,index))];
    maxError = [maxError;max(derror(:,index))];
    minError = [minError;min(derror(:,index))];
end    
