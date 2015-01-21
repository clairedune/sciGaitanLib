// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touchent le sol.
// read qualysis data
path = '/home/dune/Documents/data/AnalysisQualisys/MADN/';
pathres = 'results/essai1-';

pathMatFile = path+ 'KBM-essai1.mat';
loadmatfile(pathMatFile);

P = mocap10dofData(essai1);

index=350;
Ptest = P(:,index);
Ptest = Ptest'/1000; //tout mettre en metre

xset("window",10);
humanMocap10dofPlot(Ptest);
show_pixmap();
