// on veut creer un champ de vitesse pour un objet...
// histoire de se derouiller les meninges
clear
getd("src/transformation")

//1 . Creation de l'objet : c'est un ensemble de points 3D
// creation d'un cube
body = [0,0,0;0,0,1;0,1,0;0,1,1;1,0,0;1,0,1;1,1,0;1,1,1];

//2. On definit la position du cube dans un repere fixe o
pose = [0, 2, 3, %pi/8, 0, 0];
oMb  = homogeneousMatrixFromPos(pose);


body_in_o =[];
//3. On place le cube dans le repere 0
for i=1:size(body,1)
  //on construit un point sous la forme homogene
  bP = [body(i,:),1]';
  oP = oMb*bP;
  body_in_o=[body_in_o;oP(1:3)'] ;
end 


//4. On trace le Cube
plot3d ([0,0.5],[0,1],[0,1]);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;


//5. On definit la vitesse a applique au cube

//6. Pour tous les points en en deduis la vitesse 

//7. On trace le cube

//8. On trace le champs de vitesse

