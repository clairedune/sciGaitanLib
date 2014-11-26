function M = homogeneousMatrix(x,y,z,rx,ry,rz)
// fonction scilab pour construire une matrice homogene en partant 
// d'un vecteur x,y,z,rx,ry,rz
// les angles sont donnes en radians
  
Tr = [x;y;z];

// Mouvement de rotation
Rot = rotationMatrixFromRxRyRz([rx,ry,rz]);


// Matrice homogene correspondante 
M = [ Rot  Tr  
      0 0 0 1 ];

endfunction
