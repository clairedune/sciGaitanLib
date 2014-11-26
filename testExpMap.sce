// ok, le test fonctionne.

clear
getd("src/transformation")

dx=[0.1 0 0 0.002 0.004 0];
disp('deplacement dx');
disp(dx);

M = homogeneousMatrixFromPos(dx);
disp('homogeneous M from dx');
disp(M);

dt=0.01;

dotx = dx/dt
vitesse = expMapInverseThetaU(M,dt)
M3 = expMapDirectThetaU(vitesse,dt)

deplacement = pFromHomogeneousMatrix(M3);
disp('le deplacement resultant est')
disp(deplacement)


