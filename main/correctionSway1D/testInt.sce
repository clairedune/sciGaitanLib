// on chercher a approximer 
// un signal sinusoidal par une linearisation
// du premier ordre
// soit 
//  F(t)=sin(b*t+a)/b un signal
// soit
// Fprime(t)=cos(b*t*a) sa derivee
// alors on doit avoir equivalence entre
// F(t)=sin(b*t+a)/b
// et
// F(k+1) = F(k) + T*Fprime(k);


clear;

// on fixe le pas d'echantillonage
T=0.01;
// on en deduit un ensemble d'echantillon temporels sur 10 secondes
t=0:T:10;

// on choisit arbitrairement a et b
a=-2;
b=0.5;

// on en deduit F
F=sin(b*t+a)/b;
// et Fprime
Fprime = cos(b*t+a);

// on les affiche
plot(t,F,'b')
plot(t,Fprime,'k-.')

k=F(1);
K=[k];
tstore=[0];
iter=1;
for t=T:T:10
  
  k = k + T*Fprime(iter);
  K = [K;k];
  tstore=[tstore;t];
  iter=iter+1;
end


plot(tstore,K,'r')

mean(K)
