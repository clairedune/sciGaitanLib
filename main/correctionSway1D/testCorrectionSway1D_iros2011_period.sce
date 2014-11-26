// 1d servo of a position x to zero
// when the PG disturbs the control with an additional sin
// comparison of 2 methods
// 1) cancelling the effect of the sin in the control
// 2) compensate for the sin

clear

// --- Control variable
sat    = 2;                // control saturation
k      = 0.7;              // gain >0
dt     = 0.1;              // sampling period
N      = 15;               // number of iterations
a      = 0;                // shift 
b      = 2*%pi/2;          // pulsation
T      = 1.5;//floor(2*%pi/(b)); // period

// --- Variables of 1st method 
x      = 5;   // initial position
c      = 0;   // initial correction 
u      = 0;   // initial control 
v      = 0;   // initial control+sin
E      = c ;  // initial biais
i      = 0;   // 0 iteration

X      = []; // position trajectory
U      = []; // control trajectory
V      = []; // control+sin trajectory
C      = []; // "correction" trajectory
Estore = []; // biais estimation bar(s0)-s0
S      = [];



// --- Begin control Loop
for t=0:dt:N,

 // compute the additional sin 
 sway=2*cos(b*t+a);

 //cancelling the sway motion
 u=-k*(x-(c-E));

 // saturation
 //if(abs(u)>1)
 //  u=sign(u)*sat;
 //end

 // the real velocity is the control + sway
 v = u + sway;
 x = x + dt*v;

 S=[S;dt*(v-u)];
 //c = sum(S(i-modulo(i,20)+1:$));
 c = sum(S); 


 // store data for display 
 C=[C; c];
 X=[X; x];
 U=[U; u];
 V=[V; v];

 // estimate the biais of the current correction 
 if (t<=T)
    E=mean(C);
 else
   // E=mean(C(i-T/dt:i-1));
   E=mean(C);
 end 
 Estore=[Estore;E];
 

i=i+1;
end;// --- end of the loop
t=0:dt:N;
// --- Display
xset("window",1);
plot(t',X,'g')
plot(t',U,'b')
plot(t',V,'r')
plot(t',C,'r-.')
plot(t',Estore,'k-.')

xset("window",2);
plot(t',C,'k-.')
//plot(t',C2,'r-.')


