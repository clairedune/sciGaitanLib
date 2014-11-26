// 1d servo of a position x to zero
// when the PG disturbs the control with an additional sin
// comparison of 2 methods
// 1) cancelling the effect of the sin in the control
// 2) compensate for the sin

clear

// --- Control variable
k  = 0.7; // gain >0
T  = 1e-1;// sampling period
N  = 15; // number of iterations
a  = 0;
b  = 2*%pi/2;

// --- Variables of 1st method 
x  = 5;   // initial position
c  = 0;   // initial correction
c2 =0
u  = 0;   // initial control 
v  = 0;   // initial control+sin
X  = [x]; // position trajectory
U  = [u]; // control trajectory
V  = [v]; // control+sin trajectory
C  = [0]; // "correction"
C2 = [0]; 
Estore=[];
S=[];

iter =0;
period=floor(2*%pi/(b*T));

// --- Begin control Loop
for i=0:T:N,
 // compute the additional sin 
 sway=2*cos(b*i+a);
 
 
 
 
 // --- 1st Method
 // estimate the mean of the current correction
 if(iter<=period)
    E=mean(C2);
 else
    E=mean(C2(iter-period:iter));
   
 end
 
 
 //cancelling the sway motion
 u=-k*(x-(c2-E));
 //if(abs(u)>1)
 // u=sign(u)*1;
 //end

 // the real velocity is the control + sway
 v = u + sway;
 x = x + T*v;

 //c = c+T*(v-u);
 S=[S;T*(v-u)];
 
 if(iter<=period)
    c=sum(S);
    c2=c;
else
   c=sum(S);
   c2=sum(S(iter-modulo(iter,period)+1:iter));
   
 end
 

 // store data for display 
 C=[C; c];
 C2=[C2;c2];
 X=[X; x];
 U=[U; u];
 V=[V; v];
 Estore=[Estore;E];
 


iter=iter+1;
end;// --- end of the loop
i=0:T:N;
// --- Display
xset("window",1);
plot(i',X(2:$),'g')
plot(i',U(2:$),'b')
plot(i',V(2:$),'r')
plot(i',C(2:$),'k-.')
plot(i',Estore,'r-.')

xset("window",2);
plot(i',C(2:$),'k-.')
plot(i',C2(2:$),'r-.')


