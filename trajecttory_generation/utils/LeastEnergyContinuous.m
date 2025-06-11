function phaseout = LeastEnergyContinuous(input)

%% constants %%

c1 = input.auxdata.c1;
c2 = input.auxdata.c2;
c3 = input.auxdata.c3;
c4 = input.auxdata.c4;
c5 = input.auxdata.c5;         
c6 = input.auxdata.c6; 
c7 = input.auxdata.c7;
c8 = input.auxdata.c8;
c9 = input.auxdata.c9;       
kb = input.auxdata.kb;
ktau = input.auxdata.ktau;

M = input.auxdata.M;  
g = input.auxdata.g; 
L = input.auxdata.L;
J = input.auxdata.J;
Iy = input.auxdata.Iy; 
Ix = input.auxdata.Ix; 
Iz = input.auxdata.Iz; 
%%

x = input.phase.state;
u = input.phase.control;

x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);
x5 = x(:,5);
x6 = x(:,6);
x7 = x(:,7);   sx7 = sin(x7); cx7 = cos(x7);
x8 = x(:,8); 
x9 = x(:,9);   sx9 = sin(x9); cx9 = cos(x9);
x10 = x(:,10);
x11 = x(:,11); sx11 = sin(x11); cx11 = cos(x11);
x12 = x(:,12);
x13 = x(:,13);
x14 = x(:,14);
x15 = x(:,15);
x16 = x(:,16);
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);
u4 = u(:,4);

x1dot  = x2;                     
x2dot  = (kb/M)*(sx7.*sx11 + cx7.*cx11.*sx9).*(x13.*x13 + x14.*x14 + x15.*x15 + x16.*x16);
x3dot  = x4;                     
x4dot  = (kb/M)*(cx7.*sx9.*sx11 - cx11.*sx7).*(x13.*x13 + x14.*x14 + x15.*x15 + x16.*x16);
x5dot  = x6;
x6dot  = (kb/M)*(cx9.*cx7).*(x13.*x13 + x14.*x14 + x15.*x15 + x16.*x16) - g;
x7dot  = x8;
x8dot  = ((Iy-Iz)/Ix)*x10.*x12 + (L*kb/Ix)*(x14.*x14 - x16.*x16) + (J*x10./Ix).*(x13 - x14 + x15 - x16); 
x9dot  = x10;
x10dot = ((Iz-Ix)/Iy)*x8.*x12 + (L*kb/Iy)*(x15.*x15 - x13.*x13) + (J*x8./Iy).*(x13 - x14 + x15 - x16); 
x11dot = x12;
x12dot = ((Ix-Iy)/Iz)*x8.*x10 + (ktau/Iz)*(x13.*x13 + x15.*x15 - x14.*x14 - x16.*x16);
x13dot = u1;
x14dot = u2;
x15dot = u3;
x16dot = u4;

phaseout.dynamics = [x1dot, x2dot, x3dot, x4dot, x5dot, x6dot, x7dot, x8dot, x9dot, x10dot, x11dot, x12dot, x13dot, x14dot, x15dot, x16dot];
phaseout.integrand = (4*c4 + c3*(x13+x14+x15+x16) + c2*(x13.*x13 + x14.*x14 + x15.*x15 + x16.*x16) + c1*(u1.*u1 + u2.*u2 + u3.*u3 + u4.*u4) + c7*(x13.*x13.*x13 + x14.*x14.*x14 + x15.*x15.*x15 + x16.*x16.*x16) + c8*(x13.*x13.*x13.*x13 + x14.*x14.*x14.*x14 + x15.*x15.*x15.*x15 + x16.*x16.*x16.*x16));

%