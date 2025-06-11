function p = Parameter(p1)
% p1=1 for Sigma25, p1=2 for MorbidiLara, p5=5 for aztec Hummningbird


if (p1 == 2) %MLara
    M = 1.3;p.M = M;    
    g = 9.8066;p.g = g; 
    L = 0.35/2;p.L = L; 
    Ix = 0.081;p.Ix = Ix; 
    Iy = 0.081;p.Iy = Iy;
    Iz = 0.142;p.Iz = Iz;
    KE = (1000/920)*9.5493e-3;                     
    KT = KE;                  
    Tf = 4e-2;
    Df = 2e-4;                
    Mmot = 0.025;                
    r_rot = 0.014;
    Jm = (0.5*Mmot)*r_rot^2;
    nb = 2;
    Mb = 0.0055;                
    r = 0.12;
    ec = 0.004;                 
    JL = nb*(Mb*(r - ec)^2/4);
    R = 0.2;                    
    J = Jm + JL;                
    p.J = J;
    CT = 0.0048;                
    CQ = CT*sqrt(CT/2);         
    A = pi*r^2;                
    rho = 1.225;               
    kb = CT*rho*A*r^2;p.kb = kb;          
    ktau = CQ*rho*A*r^3;p.ktau = ktau;     
    p.c1 = R*(J/KT)^2;
    p.c2 = (Df/KT)*(R*Df/KT + KE) + 2*R*Tf*ktau/(KT^2);
    p.c3 = (Tf/KT)*(2*R*Df/KT + KE);
    p.c4 = R*(Tf/KT)^2;
    p.c5 = 2*R*J*Tf/(KT^2);         
    p.c6 = (J/KT)*(2*R*Df/KT + KE); 
    p.c7 = (ktau/KT)*(2*R*Df/KT + KE);
    p.c8 = R*(ktau/KT)^2;
    p.c9 = 2*R*J*ktau/(KT^2);   
    p.wh = 912.109;% hover omega
    %p.wh = sqrt((p.M*p.g)/(4*p.kb));
    p.omegamax = 1047;%10000/9.5493;
    %p.omegamax = 2000;%10000/9.5493;
    p.a1 = (p.Iy - p.Iz)/p.Ix;
    p.a3 = (p.Iz - p.Ix)/p.Iy;
    p.rate = 1000;
end
if (p1 == 4) %CDP
    M = 3;p.M = M;    
    g = 9.8066;p.g = g; 
    L = 0.3;p.L = L; 
    Ix = 0.0429;p.Ix = Ix; 
    Iy = 0.0429;p.Iy = Iy;
    Iz = 0.077;p.Iz = Iz;
    KE = 0.025;                     
    KT = KE;                  
    Tf = 4e-2;
    Df = 2e-4;                
    Mmot = 0.025;                
    r_rot = 0.014;
    Jm = (0.5*Mmot)*r_rot^2;
    nb = 2;
    Mb = 0.0055;                
    r = 0.12;
    ec = 0.004;                 
    JL = nb*(Mb*(r - ec)^2/4);
    R = 0.4;                    
    J = Jm + JL;                
    p.J = J;
    CT = 0.0048;                
    CQ = CT*sqrt(CT/2);         
    A = pi*r^2;                
    rho = 1.225;               
    kb = CT*rho*A*r^2;p.kb = 4.848e-5;          
    ktau = CQ*rho*A*r^3;p.ktau = 8.891e-7;     
    p.c1 = R*(J/KT)^2;
    p.c2 = (Df/KT)*(R*Df/KT + KE) + 2*R*Tf*ktau/(KT^2);
    p.c3 = (Tf/KT)*(2*R*Df/KT + KE);
    p.c4 = R*(Tf/KT)^2;
    p.c5 = 2*R*J*Tf/(KT^2);         
    p.c6 = (J/KT)*(2*R*Df/KT + KE); 
    p.c7 = (ktau/KT)*(2*R*Df/KT + KE);
    p.c8 = R*(ktau/KT)^2;
    p.c9 = 2*R*J*ktau/(KT^2);   
    p.wh = 389.37;
    p.omegamax = 640;%10000/9.5493;
    p.a1 = (p.Iy - p.Iz)/p.Ix;
    p.a3 = (p.Iz - p.Ix)/p.Iy;
end
if (p1 == 5) %aztec hummingbird
    M = .698;p.M = M;    
    g = 9.8066;p.g = g; 
    L = .171;p.L = L; 
    Ix = 0.0034;p.Ix = Ix; 
    Iy = 0.0034;p.Iy = Iy;
    Iz = 0.006;p.Iz = Iz;
%     same motro used as in phantom2
    kv=920;%(rpm/v)
    KE = (1/kv)*9.5493;                     
    KT = KE; 
%     obyain experimebtally
    Tf = 4e-2;
    Df = 2e-4;                
%     Mmot = 0.025;                
%     r_rot = 0.014;
%     Jm = (0.5*Mmot)*r_rot^2;
%     nb = 2;
%     Mb = 0.0055;                
%     r = 0.12;
%     ec = 0.004;                 
%     JL = nb*(Mb*(r - ec)^2/4);
% J = Jm + JL; 

%     J = 1.302e-6; 
    J=3.9454e-05;
    p.J = J;
    R = 0.2;
    kb = 7.6184*10^(-8)*(60/(2*pi))^2;                
    ktau =2.6839*10^(-9)*(60/(2*pi))^2;         
%     A = pi*r^2;                
%     rho = 1.225;               
p.kb = kb;          
p.ktau = ktau;     
    p.c1 = R*(J/KT)^2;
    p.c2 = (Df/KT)*(R*Df/KT + KE) + 2*R*Tf*ktau/(KT^2);
    p.c3 = (Tf/KT)*(2*R*Df/KT + KE);
    p.c4 = R*(Tf/KT)^2;
    p.c5 = 2*R*J*Tf/(KT^2);         
    p.c6 = (J/KT)*(2*R*Df/KT + KE); 
    p.c7 = (ktau/KT)*(2*R*Df/KT + KE);
    p.c8 = R*(ktau/KT)^2;
    p.c9 = 2*R*J*ktau/(KT^2);   
    p.wh = 496.39;
    %p.wh = sqrt((p.M*p.g)/(4*p.kb));
    p.omegamax = 860*pi/3;%10000/9.5493;
    %p.omegamax = 2000;%10000/9.5493;
    p.a1 = (p.Iy - p.Iz)/p.Ix;
    p.a3 = (p.Iz - p.Ix)/p.Iy;
    p.rate = 1000;
end
