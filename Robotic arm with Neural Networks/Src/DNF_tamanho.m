function output_tamanho=DNF_tamanho(tamanho,W_forma)
%  Amari1d_simple_EB.m
%  
% This script file simulates an Amari bistable neural field of 4th Type,
% i.e., in which an initial excitation causes either a localized 
% excitation (peak) of a definite length or its extinction. 
% 
% Tau* du(x,t)/dt = -u(x,t) + Integral( w(x-x')f[u(x)] ) dx + 
%			 + h + S(x,t)    
% 
% u(x,t) - is the activation of neuron at position x at time t
% S(x,t) - is the external input to neuron at position x at time t
% h is made dynamic in order to accomplish adaptation:
% Tau - is the time constant
% 
%
% Estela Bicho 
% Marseille 25th August 1996
%


% ---------------- Field Variables: -----------------------------
%
% u(x,t) - is the the average potential at neuron position x at time t.
% u_conv - auxiliar array which is used to accomplish a linear 
%          convolution from the CONV matlab function which 
%	        computes a circular convolution.
%
% nx - Number of samplig points over the behavioral dimension x 
%      (number of neurons).
% dx - Sampling distance along behavioral dimension x.
% nt - Number of Euler time steps along which field equation 
%      is iterated.
% dt - Euler time step in the iteration of the field equation.
% Tau - Relaxation Constant of the field.
% TH - Output Threshold to excitation after the field as converged


% Notes about equilibrium solutions  when S(x)=0 :
% for a kernel of the type
%
%         w = Ak*exp[-(( x-xok).^2)./(2*sigmak^2) ] + Kk
%  
%
%  For -Wm < h < -2Woo   (Woo < 0) --> { 0 a1~ a2 }   (s1)
%  For  h < -Wm    --> { 0 }   (s2)
%
%  See Amari's paper from 1977.

clf

% Initializations
dx = 1; %0.5
x_forma = 0:dx:500;  
nx_forma=length(x_forma);
x_saida_forma=0:dx:360;
nx_saida_forma=length(x_saida_forma);

Tau = 0.1; % time constant
Time = 10*Tau; %time during which the field is integrated 
dt =Tau/5; %Euler time step
nt = round(Time/dt); % number of cycles 

u_forma = zeros(nt,nx_forma);  % field activation
u_forma(1,:) = 0;


% -------------Parameter for the the weighting function, ------------
% ------------------ i.e. interaction kernel, W --------------------
%
% The kernel is a gaussian like type function with infinite 
% negative tails in order to ensure global inibition:
% Ak - kernel amplitude.  Difference between maximum and minimum.
% lcoop - length of cooperative interaction. I.e. in -lcoop/2< x < lcoop/2
%         kernel is positive. 
% sigmak - sigma parameter of gaussian curve.
%
%                       OR
%
% The kernel is a rectangle like type function with :
%    w = kp   if x <= lcoop
%    w = -Kn  if x > lcoop  
%
%
%                       OR
% The kernel can be a mexican hat for allowing the coexistance of
% multiple peaks.


% gaussian kernel:
if 1
%****gaussian kernel*************************************************
Ak =4;   %3 1
lcoop = 5; %5 20           
sigmak = 20;
kk_forma = - Ak*exp(-lcoop^2/(8*sigmak^2)); % minimum value of kernel
xc_forma=(max(x_forma)-min(x_forma))/2;
w_forma=gauss(x_forma,xc_forma,sigmak,Ak,kk_forma);  
Wm_forma = (Ak-abs(kk_forma))*lcoop/4;
end

%plot kernel
figure(1)
clf
%subplot(6,1,1)
plot(x_forma-xc_forma,w_forma);  
grid on
title('Interaction Kernel');


% --------------------------- Resting level, h---:

h=-0.5;

%---------CRIAÇAO DAS GAUSSIANAS--------------
%Gaussiana para a size small
x0_s = 100; 	
sigma_s = 20;	

%Gaussiana para a size large
x0_l=300;
sigma_l=20;

%dados para o calculo da gaussiana
A = 5;	
k = -0.5;	

%calculo da gaussiana input
S_s = 1.5*gauss(x_forma,x0_s,sigma_s,A,k);
S_l= 1.5*gauss(x_forma,x0_l,sigma_l,A,k);


% -------------------- Parameters for s(x,t) ----------------------
% Inputs are considered gaussian like type functions :

switch tamanho
    case 's'
        flag_small=1;
        flag_large=0;
    case 'l'
        flag_small=0;
        flag_large=1;
    otherwise
        disp("erro: tamanho nao reconhecido");
end
S_forma = flag_small*S_s+flag_large*S_l;

figure(2)
subplot(2,1,1)
plot(x_forma,S_forma)
grid on
title('Input to the field : S(x)');



% ------------ Amari field dynamics -------------------------
lw_forma = length(w_forma);
u_conv_forma = zeros(1,nx_forma+lw_forma); 

for t=2:nt
    u_past_forma = u_forma(t-1,:);

    % Periodic boundary conditions:   
    lw2_forma=fix(lw_forma/2);
    u_conv_forma(1:lw2_forma) = u_past_forma(nx_forma-lw2_forma+1:nx_forma);     
    u_conv_forma(lw2_forma+2:nx_forma+lw2_forma+1) =u_past_forma(1:nx_forma);
    u_conv_forma(nx_forma+lw2_forma+2:nx_forma+lw_forma) = u_past_forma(1:lw2_forma);
    
    
    % applies non-linearity to u_conv :
    f_u_conv_forma = zeros(1,nx_forma+lw_forma);
%---- ramp function with saturation: 
    for j=1:nx_forma+lw_forma
        if u_conv_forma(j) > 0 &&  u_conv_forma(j) < 1
           f_u_conv_forma(j) = u_conv_forma(j);
        elseif u_conv_forma(j) >= 1
               f_u_conv_forma(j) = 1;
        end
    end

    % Computes convolution:
    % IMP: in order for the discret convolution to be equal to the
    % continuos convolution in time it is necessary, despite the
    % padding with zeros, also to multiply by dx:
    %
    %  f*w = int[ f(x-x').w(x') . dx'] = sum[ f(i-j).w(i).dx'] 
    %
    c_forma = conv(f_u_conv_forma,w_forma*dx);
    % Follows field iteration:
    u_forma(t,:) = u_past_forma + dt/Tau *(-u_past_forma + c_forma(lw_forma+1:nx_forma+lw_forma)+h+S_forma )+ ...
                                 0.01*randn(1,nx_forma);
   %plot input and field activation
   subplot(2,1,1)
   plot(x_forma,S_forma,'blue',x_forma,u_forma(t,:),'m')
   xlabel(' blue- input  magenta- excitation')
   grid on
   title('Input to the field : S(x)');
                             
   %plot field activity
   figure(2)
   subplot(2,1,2)
   plot(x_forma,u_forma(t,:),'m', x_forma,u_forma(1,:),'g')
   xlabel(' green- initial excitation  magenta- excitation')
   title('u(x,t)')
   grid on
   pause(0.25)

end

% Funcao que filtra o output da amari sobrando apenas valores
% positivos, tornando os negativos=0
y_forma=zeros(1,nx_forma); 
for i=1:nx_forma
    if(u_forma(nt,i)>0)
        y_forma(1,i)=u_forma(nt,i);
    else
        y_forma(1,i)=0;
    end
end
figure (4)
plot(x_forma,y_forma);
S_forma_p=zeros(nx_saida_forma,1);
for i=1:nx_saida_forma
    for j=1:nx_forma
    S_forma_p(i)=S_forma_p(i)+(y_forma(j)*W_forma(i,j));  
    end
end
figure(5)
plot(x_saida_forma,S_forma_p)
output_tamanho=S_forma_p;
save s_l
save s_forma_p
end