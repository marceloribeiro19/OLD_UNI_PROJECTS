function output_cor=DNF_cor(cor,W_cor)
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

dx = 1;
x_cor = 0:dx:900;
x_saida_cor=0:dx:360;
nx_cor=length(x_cor);
nx_cor_saida=length(x_saida_cor);

Tau = 0.1;     % time constant
Time = 10*Tau;  %time during which the field is integrated 
dt =Tau/5;             %Euler time step
nt = round(Time/dt);   % number of cycles 

u_cor = zeros(nt,nx_cor);  % field activation
u_cor(1,:) = 0;


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
Ak =4;
lcoop = 5;          
sigmak = 20;
kk_cor = - Ak*exp(-lcoop^2/(8*sigmak^2)); % minimum value of kernel
xc_cor=(max(x_cor)-min(x_cor))/2;
w_cor=gauss(x_cor,xc_cor,sigmak,Ak,kk_cor);  
Wm_cor= (Ak-abs(kk_cor))*lcoop/4;
end

%plot kernel
figure(1)
clf
plot(x_cor-xc_cor,w_cor);  
grid on
title('Interaction Kernel');

%------------CRIAÇAO DAS GAUSSIANAS----------

%Gaussiana para a cor vermelha
x0_r = 682.5;
sigma_r = 67.5;

%Gaussiana para a cor verde
x0_g=532.5;
sigma_g=32.5;

%dados para o calculo da gaussiana
A = 5;
k = -0.5;

%calculo da gaussiana input
S_r = 1.5*gauss(x_cor,x0_r,sigma_r,A,k);
S_g= 1.5*gauss(x_cor,x0_g,sigma_g,A,k);


% --------------------------- Resting level, h---:

h=-0.5;


% -------------------- Parameters for s(x,t) ----------------------
% Inputs are considered gaussian like type functions :

switch cor
    case 'r'
        flag_r=1;
        flag_g=0;
    case 'g'
        flag_r=0;
        flag_g=1;
    otherwise
        disp("erro: cor nao reconhecida");
end
S_cor = flag_g*S_g+flag_r*S_r;

figure(2)
subplot(2,1,1)
plot(x_cor,S_cor)
grid on
title('Input to the field : S(x)');



% ------------ Amari field dynamics -------------------------
lw_cor = length(w_cor);
u_conv_cor = zeros(1,nx_cor+lw_cor); 

for t=2:nt
    u_past_cor = u_cor(t-1,:);

    % Periodic boundary conditions:   
    lw2_cor=fix(lw_cor/2);
    u_conv_cor(1:lw2_cor) = u_past_cor(nx_cor-lw2_cor+1:nx_cor);     
    u_conv_cor(lw2_cor+2:nx_cor+lw2_cor+1) =u_past_cor(1:nx_cor);
    u_conv_cor(nx_cor+lw2_cor+2:nx_cor+lw_cor) = u_past_cor(1:lw2_cor);
    
    
    % applies non-linearity to u_conv :
    f_u_conv_cor = zeros(1,nx_cor+lw_cor);
%---- ramp function with saturation: 
    for j=1:nx_cor+lw_cor
        if u_conv_cor(j) > 0 &&  u_conv_cor(j) < 1
           f_u_conv_cor(j) = u_conv_cor(j);
        elseif u_conv_cor(j) >= 1
               f_u_conv_cor(j) = 1;
        end
    end

    % Computes convolution:
    % IMP: in order for the discret convolution to be equal to the
    % continuos convolution in time it is necessary, despite the
    % padding with zeros, also to multiply by dx:
    %
    %  f*w = int[ f(x-x').w(x') . dx'] = sum[ f(i-j).w(i).dx'] 
    %
    c_cor = conv(f_u_conv_cor,w_cor*dx);
    % Follows field iteration:
    u_cor(t,:) = u_past_cor + dt/Tau *(-u_past_cor + c_cor(lw_cor+1:nx_cor+lw_cor)+h+S_cor )+ ...
                                 0.01*randn(1,nx_cor);
   %plot input and field activation
   subplot(2,1,1)
   plot(x_cor,S_cor,'blue',x_cor,u_cor(t,:),'m')
   xlabel(' blue- input  magenta- excitation')
   grid on
   title('Input to the field : S(x)');
                             
   %plot field activity
   figure(2)
   subplot(2,1,2)
   plot(x_cor,u_cor(t,:),'m', x_cor,u_cor(1,:),'g')
   xlabel(' green- initial excitation  magenta- excitation')
   title('u(x,t)')
   grid on
   pause(0.25)

end

% Funcao que filtra o output da amari sobrando apenas valores
% positivos, tornando os negativos=0

y_cor=zeros(1,nx_cor);
for i=1:nx_cor
    if(u_cor(nt,i)>0)
        y_cor(1,i)=u_cor(nt,i);
    else
        y_cor(1,i)=0;
    end
end
figure (4)
plot(x_cor,y_cor);
S_cor_p=zeros(nx_cor_saida,1);
for i=1:nx_cor_saida
    for j=1:nx_cor
    S_cor_p(i)=S_cor_p(i)+(y_cor(j)*W_cor(i,j));  
    end
end
figure(5)
output_cor=S_cor_p;
plot(x_saida_cor,S_cor_p)
end