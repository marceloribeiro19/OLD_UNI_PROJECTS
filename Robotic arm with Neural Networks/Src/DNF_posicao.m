function output_posicao=DNF_posicao(output_tamanho,output_cor)
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
x = 0:dx:360; 
nx = length(x);

Tau = 0.1;       % time constant
Time = 10*Tau;    %time during which the field is integrated 
dt =Tau/5;             %Euler time step
nt = round(Time/dt);   % number of cycles 

u_posicao = zeros(nt,nx);  % field activation
u(1,:) = 0;


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
sigmak = 10;
kk = - Ak*exp(-lcoop^2/(8*sigmak^2)); % minimum value of kernel
xc=(max(x)-min(x))/2;
w=gauss(x,xc,sigmak,Ak,kk);  
Wm = (Ak-abs(kk))*lcoop/4;
end

%plot kernel
figure(1)
clf
%subplot(6,1,1)
plot(x-xc,w);  
grid on
title('Interaction Kernel');


% --------------------------- Resting level, h---:

h=-0.5;


% -------------------- Parameters for s(x,t) ----------------------
% Inputs are considered gaussian like type functions :
%S= zeros(1,nx);
S_cor_p=output_cor;
S_forma_p=output_tamanho;
S_posicao =S_cor_p'+S_forma_p';

figure(2)
subplot(2,1,1)
plot(x,S_posicao)
grid on
title('Input to the field : S(x)');



% ------------ Amari field dynamics -------------------------
lw = length(w);
u_conv = zeros(1,nx+lw); 

for t=2:nt,
    u_past_posicao = u_posicao(t-1,:);

    % Periodic boundary conditions:   
    lw2=fix(lw/2);
    u_conv(1:lw2) = u_past_posicao(nx-lw2+1:nx);     
    u_conv(lw2+2:nx+lw2+1) =u_past_posicao(1:nx);
    u_conv(nx+lw2+2:nx+lw) = u_past_posicao(1:lw2);
    
    
    % applies non-linearity to u_conv :
    f_u_conv = zeros(1,nx+lw);

%---- ramp function with saturation: 
    for j=1:nx+lw
        if u_conv(j) > 0 &&  u_conv(j) < 1
           f_u_conv(j) = u_conv(j);
        elseif u_conv(j) >= 1
               f_u_conv(j) = 1;
        end
    end

    % Computes convolution:
    % IMP: in order for the discret convolution to be equal to the
    % continuos convolution in time it is necessary, despite the
    % padding with zeros, also to multiply by dx:
    %
    %  f*w = int[ f(x-x').w(x') . dx'] = sum[ f(i-j).w(i).dx'] 
    %
    c = conv(f_u_conv,w*dx);
    % Follows field iteration:
    u_posicao(t,:) = u_past_posicao + dt/Tau *(-u_past_posicao + c(lw+1:nx+lw)+h+S_posicao )+ ...
                                 0.01*randn(1,nx);
   %plot input and field activation
   subplot(2,1,1)
   plot(x,S_posicao','blue',x,u_posicao(t,:),'m')
   xlabel(' blue- input  magenta- excitation')
   grid on
   title('Input to the field : S(x)');
                             
   %plot field activity
   figure(2)
   subplot(2,1,2)
   plot(x,u_posicao(t,:),'m', x,u_posicao(1,:),'g')
   xlabel(' green- initial excitation  magenta- excitation')
   title('u(x,t)')
   grid on
   pause(0.25)

end 
% Funcao que filtra o output da amari sobrando apenas valores
% positivos, tornando os negativos=0
y_posicao=zeros(1,nx);
for i=1:nx
    if(u_posicao(50,i)>0)
    y_posicao(1,i)=u_posicao(50,i);
    else
    y_posicao(1,i)=0;
    end
end
figure (8)
plot(x,y_posicao)

% Funcao para encontrar o pico da gassiana de saida e a sua respetiva
% posicao no array de angulos

max_posicao=y_posicao(1,1);
                       for i=1:nx
                            if(y_posicao(1,i)>max_posicao)
                                max_posicao=y_posicao(1,i);
                                posicao=i;
                            end
                       end
output_posicao=posicao;
end
