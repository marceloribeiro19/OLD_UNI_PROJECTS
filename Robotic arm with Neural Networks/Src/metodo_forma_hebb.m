function W_forma=metodo_forma_hebb()
dx = 1; %0.5
x_forma = 0:dx:500;  
nx_forma=length(x_forma);
x_saida_forma=0:dx:360;
nx_saida_forma=length(x_saida_forma);
% Inputs are considered gaussian like type functions :
%Gaussiana para a size small
x0_s = 100; 	
sigma_s = 20;

%Gaussiana para a size large
x0_l=300;
sigma_l=20;

%Gaussiana de saida para size large
x0_70=70;
x0_110=110;
sigma_angulos=2;

%gaussiana de saida para o size small
x0_90=90;

%dados para o calculo da gaussiana
A = 5;
k = -0.5;

%calculo da gaussiana input
S_s = 1.5*gauss(x_forma,x0_s,sigma_s,A,k);
S_l= 1.5*gauss(x_forma,x0_l,sigma_l,A,k);
S_entrada_forma=zeros(nx_forma,2);
S_entrada_forma(:,1)=S_s;
S_entrada_forma(:,2)=S_l;
figure(1)
subplot(2,1,1)
plot(x_forma,S_s,'b',x_forma,S_l,'y');
grid on
title('Input:');

%calculo da gaussiana output
S_output_l = 1.5*gauss(x_forma,x0_70,sigma_angulos,A,k)+1.5*gauss(x_forma,x0_110,sigma_angulos,A,k);
S_output_s= 1.5*gauss(x_forma,x0_90,sigma_angulos,A,k);
S_saida_forma=zeros(nx_forma,2);
S_saida_forma(:,1)=S_output_s;
S_saida_forma(:,2)=S_output_l;
subplot(2,1,2)
plot(x_forma,S_output_s,'b',x_forma,S_output_l,'y')
grid on
title('Output:');

 N_forma=length(S_saida_forma(:,1));
 M_forma=length(S_entrada_forma(:,1));
 alfa=0.01;
 W_forma=zeros(M_forma,N_forma);
  for k=1:2
     for i=1:M_forma  %each ouput neuron i 
         for j=1:N_forma  % for each input xj 
             dW_forma = alfa*S_saida_forma(i,k)*S_entrada_forma(j,k);
             W_forma(i,j)=W_forma(i,j)+dW_forma;  %update synaptic weight from input unit j to ouput neuron i
         end 
     end 
  end
end