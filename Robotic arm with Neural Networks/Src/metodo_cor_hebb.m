function W_cor=metodo_cor_hebb()
dx = 1; %0.5
x_cor = 0:dx:900;  %40
nx_cor=length(x_cor);

%Gaussiana para a cor vermelha
x0_r = 682.5; 	
sigma_r = 67.5;	

%Gaussiana para a cor verde
x0_g=532.5;
sigma_g=32.5;

%Gaussiana de saida para a cor verde
x0_90=90;
x0_110=110;
sigma_angulos=2;

%gaussiana de saida para a cor vermelha
x0_70=70;

%dados para o calculo da gaussiana
A = 5;	
k = -0.5;	

%calculo da gaussiana input
S_r = 1.5*gauss(x_cor,x0_r,sigma_r,A,k);
S_g= 1.5*gauss(x_cor,x0_g,sigma_g,A,k);
S_entrada_cor=zeros(nx_cor,2);
S_entrada_cor(:,1)=S_r;
S_entrada_cor(:,2)=S_g;
figure(1)
subplot(2,1,1)
plot(x_cor,S_r,'r',x_cor,S_g,'g')
grid on
title('Input:');

%calculo da gaussiana output
S_output_g = (1.5*gauss(x_cor,x0_90,sigma_angulos,A,k)+1.5*gauss(x_cor,x0_110,sigma_angulos,A,k));
S_output_r= (1.5*gauss(x_cor,x0_70,sigma_angulos,A,k));
S_saida_cor=zeros(nx_cor,2);
S_saida_cor(:,1)=S_output_r;
S_saida_cor(:,2)=S_output_g;
subplot(2,1,2)
plot(x_cor,S_output_r,'r',x_cor,S_output_g,'g')
grid on
title('Output:');

 N_cor=length(S_saida_cor(:,1));
 M_cor=length(S_entrada_cor(:,1));
 alfa=0.01;
 W_cor=zeros(M_cor,N_cor);
  for k=1:2
     for i=1:M_cor  %each ouput neuron i 
         for j=1:N_cor  % for each input xj 
             dW_cor = alfa*S_saida_cor(i,k)*S_entrada_cor(j,k);
             W_cor(i,j)=W_cor(i,j)+dW_cor;  %update synaptic weight from input unit j to ouput neuron i
         end 
     end 
  end
end