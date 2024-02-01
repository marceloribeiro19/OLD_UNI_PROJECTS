[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end
[obj,error_rob,pub_joint_single,msg_joint_single,sub_joint_position] = px100_interface(sim);
if(error_rob==1)
    return;
end

% Estabelece a comunicacao entre o computador onde corre o matlab e o
% computador onde esta a correr o ROS_MASTER

% try 
%     setenv("ROS_IP",'192.168.150.23');  %IP pconde está a correr o matlab
%     setenv("ROS_HOSTNAME",'192.168.150.89'); %IP pconde está a correr o ROS_MASTER 
%     rosinit('192.168.150.89', 'NodeHost','192.168.150.23','NodeName','/test_node') 
% catch 
%     warning('ROS alreadyinitialized'); 
% end

% Carregar os ficheiros das redes ja treinadas e guardadas
load('Rede_direta.mat');
load('Rede_inversa.mat');
load('rede_inversa_real.mat'

% Calcular os pesos das ligacoes entre neuronios segundo o Hebbian learning
W_forma=metodo_forma_hebb();
W_cor=metodo_cor_hebb();

n_objetos=1;
  while n_objetos>0

    [error,img,posicoes,posicoes_tipo,n_objetos]=obj.getvision_real();
    obj.home_braco(pub_joint_single,msg_joint_single);
    obj.open_gripper(pub_joint_single,msg_joint_single);
    prompt="SELECIONE OBJETO: ";
    objeto_selecionado = input(prompt)
    [error,angulos]=obj.placeobject_real(objeto_selecionado,posicoes,posicoes_tipo,pub_joint_single,msg_joint_single,W_forma,W_cor);
    n_objetos=n_objetos-1;
 end
 [error]=obj.desligar_braco(pub_joint_single,msg_joint_single);
 obj.close_gripper(pub_joint_single,msg_joint_single); 







 


