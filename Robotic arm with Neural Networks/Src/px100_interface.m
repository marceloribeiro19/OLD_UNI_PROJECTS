classdef px100_interface < handle
%   RobDyn_interface Interface to a simulated robot in CoppeliaSim simulator
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2022
%   2022/03/17
    
    properties (Access = private)
        vrep
        clientID
        RobotHandle
        MotorHandle
        camhandle
        
    end
    
    properties
        Motor_Number
        Motor_Name
        camara
 
    end
    
    methods(Access=public)
       
        function [obj,error,pub_joint_single,msg_joint_single,sub_joint_position] = px100_interface(sim_obj)
            
            error = 0;
            [pub_joint_single, msg_joint_single] = rospublisher('/px100/commands/joint_single');% obtencao dos apontadores para publicar no topico joint single

            sub_joint_position= rossubscriber('/px100/joint_states');% obtencao do apontador para subscrever o topico joint states


            [obj.vrep, obj.clientID] = sim_obj.get_connection();
            
            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg);
            end
            
            robot_name = 'px100';
            [res, obj.RobotHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, robot_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting robot handle ');
                error = 1;
                return;
            end
            
            
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
            
            [res,~] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot orientation information');
                error = 1;
                return;
            end
 
            % robot handle for motors
            obj.Motor_Number=6;
            
            % Motor handle

            obj.Motor_Name{1} = 'waist';
            obj.Motor_Name{2} = 'shoulder';
            obj.Motor_Name{3} = 'elbow';
            obj.Motor_Name{4} = 'wrist_angle';
            obj.Motor_Name{5} = 'left_finger';
            obj.Motor_Name{6} = 'right_finger';
           
            
            %Camerahandle
            obj.camara='Vision_sensor';

           

            for t=1:obj.Motor_Number
                [res,obj.MotorHandle{t}]=obj.vrep.simxGetObjectHandle(obj.clientID,obj.Motor_Name{t},obj.vrep.simx_opmode_blocking);
       
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting target handle ');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.MotorHandle{t},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
                
            end
        
            [res,obj.camhandle]=obj.vrep.simxGetObjectHandle(obj.clientID,obj.camara,obj.vrep.simx_opmode_oneshot_wait);
           
            
        end

        function  [error] = setJointPosition(obj,position,MotorNumber,pub_joint_single,msg_joint_single)

            
            
            msg_joint_single.Name= obj.Motor_Name{MotorNumber};% indicacao de qual junta sera alterada
            msg_joint_single.Cmd= [deg2rad(position)];% indicacao do angulo a atribuir
            send(pub_joint_single, msg_joint_single);% publicacao no topico

            res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.MotorHandle{MotorNumber},deg2rad(position), obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending waistangle data!');
                error = 1;
                return;
            end
        end

        function  [error] = close_gripper(obj,pub_joint_single,msg_joint_single) 
            msg_joint_single.Name= 'gripper';% indicacao que a junta a ser alterada e a gripper
            msg_joint_single.Cmd= -100;% valor de PWM para a junta(PWM negativo fecha a gripper)
            send(pub_joint_single, msg_joint_single);% publicacao no topico
            
            res = obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.MotorHandle{5},0.015, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending close command to gripper(left finger)!'); %por posiçao   dois handles left right
                error = 1;
                return;
            end
            res = obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.MotorHandle{6},-0.015, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending close command to gripper(right finger)!'); %por posiçao   dois handles left right
                error = 1;
                return;
            end
       end

        function  [error] = open_gripper(obj,pub_joint_single,msg_joint_single)
            
            msg_joint_single.Name= 'gripper';% indicacao que a junta a ser alterada e a gripper
            msg_joint_single.Cmd= 90;% valor de PWM para a junta(PWM positivo abre a gripper)
            send(pub_joint_single, msg_joint_single);% publicacao no topico
            
             res = obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.MotorHandle{5},0.047, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending close command to gripper(left finger)!'); 
                error = 1;
                return;
            end
            res = obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.MotorHandle{6},-0.047, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending close command to gripper(right finger)!');
                error = 1;
                return;
            end
       end

        function  [error,JointAngle,msg] = getJointAngle(obj,MotorNumber,sub_joint_position)
            
            [res,JointAnglerad] = obj.vrep.simxGetJointPosition(obj.clientID, obj.MotorHandle{MotorNumber}, obj.vrep.simx_opmode_streaming);
            JointAngle=rad2deg(JointAnglerad);
            msg = receive(sub_joint_position);% leitura do topico que retorna todos os angulos atuais das juntas
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending waistangle data!');
                error = 1;
                return;
            else
                error=0;
            end
        end

        function [error,Position] = getJointPosition( obj, MotorNumber)

            error = 0;
            [res,Position] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.MotorHandle{MotorNumber},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting target position!');
                error = 1;
                return;
            end
       end

        function [error,resolution,img,posicoes]=getvision(obj)
        
        [error,resolution,img]=obj.vrep.simxGetVisionSensorImage2(obj.clientID,obj.camhandle,0, obj.vrep.simx_opmode_oneshot_wait);%recolhe a imagem do vision sensor
        error=0;
        posicoes=zeros(3,3);% cria a matriz de posicoes
        figure(1),imshow(img);
        [r,g,b]=imsplit(img);% divide a imagem por cores
        %figure(2),imshow(r);
        %figure(3),imshow(g);
        %figure(4),imshow(b);
        n_r=im2bw(r,0.65);% define o threshold para separar o preto do branco
        %figure(5),imshow(n_r);
        n_g=im2bw(g,0.65);% define o threshold para separar o preto do branco
        %figure(6),imshow(n_g);
        edge_r=edge(n_r,'roberts');% cria bounding boxes a volta dos objetos detetados
        %figure(7),imshow(edge_r);
        stats_r=regionprops(edge_r,'all');% cria uma matriz com diversas informacoes sobre os objetos detetados
        edge_g=edge(n_g,'roberts');% cria bounding boxes a volta dos objetos detetados
        %figure(8),imshow(edge_g);
        stats_g=regionprops(edge_g,'all');% cria uma matriz com diversas informacoes sobre os objetos detetados
        centroids_r = cat(1,stats_r.Centroid);% guarda os centroides
        if(size(centroids_r,2)>0)
        figure(9),imshow(edge_r);
        hold on
        plot(centroids_r(:,1),centroids_r(:,2),'b*');% representacao dos centroides na imagem
        hold off
        end
        centroids_g = cat(1,stats_g.Centroid);% guarda os centroides dos objetos
        figure(10),imshow(edge_g);
        if(size(centroids_g,2)>0)
        hold on
        plot(centroids_g(:,1),centroids_g(:,2),'b*')% representacao dos centroides na imagem
        hold off 
        end

        convex_area_r = cat(1,stats_r.ConvexArea);% guarda a convex area dos objetos
        filled_area_r = cat(1,stats_r.FilledArea);% guarda a filled area dos objetos
        i_r=1;
        linhas_r= size(stats_r,1);%numero de objtos listados
        
           while(i_r<=linhas_r)              
               if(convex_area_r(i_r)==filled_area_r(i_r))% distingue se e cubo ou cilindro
                        
                       x_cubo_vermelho=-0.0015*centroids_r(i_r,1)+0.1882;%converte as coordenadas do vision para coordenadas do coppelia(x)
                       y_cubo_vermelho=0.0015*centroids_r(i_r,2)+0.387;%converte as coordenadas do vision para coordenadas do coppelia(y)
                       fprintf('cubo vermelho\n')
                       fprintf('x = %2f y = %2f\n',x_cubo_vermelho,y_cubo_vermelho);
                       % guarda as posicoes
                       posicoes(1,1)=x_cubo_vermelho;
                       posicoes(2,1)=y_cubo_vermelho;
                       posicoes(3,1)=0.7711; %define a altura 

               end
               i_r=i_r+1;
           end
                  
        convex_area_g = cat(1,stats_g.ConvexArea);% guarda a convex area dos objetos
        filled_area_g = cat(1,stats_g.FilledArea);% guarda a filled area dos objetos
        i_g=1;
        linhas_g= size(stats_g,1);%numero de objtos listados
        
           while(i_g<=linhas_g)              
               if(convex_area_g(i_g)==filled_area_g(i_g))% distingue se e cubo ou cilindro

                    x_cubo_verde=-0.0015*centroids_g(i_g,1)+0.1882;%converte as coordenadas do vision para coordenadas do coppelia(x)
                    y_cubo_verde=0.0015*centroids_g(i_g,2)+0.387;%converte as coordenadas do vision para coordenadas do coppelia(y)
                    fprintf('cubo verde\n')
                    fprintf('x = %2f y = %2f\n',x_cubo_verde,y_cubo_verde);
                    % guarda as posicoes
                    posicoes(1,2)=x_cubo_verde;
                    posicoes(2,2)=y_cubo_verde;
                    posicoes(3,2)=0.7711;%define a altura

               end
              
               if(convex_area_g(i_g)>filled_area_g(i_g))% distingue se e cubo ou cilindro
                   
                    x_cilindro_verde=-0.0015*centroids_g(i_g,1)+0.1882;%converte as coordenadas do vision para coordenadas do coppelia(x)
                    y_cilindro_verde=0.0015*centroids_g(i_g,2)+0.387;%converte as coordenadas do vision para coordenadas do coppelia(y)
                    fprintf('cilindro verde\n')
                    fprintf('x = %2f y = %2f\n',x_cilindro_verde,y_cilindro_verde);
                    % guarda as posicoes
                    posicoes(1,3)=x_cilindro_verde;
                    posicoes(2,3)=y_cilindro_verde;
                    posicoes(3,3)=0.7711;%define a altura

               end
           i_g=i_g+1;
           end
        return
        end

        function [error,angulos]=placeobject_real(obj,objecttype,posicoes,posicoes_tipo,pub_joint_single,msg_joint_single,W_forma,W_cor)

            [angulo_base]=obj.base(posicoes_tipo(1,objecttype),W_forma,W_cor); %angulo atribuido à waist para pousar na base correta

            load('Rede_inversa_real.mat');
            error=0;
            angulos=net3(posicoes(:,objecttype));% usa a rede neurunal inversa para calcular os angulos para a posicao do objeto
            obj.open_gripper(pub_joint_single,msg_joint_single);% abre a gripper
            pause(3);
            % define os angulos das juntas
            obj.setJointPosition(angulos(1,1),1,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(angulos(3,1),3,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(angulos(4,1),4,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(angulos(2,1),2,pub_joint_single,msg_joint_single);
            pause(1);
            obj.close_gripper(pub_joint_single,msg_joint_single);% fecha a gripper
            pause(10);
            obj.setJointPosition(0,2,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(angulo_base,1,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(-40,3,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(-26,4,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(70,2,pub_joint_single,msg_joint_single);
            pause(10);
            obj.open_gripper(pub_joint_single,msg_joint_single);% abre a gripper
            pause(10);
            obj.setJointPosition(0,2,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(0,3,pub_joint_single,msg_joint_single);
            obj.setJointPosition(0,4,pub_joint_single,msg_joint_single);

        end
        
        function[error]=desligar_braco(obj,pub_joint_single,msg_joint_single)
            % funcao que coloca o braco na posicao de descanso
            error=0;
            pause(1);
            obj.setJointPosition(0,1,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(-111,2,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(92,3,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(45,4,pub_joint_single,msg_joint_single);

        end

        function[error]=home_braco(obj,pub_joint_single,msg_joint_single)
            % funcao que coloca o braco na posicoa inicial
            error=0;
            pause(1);
            obj.setJointPosition(0,1,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(0,3,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(0,4,pub_joint_single,msg_joint_single);
            pause(1);
            obj.setJointPosition(0,2,pub_joint_single,msg_joint_single);
            pause(1);
            

        end

        function [error,img,posicoes,posicoes_tipo,n]=getvision_real(obj)
        error=0;
        posicoes=zeros(3,:);% cria a matriz de posicoes
        posicoes_tipo=zeros(1,:);% cria a matriz de posicoes_tipo
        cam=webcam(2);%define a camara que sera utilizada
        img=snapshot(cam);%recolhe a imagem da camara
        img = imresize(img,[256 256]);% resize da imagem da camara
        figure(1),imshow(img);
        [r,g,~]=imsplit(img);% divide a imagem por cores
        %figure(2),imshow(r);
        %figure(3),imshow(g);
        %figure(4),imshow(b);
        n_r=im2bw(r,0.85);% define o threshold para separar o preto do branco
        figure(5),imshow(n_r);
        n_g=im2bw(g,0.9);% define o threshold para separar o preto do branco
        figure(6),imshow(n_g);
        edge_r=edge(n_r,'roberts');% cria bounding boxes a volta dos objetos detetados
        %figure(7),imshow(edge_r);
        stats_r=regionprops(edge_r,'all');% cria uma matriz com diversas informacoes sobre os objetos detetados
        edge_g=edge(n_g,'roberts');% cria bounding boxes a volta dos objetos detetados
        %figure(8),imshow(edge_g);
        stats_g=regionprops(edge_g,'all');% cria uma matriz com diversas informacoes sobre os objetos detetados
        centroids_r = cat(1,stats_r.Centroid);% guarda os centroides dos objetos
        figure(9),imshow(edge_r);
        if(size(centroids_r,2)>0)
        hold on
        plot(centroids_r(:,1),centroids_r(:,2),'b*');% representacao dos centroides na imagem
        hold off
        end
        centroids_g = cat(1,stats_g.Centroid);% guarda os centroides dos objetos
        figure(10),imshow(edge_g);
        if(size(centroids_g,2)>0)
        hold on
        plot(centroids_g(:,1),centroids_g(:,2),'b*')% representacao dos centroides na imagem
        hold off 
        end

       
        convex_area_r = cat(1,stats_r.ConvexArea);% guarda a convex area dos objetos
        bounding_box_r = cat(1,stats_r.BoundingBox);% guarda a filled area dos objetos
        i_r=1;
        n=0;
        linhas_r= size(stats_r,1);%numero de objtos listados
        
           while(i_r<=linhas_r)
               
               if(convex_area_r(i_r)>20)%filtra os objetos para nao apanhar objetos pequenos
                    area_r=bounding_box_r(i_r,3)*bounding_box_r(i_r,4);% calcula area dos objetos
 
                    if(area_r-convex_area_r(i_r)<50)% distingue se e cubo ou cilindro

                       n=n+1;
                       posicoes(1,n)=0.001828977843786*centroids_r(i_r,1)-0.226942392708475;%converte as coordenadas do vision para coordenadas do coppelia(x)
                       posicoes(2,n)=-0.001034944480582*centroids_r(i_r,2)+0.736223935566372;%converte as coordenadas do vision para coordenadas do coppelia(y)
                       posicoes(3,n)=0.01;% define a altura
                       posicoes_tipo(1,n)=1;% define os tipo de objeto

                    end
               end
               i_r=i_r+1;
              
           end
                  
        convex_area_g = cat(1,stats_g.ConvexArea);% guarda a convex area dos objetos
        bounding_box_g = cat(1,stats_g.BoundingBox);% guarda a filled area dos objetos
        i_g=1;
        
        linhas_g= size(stats_g,1);%numero de objtos listados
        
           while(i_g<=linhas_g)  
                if(convex_area_g(i_g)>20)%filtra os objetos para nao apanhar objetos pequenos
                    area_g=bounding_box_g(i_g,3)*bounding_box_g(i_g,4);% calcula area dos objetos
                    if(area_g-convex_area_g(i_g)<50)% distingue se e cubo ou cilindro
                        n=n+1;  
                        posicoes(1,n)=0.001828977843786*centroids_g(i_g,1)-0.226942392708475;%converte as coordenadas do vision para coordenadas do coppelia(x)
                        posicoes(2,n)=-0.001034944480582*centroids_g(i_g,2)+0.736223935566372;%converte as coordenadas do vision para coordenadas do coppelia(y)
                        posicoes(3,n)=0.01;%define a altura
                        posicoes_tipo(1,n)=2;% define os tipo de objeto
                            
                    end
              
                    if(area_g-convex_area_g(i_g)>50)% distingue se e cubo ou cilindro
                        n=n+1;
                        posicoes(1,n)=0.001828977843786*centroids_g(i_g,1)-0.226942392708475;%converte as coordenadas do vision para coordenadas do coppelia(x)
                        posicoes(2,n)=-0.001034944480582*centroids_g(i_g,2)+0.736223935566372;%converte as coordenadas do vision para coordenadas do coppelia(y)
                        posicoes(3,n)=0.01;%define a altura
                        posicoes_tipo(1,n)=3;% define os tipo de objeto

                    end
                end
                i_g=i_g+1;

           end

           % ciclo que mostra os objetos disponiveis ao utilizador

           for i=1:n
               fprintf("%d-",i)
               if posicoes_tipo(1,i)==1
                  disp("CUBO VERMELHO");
               elseif posicoes_tipo(1,i)==2
                  disp("CUBO VERDE");
               elseif posicoes_tipo(1,i)==3
                  disp("CILINDRO VERDE");
               end
           end
                    

        end

        function[angulo]=base(obj,tipo,W_forma,W_cor)
            % funcao que calcula o angulo da waist para a base
            % correspondente

            switch tipo
                case 1 % cubo vermelho
                    output_tamanho=DNF_tamanho('l',W_forma);
                    output_cor=DNF_cor('r',W_cor);
                    angulo=DNF_posicao(output_tamanho,output_cor);
                case 2 % cubo verde
                    output_tamanho=DNF_tamanho('l',W_forma);
                    output_cor=DNF_cor('g',W_cor);
                    angulo=DNF_posicao(output_tamanho,output_cor);
                case 3 % cilindro verde
                    output_tamanho=DNF_tamanho('s',W_forma);
                    output_cor=DNF_cor('g',W_cor);
                    angulo=DNF_posicao(output_tamanho,output_cor);
            end
        end
        
    end
end
       
    
    


