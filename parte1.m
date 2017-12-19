%inicializa��o
clear all;
close all;
clc;

%declara��o de variaveis globais
numero_imagens=25;
imagens_background=round(numero_imagens*0.4);
nome_depth='depth';
nome_rgb='rgb_image';
load cameraparametersAsus.mat;
objects(1)=struct('x',[],'y',[],'z',[],'frames_tracked',[]);

%% C�lculo da imagem depth de background para as duas c�maras, e das respectivas coordenadas
back1=zeros(480,640, imagens_background);
back2=zeros(480,640, imagens_background);
i=1;
while(i<=imagens_background)
    nome1_depth=strcat(nome_depth,num2str(1),'_',num2str(i+2),'.mat');
    nome2_depth=strcat(nome_depth,num2str(2),'_',num2str(i+2),'.mat');
    load(nome1_depth);
    back1(:,:,i)=depth_array;
    load(nome2_depth);
    back2(:,:,i)=depth_array;
	i = (i+1);
end
back1=median(back1,3);
back2=median(back2,3);
figure(1)
imagesc([back1, back2]);
xyz_background1=get_xyzasus(back1(:), [480 640], find(back1(:)>0), cam_params.Kdepth, 1, 0);
xyz_background2=get_xyzasus(back2(:), [480 640], find(back2(:)>0), cam_params.Kdepth, 1, 0);
clear nome1_depth nome2_depth i imagens_background;

%% Ciclo principal do projecto
%Este ciclo percorre todas as imagens, deteta os objectos, efetua o seu
%tracking e armazena a informa��o devidamente

for i=1:numero_imagens
    if i==1
        %� a primeira imagem de cada c�mara, logo n�o � preciso comparar os
        %objectos detetados com nada, � s� inserir
        nome1_depth=strcat(nome_depth,num2str(1),'_',num2str(i),'.mat');
        nome2_depth=strcat(nome_depth,num2str(2),'_',num2str(i),'.mat');
        nome1_rgb=strcat(nome_rgb,num2str(1),'_',num2str(i),'.png');
        nome2_rgb=strcat(nome_rgb,num2str(2),'_',num2str(i),'.png');
        load(nome1_depth);
        im_depth1=depth_array;
        load(nome2_depth);
        im_depth2=depth_array;
        im_rgb1=imread(nome1_rgb);
        im_rgb2=imread(nome2_rgb);
        %figure(2);
        %imagesc([im_rgb1, im_rgb2]);
        figure(3);
        imagesc([im_depth1, im_depth2]);
        xyz_original_1=get_xyzasus(im_depth1(:), [480 640], find(im_depth1(:)>0), cam_params.Kdepth, 1, 0);
        xyz_original_2=get_xyzasus(im_depth2(:), [480 640], find(im_depth2(:)>0), cam_params.Kdepth, 1, 0);
    
        %% Remo��o do background
        %Retira o background, retira pontos a que a profundidade seja superior
        %a 6,5m por n�o serem precisos, e retira pontos em que na imagem de
        %background a profundidade � nula, por n�o serem pontos v�lidos
        for aux1=1:480
        for aux2=1:640
            %c�mara 1
            %if aux1==129 && aux2==523
            %    debug=1;
            %end
            if ((back1(aux1,aux2)-im_depth1(aux1,aux2))<=250)
                im_depth1(aux1,aux2)=0;
            end
            if (im_depth1(aux1, aux2)>6500)
                im_depth1(aux1, aux2)=0;
            end
            if (back1(aux1,aux2)==0)
                im_depth1(aux1,aux2)=0;
            end
            %c�mara 2
            if ((back2(aux1,aux2)-im_depth2(aux1,aux2))<=250)
                im_depth2(aux1,aux2)=0;
            end
            if (im_depth2(aux1, aux2)>6500)
                im_depth2(aux1, aux2)=0;
            end
            if (back2(aux1,aux2)==0)
                im_depth2(aux1,aux2)=0;
            end
            
        end
        end
        figure(4);
        imagesc([im_depth1, im_depth2]);
        xyz_backremoved1=get_xyzasus(im_depth1(:), [480 640], find(im_depth1(:)>0), cam_params.Kdepth, 1, 0);
        xyz_backremoved2=get_xyzasus(im_depth2(:), [480 640], find(im_depth2(:)>0), cam_params.Kdepth, 1, 0);
    
        rgbd1=get_rgbd(xyz_backremoved1, im_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
        rgbd2=get_rgbd(xyz_backremoved2, im_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
        cl1=reshape(rgbd1,480*640,3);
        cl2=reshape(rgbd2,480*640,3);
        p1=pointCloud(xyz_backremoved1,'Color',cl1);
        p2=pointCloud(xyz_backremoved2,'Color',cl2);


        %% Dete��o de objectos na imagem de profundidade
        [l1, num1]=bwlabel(im_depth1);
        for aux1=1:num1
        [row, col]=find(l1==aux1);
       	if(length(row)<400)
        %significa que n�o tem pontos suficientes, logo n�o � objecto
            for aux2=1:length(row)
                l1(row(aux2), col(aux2))=0;
            end
        end
        end
        [l1, num1]=bwlabel(l1);
    
        [l2, num2]=bwlabel(im_depth2);
        for aux1=1:num2
        [row, col]=find(l2==aux1);
        if(length(row)<400)
            %significa que nao tem pontos suficientes, logo n�o � objecto
            for aux2=1:length(row)
                l2(row(aux2), col(aux2))=0;
        	end
        end
        end
        [l2, num2]=bwlabel(l2);
    
        figure(7);
        imagesc([l1, l2]);
        figure(8);
        imagesc([im_rgb1, im_rgb2]);
        %Filtro para garantir que n�o h� pontos incorretamente medidos na
        %extremidade dos objectos que levem, a uma incorreta obten��o da caixa
        l1=imclose(l1, strel('disk',10));
        l2=imclose(l2, strel('disk',10));
    
        %% Filtro do gradiente
        %Este filtro tem por objectivo detetar objectos que estejam parcialemte
        %sobrepostos, para que cada um seja tido em conta separadamente do
        %outro
        [fx1, fy1]=gradient(mat2gray(im_depth1));
        [fx2, fy2]=gradient(mat2gray(im_depth2));
        G1=(fx1.^2 + fy1.^2);%>((250)^2);
        G2=(fx2.^2 + fy2.^2);%>((250)^2);
        im_depth1=double(im_depth1);
        im_depth2=double(im_depth2);
        figure(9);
        imagesc([im_depth1, G1]);
    
        %% C�lculo das caixas para cada um dos objectos detetados nas duas imagens
        %este ciclo percorre todos os objectos encontrados na imagem 1 e calcula a caixa no
        %qual cada um est� incluido. os valores limites da caixa devem depois
        %ser armazenados na estrutura correspondnete ao tracking
        xmax_1=zeros(1,num1);
        ymax_1=zeros(1,num1);
        zmax_1=zeros(1,num1);
        xmin_1=repmat(6,1,num1);
        ymin_1=repmat(6,1,num1);
        zmin_1=repmat(6,1,num1);
        for aux1=1:num1
            [row, col]=find(l1==aux1);
            for aux2=1:length(row)
                linear=sub2ind([480 640],row(aux2),col(aux2));
                if(xyz_original_1(linear,1)>xmax_1(1,aux1))%xmax
                    xmax_1(1,aux1)=xyz_original_1(linear,1);
                end
                if(xyz_original_1(linear,1)<xmin_1(1,aux1))%xmin
                    xmin_1(1,aux1)=xyz_original_1(linear,1);
                end
                if(xyz_original_1(linear,2)>ymax_1(1,aux1))%ymax
                    ymax_1(1,aux1)=xyz_original_1(linear,2);
                end
                if(xyz_original_1(linear,2)<ymin_1(1,aux1))%ymin
                    ymin_1(1,aux1)=xyz_original_1(linear,2);
                end
                if(xyz_original_1(linear,3)>zmax_1(1,aux1))%zmax
                    zmax_1(1,aux1)=xyz_original_1(linear,3);
                end
                if(xyz_original_1(linear,3)<zmin_1(1,aux1))%zmin
                    zmin_1(1,aux1)=xyz_original_1(linear,3);
                end
            end
        end
        %este ciclo percorre todos os objectos encontrados na imagem 2 e
        %calcula a caixa no qual cada objecto est� incluido. Os valores limites
        %da caixa devem depois ser armazenados para posteriormente serem
        %colocados na estrutura dos objectos detetados
        xmax_2=zeros(1,num2);
        ymax_2=zeros(1,num2);
        zmax_2=zeros(1,num2);
        xmin_2=repmat(6,1,num2);
        ymin_2=repmat(6,1,num2);
        zmin_2=repmat(6,1,num2);
        for aux1=1:num2
            [row, col]=find(l2==aux1);
            for aux2=1:length(row)
                linear=sub2ind([480 640],row(aux2),col(aux2));
                if(xyz_original_2(linear,1)>xmax_2(1,aux1))%xmax
                    xmax_2(1,aux1)=xyz_original_2(linear,1);
                end
                if(xyz_original_2(linear,1)<xmin_2(1,aux1))%xmin
                    xmin_2(aux1)=xyz_original_2(linear,1);
                end
                if(xyz_original_2(linear,2)>ymax_2(1,aux1))%ymax
                    ymax_2(1,aux1)=xyz_original_2(linear,2);
                end
                if(xyz_original_2(linear,2)<ymin_2(1,aux1))%ymin
                    ymin_2(1,aux1)=xyz_original_2(linear,2);
                end
                if(xyz_original_2(linear,3)>zmax_2(1,aux1))%zmax
                    zmax_2(1,aux1)=xyz_original_2(linear,3);
                end
                if(xyz_original_2(linear,3)<zmin_2(1,aux1))%zmin
                    zmin_2(1,aux1)=xyz_original_2(linear,3);
                end
            end        
        end
    
        regioes_camara1=num1;
        regioes_camara2=num2;
        %fim da an�lise da imagem1
        for aux1=1:num1
            objects(aux1).x=[xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1)];
            objects(aux1).y=[ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1), ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1)];
            objects(aux1).z=[zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1)];
            objects(aux1).frames_tracked=1;
        end
    else
        %a partir daqui j� temos de comparar a imagem em quest�o com a
        %�ltima imagem para proceder ao tracking de objectos
    end
  
end







function [ s ] = verificar_separacoes( s )
% VERIFICAR_SEPARACOES Esta fun��o tem como objectivo analisar se alguma das
% regi�es se dividiu dando origem a duas ou mais regi�es.
% Come�a por verificar se existe mais do que uma reg�o na imagem e, caso
% existam, verifica se podem pertencer � mesma pessoa. Para isso,
% verifica se a dist�ncia entre cada par de regi�es � inferior a um
% determinado valor. Caso se verifique, assume-se que as v�rias regi�es
% correspondem � mesma pessoa e por isso, faz-se uma conjuga��o de todos
% os centroides, obtendo-se o ponto m�dio, e somam-se as v�rias �reas e
% os v�rios n�meros de pontos.
    
    k = 1;
    ind_apagar = 1;
   
    if length(s) == 1
        % N�o ocorreu nenhuma separa��o em nenhuma regi�o porque apenas
        % existe uma regi�o

    elseif length(s) > 1
        
        for i=1:length(s)
            for j=(i+1):length(s)               
                   
                if norm(s(i).Centroid - s(j).Centroid) < 46
                    array(k,1)=i;
                    array(k,2)=j;
                    array(k,3)=s(i).Area;
                    array(k,4)=s(j).Area;
                    array(k,5)=s(i).N_pontos;
                    array(k,6)=s(j).N_pontos;
                    k = k+1;
                end                  
            end
        end
        
        % Depois de verificadas as separa��es � necess�rio juntar as regi�es.
        % Para isso vai-se juntar as �reas mais pequenas �s maiores.
        if k > 1
            
            [linhas,~] = size(array);
            
            for i=1:linhas    
                if array(i,1) ~= array(i,2)
                    if array(i,3) > array(i,4)
                        control = 1;             
                    else
                        control = 2;                   
                    end      
                
                    % Atribui��o dos valores de �rea e do n�mero de pontos
                    s(array(i,control)).Area = array(i,3)+array(i,4);
                    s(array(i,control)).N_pontos = array(i,5)+array(i,6);

                    % C�lculo do novo centroide corresponder� ao ponto m�dio
                    % entre os dois anteriores
                    dif1_medio = abs(s(array(i,1)).Centroid(1) - s(array(i,2)).Centroid(1))/2;
                    dif2_medio = abs(s(array(i,1)).Centroid(2) - s(array(i,2)).Centroid(2))/2;
                
                    if s(array(i,1)).Centroid(1) > s(array(i,2)).Centroid(1)
                        s(array(i,control)).Centroid(1) = s(array(i,2)).Centroid(1) + dif1_medio;
                    else
                        s(array(i,control)).Centroid(1) = s(array(i,1)).Centroid(1) + dif1_medio;
                    end

                    if s(array(i,1)).Centroid(2) > s(array(i,2)).Centroid(2)
                        s(arfunction [ s ] = verificar_separacoes( s )
% VERIFICAR_SEPARACOES Esta fun��o tem como objectivo analisar se alguma das
% regi�es se dividiu dando origem a duas ou mais regi�es.
% Come�a por verificar se existe mais do que uma reg�o na imagem e, caso
% existam, verifica se podem pertencer � mesma pessoa. Para isso,
% verifica se a dist�ncia entre cada par de regi�es � inferior a um
% determinado valor. Caso se verifique, assume-se que as v�rias regi�es
% correspondem � mesma pessoa e por isso, faz-se uma conjuga��o de todos
% os centroides, obtendo-se o ponto m�dio, e somam-se as v�rias �reas e
% os v�rios n�meros de pontos.
    
    k = 1;
    ind_apagar = 1;
   
    if length(s) == 1
        % N�o ocorreu nenhuma separa��o em nenhuma regi�o porque apenas
        % existe uma regi�o

    elseif length(s) > 1
        
        for i=1:length(s)
            for j=(i+1):length(s)               
                   
                if norm(s(i).Centroid - s(j).Centroid) < 46
                    array(k,1)=i;
                    array(k,2)=j;
                    array(k,3)=s(i).Area;
                    array(k,4)=s(j).Area;
                    array(k,5)=s(i).N_pontos;
                    array(k,6)=s(j).N_pontos;
                    k = k+1;
                end                  
            end
        end
        
        % Depois de verificadas as separa��es � necess�rio juntar as regi�es.
        % Para isso vai-se juntar as �reas mais pequenas �s maiores.
        if k > 1
            
            [linhas,~] = size(array);
            
            for i=1:linhas    
                if array(i,1) ~= array(i,2)
                    if array(i,3) > array(i,4)
                        control = 1;             
                    else
                        control = 2;                   
                    end      
                
                    % Atribui��o dos valores de �rea e do n�mero de pontos
                    s(array(i,control)).Area = array(i,3)+array(i,4);
                    s(array(i,control)).N_pontos = array(i,5)+array(i,6);

                    % C�lculo do novo centroide corresponder� ao ponto m�dio
                    % entre os dois anteriores
                    dif1_medio = abs(s(array(i,1)).Centroid(1) - s(array(i,2)).Centroid(1))/2;
                    dif2_medio = abs(s(array(i,1)).Centroid(2) - s(array(i,2)).Centroid(2))/2;
                
                    if s(array(i,1)).Centroid(1) > s(array(i,2)).Centroid(1)
                        s(arrray(i,control)).Centroid(2) = s(array(i,2)).Centroid(2) + dif2_medio;
                    else
                        s(array(i,control)).Centroid(2) = s(array(i,1)).Centroid(2) + dif2_medio;
                    end
                
                    % Guarda a linha da estrutura que � necess�rio eliminar
                    % num vetor para no fim se eliminar tudo de uma s� vez.
                    if control == 1
                        % Procura quais os �ndices com o mesmo valor para
                        % se poder atribuir o n�mero da nova linha
                        [idx1,idx2] = find(array(:,3:4) == array(i,4));
                        
                        apagar(ind_apagar) = array(i,2);
                        ind_apagar = ind_apagar + 1;
                        
                        % Mudan�a do n�mero da linha
                        for j=1:length(idx1)
                            array(idx1,idx2) = array(i,1);                    
                        end
                        
                    else
                        % Procura quais os �ndices com o mesmo valor para
                        % se poder atribuir o n�mero da nova linha
                        [idx1,idx2] = find(array(:,3:4) == array(i,3));
                        
                        apagar(ind_apagar) = array(i,1);
                        ind_apagar = ind_apagar + 1;
                        
                        % Mudan�a do n�mero da linha
                        for j=1:length(idx1)
                            array(idx1,idx2) = array(i,2);                    
                        end                        
                    end
                end
            end
            
            % Ordena as linhas que s�o necess�rias apagar e de seguida
            % apaga-as.
            apagar = sort(apagar, 'descend');
            for j=1:(ind_apagar-1)                
                s(apagar(j))=[];
            end 
        end             
        
    end

end







