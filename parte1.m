%inicialização
clear all;
close all;
clc;

%declaração de variaveis globais
numero_imagens=25;
imagens_background=round(numero_imagens*0.4);
nome_depth='depth';
nome_rgb='rgb_image';
load cameraparametersAsus.mat;
objects(1)=struct('x',[],'y',[],'z',[],'frames_tracked',[]);

%% Cálculo da imagem depth de background para as duas câmaras, e das respectivas coordenadas
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
%tracking e armazena a informação devidamente

for i=1:numero_imagens
    if i==1
        %é a primeira imagem de cada câmara, logo não é preciso comparar os
        %objectos detetados com nada, é só inserir
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
    
        %% Remoção do background
        %Retira o background, retira pontos a que a profundidade seja superior
        %a 6,5m por não serem precisos, e retira pontos em que na imagem de
        %background a profundidade é nula, por não serem pontos válidos
        for aux1=1:480
        for aux2=1:640
            %câmara 1
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
            %câmara 2
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


        %% Deteção de objectos na imagem de profundidade
        [l1, num1]=bwlabel(im_depth1);
        for aux1=1:num1
        [row, col]=find(l1==aux1);
       	if(length(row)<400)
        %significa que não tem pontos suficientes, logo não é objecto
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
            %significa que nao tem pontos suficientes, logo não é objecto
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
        %Filtro para garantir que não há pontos incorretamente medidos na
        %extremidade dos objectos que levem, a uma incorreta obtenção da caixa
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
    
        %% Cálculo das caixas para cada um dos objectos detetados nas duas imagens
        %este ciclo percorre todos os objectos encontrados na imagem 1 e calcula a caixa no
        %qual cada um está incluido. os valores limites da caixa devem depois
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
        %calcula a caixa no qual cada objecto está incluido. Os valores limites
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
        %fim da análise da imagem1
        for aux1=1:num1
            objects(aux1).x=[xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1)];
            objects(aux1).y=[ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1), ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1)];
            objects(aux1).z=[zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1)];
            objects(aux1).frames_tracked=1;
        end
    else
        %a partir daqui já temos de comparar a imagem em questão com a
        %última imagem para proceder ao tracking de objectos
    end
  
end