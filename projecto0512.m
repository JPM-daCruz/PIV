%Iniciação, e limpeza do matlab
clear all;
close all;
clc;

%isto é para substituir quando for para entregar o projecto
numero_imagens=25;
imagens_background=20;
tracked_objs = cell(1,[]);
nome_depth='depth';
nome_rgb='rgb_image';
load cameraparametersAsus.mat;

%% Cálculo da imagem de background
r=randi([1 numero_imagens],1,imagens_background);
array1=zeros(480, 640, imagens_background);
array2=zeros(480, 640, imagens_background); 
for i=1:length(r)
    nome1=strcat(nome_depth,num2str(1),'_',num2str(r(i)),'.mat');
    nome2=strcat(nome_depth,num2str(2),'_',num2str(r(i)),'.mat');
    load(nome1);
    array1(:,:,i)=depth_array;
    load(nome2);
    array2(:,:,i)=depth_array;
end
background1=median(array1,3);
background2=median(array2,3);
imagesc(background1);
clear array1 array2 nome1 nome2 depth_array;
xyz_background1=get_xyzasus(background1(:), [480 640], find(background1(:)>0), cam_params.Kdepth, 1, 0);
xyz_background2=get_xyzasus(background2(:), [480 640], find(background2(:)>0), cam_params.Kdepth, 1, 0);
%neste momento temos guardadas as imagens de background, as suas
%coordenadas, o seu rgb, o seu depth, tudo
%% Ciclo principal do projecto
for i=1:numero_imagens
	nome1=strcat(nome_depth,num2str(1),'_',num2str(i),'.mat');
    load(nome1);
    depth1=depth_array;
    rgb1=strcat(nome_rgb,num2str(1),'_',num2str(i),'.png');
    im1=imread(rgb1);
    nome2=strcat(nome_depth,num2str(2),'_',num2str(i),'.mat');    
    load(nome2);
    depth2=depth_array;
    rgb2=strcat(nome_rgb,num2str(2),'_',num2str(i),'.png');
    im2=imread(rgb2);
    xyz_d1=get_xyzasus(depth1(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);
    xyz_d2=get_xyzasus(depth2(:), [480 640], find(depth2(:)>0), cam_params.Kdepth, 1, 0);
    %% mapeamento das coordenadas de um ponto depth na imagem RGB
    RT=horzcat(cam_params.R, cam_params.T);
    homogeneas1(1,:)=xyz_d1(:,1);
    homogeneas1(2,:)=xyz_d1(:,2);
    homogeneas1(3,:)=xyz_d1(:,3);
    homogeneas1(4,:)=1;
    lambda_u_v1=cam_params.Krgb*RT*homogeneas1;
    u_v1(1,:)=lambda_u_v1(1,:)./lambda_u_v1(3,:);
    u_v1(2,:)=lambda_u_v1(2,:)./lambda_u_v1(3,:);
    
    homogeneas2(1,:)=xyz_d2(:,1);
    homogeneas2(2,:)=xyz_d2(:,2);
    homogeneas2(3,:)=xyz_d2(:,3);
    homogeneas2(4,:)=1;
    lambda_u_v1=cam_params.Krgb*RT*homogeneas1;
    u_v1(1,:)=lambda_u_v1(1,:)./lambda_u_v1(3,:);
    u_v1(2,:)=lambda_u_v1(2,:)./lambda_u_v1(3,:);    
    
    %% Remoção do background
    for aux1=1:480
       for aux2=1:640
           if (abs(depth1(aux1,aux2)-background1(aux1,aux2))<150)
                depth1(aux1,aux2)=0;
           end
           if (background1(aux1,aux2)==0)
               depth1(aux1,aux2)=0;
           end 
           if (depth1(aux1,aux2)>6000)
                depth1(aux1,aux2)=0;
           end
           if (abs(depth2(aux1,aux2)-background2(aux1,aux2))<150)
                depth2(aux1,aux2)=0;
           end
           if (background2(aux1,aux2)==0) 
               depth2(aux1,aux2)=0;
           end
           if (depth2(aux1,aux2)>6000)
              depth2(aux1,aux2)=0; 
           end
       end
    end
    clear aux1 aux2;
    figure(1);
    imagesc(depth1);
    %coordenadas das imagens atuais
    xyz_1=get_xyzasus(depth1(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);
    xyz_2=get_xyzasus(depth2(:), [480 640], find(depth2(:)>0), cam_params.Kdepth, 1, 0);
    
    rgbd2=get_rgbd(xyz_2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd1=get_rgbd(xyz_1, im1, cam_params.R, cam_params.T, cam_params.Krgb);

    cl1=reshape(rgbd1,480*640,3);
    cl2=reshape(rgbd2,480*640,3);
   
    p1=pointCloud(xyz_1,'Color',cl1);
    p2=pointCloud(xyz_2,'Color',cl2);
    %figure(3);
    %showPointCloud(p1);
    %figure(4);
    %showPointCloud(p2);
    
    [L1, num1]=bwlabel(depth1);
    [L2, num2]=bwlabel(depth2);
    %% Remove pequenos erros da imagem ao garantir que para ser considerado
    %% forground tem de ter pelo menos 400 pontos no rgb
    for aux1=1:num1
       [row, col, v] = find(L1==aux1);
       if(length(v)<400)
            for aux2=1:length(row)
                L1(row, col)=0;
            end
       end
    end
    clear row col v;
    for aux1=1:num2
        [row, col, v] = find(L2==aux1);
        if(length(v)<400)
            for aux2=1:length(row)
                L2(row, col)=0;
            end
       end 
    end

    [L1, num1] = bwlabel(L1);%Repoem os valores da label para 0,1,2 em vez de 400
    [L2, num2] = bwlabel(L2);
    figure(6);
    imagesc([L1 L2]);
    pause(0.5);
    
    stat1 = regionprops(L1,'centroid', 'Area');
    stat2 = regionprops(L2,'centroid', 'Area');

   

end

