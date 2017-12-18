%Iniciação, e limpeza do matlab
clear;
close all;
clc;
nome_depth='depth';
nome_rgb='rgb_image';
load cameraparametersAsus.mat
numero_ransacs=100;
tres=0.3;
%%
nome1=strcat(nome_depth,num2str(1),'_',num2str(1),'.mat');
load(nome1);
depth1=depth_array;
rgb1=strcat(nome_rgb,num2str(1),'_',num2str(1),'.png');
im1=imread(rgb1);
nome2=strcat(nome_depth,num2str(2),'_',num2str(1),'.mat');    
load(nome2);
depth2=depth_array;
rgb2=strcat(nome_rgb,num2str(2),'_',num2str(1),'.png');
im2=imread(rgb2);
xyz_1=get_xyzasus(depth1(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);
xyz_2=get_xyzasus(depth2(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);
clear nome1 nome2 depth_array;

cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
p1=pointCloud(xyz_1,'Color',cl1);
p2=pointCloud(xyz_2,'Color',cl2);

RT=horzcat(cam_params.R, cam_params.T);
homogeneas1(1,:)=xyz_1(:,1);
homogeneas1(2,:)=xyz_1(:,2);
homogeneas1(3,:)=xyz_1(:,3);
homogeneas1(4,:)=1;
lambda_u_v1=cam_params.Krgb*RT*homogeneas1;
u_v1(1,:)=lambda_u_v1(1,:)./lambda_u_v1(3,:);
u_v1(2,:)=lambda_u_v1(2,:)./lambda_u_v1(3,:);

homogeneas2(1,:)=xyz_2(:,1);
homogeneas2(2,:)=xyz_2(:,2);
homogeneas2(3,:)=xyz_2(:,3);
homogeneas2(4,:)=1;
lambda_u_v2=cam_params.Krgb*RT*homogeneas2;
u_v2(1,:)=lambda_u_v2(1,:)./lambda_u_v2(3,:);
u_v2(2,:)=lambda_u_v2(2,:)./lambda_u_v2(3,:);


[f1,d1] = vl_sift(single(rgb2gray(im1)));
[f2,d2] = vl_sift(single(rgb2gray(im2)));
[matches, scores] = vl_ubcmatch(d1, d2);

% Indices d u e v da imagem rgb associados aos matches
for i=1:length(matches)
    uv1(:,i)=f1(1:2,matches(1,i));
    uv2(:,i)=f2(1:2,matches(2,i));    
end

%calculo da distancia euclidiana
for i=1:length(matches)
    euclideana1=sqrt((uv1(1,i)-u_v1(1,:)).^2+(uv1(2,i)-u_v1(2,:)).^2);
    euclideana2=sqrt((uv2(1,i)-u_v2(1,:)).^2+(uv2(2,i)-u_v2(2,:)).^2);
    [M1, I1]=min(euclideana1);
    [M2, I2]=min(euclideana2);
    xyz1(i,:)=xyz_1(I1,:);
    xyz2(i,:)=xyz_2(I2,:);
    dist1(i)=M1;
    dist2(i)=M2;
end
xyz1=xyz1';
xyz2=xyz2';
mais_inliers=0;
melhores_inliers=[];
for i=1:numero_ransacs
    four_points = randperm(length(matches),4); 
    xyz11=xyz1(:,four_points);
    xyz22=xyz2(:,four_points);
    [d, z, transform]=procrustes(xyz22',xyz11','scaling',false,'reflection', false);
    r=transform.T';
    t=transform.c(1,:)';
    for aux1=1:length(matches)
        r12(:,aux1)=r*xyz1(:,aux1)+t;
    end
    for aux1=1:length(matches)
        erros(aux1)=sqrt(((xyz2(1,aux1)-r12(1,aux1))^2)+((xyz2(2,aux1)-r12(2,aux1))^2)+((xyz2(3,aux1)-r12(3,aux1))^2));
    end
    indices=find(erros<tres);
    inliers=length(indices);
    if inliers>mais_inliers
       mais_inliers=inliers;
       clear melhores_inliers;
       melhores_inliers=indices;
    end
end
xyz_inliers1=xyz1(:,melhores_inliers);
xyz_inliers2=xyz2(:,melhores_inliers);
[d, z, transform]=procrustes(xyz_inliers2',xyz_inliers1','scaling',false,'reflection', false);
rfinal=transform.T';
tfinal=transform.c(1,:)';
xyz_1=xyz_1';
for i=1:length(xyz_1)
   xyz_2r(i,:)=rfinal*xyz_1(:,i)+tfinal;
end

ptotal=pointCloud([xyz_2r;xyz_2],'Color',[cl1;cl2]);
showPointCloud(ptotal);
