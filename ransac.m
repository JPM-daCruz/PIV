function [ rfinal, tfinal ] = ransac( im1, im2, depth1, depth2, cam_params )
%UNTITLED5 Esta função recebe as duas imagens, rgb e produndidade das duas
%câmaras, e os parametros intrinsecos da câmara e calcula a matriz de
%rotação e translação

numero_ransacs=700;
tres=0.2;

xyz_1=get_xyzasus(depth1(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);
xyz_2=get_xyzasus(depth2(:), [480 640], find(depth1(:)>0), cam_params.Kdepth, 1, 0);

cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
p1=pointCloud(xyz_1,'Color',cl1);
p2=pointCloud(xyz_2,'Color',cl2);
%% conversão das coordenadas xyz de cada ponto da depth, para indices u,v da
%imagem rgb
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

%% Deteção de pontos de interesse e match dos pontos de interesse entre imagens
[f1,d1] = vl_sift(single(rgb2gray(im1)));
[f2,d2] = vl_sift(single(rgb2gray(im2)));
[matches, scores] = vl_ubcmatch(d1, d2);

%% indices u e v dos ponto de rgb onde houve match
for i=1:length(matches)
    uv1(:,i)=f1(1:2,matches(1,i));
    uv2(:,i)=f2(1:2,matches(2,i));    
end

%melhor correspondência entre os pontos detetados pelo sift e aqueles que
%nós conhecemos as coordenadas xyz
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
    [d, z, transform]=procrustes(xyz11',xyz22','scaling',false,'reflection', false);
    r=transform.T';
    t=transform.c(1,:)';
    for aux1=1:length(matches)
        r21(:,aux1)=r*xyz2(:,aux1)+t;
    end
    for aux1=1:length(matches)
        erros(aux1)=sqrt(((xyz1(1,aux1)-r21(1,aux1))^2)+((xyz1(2,aux1)-r21(2,aux1))^2)+((xyz1(3,aux1)-r21(3,aux1))^2));
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
[d, z, transform]=procrustes(xyz_inliers1',xyz_inliers2','scaling',false,'reflection', false);
rfinal=transform.T';
tfinal=transform.c(1,:)';

end

