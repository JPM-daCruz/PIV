%%
clear;
close all;
load('cameraparametersAsus.mat');

parte=2;

margem1=0.1;
margem2=0.1;

peso_area=0.4;
peso_distancia=0.6;

detected=struct('xlimits',{},'x_st',{},'ylimits',{},'y_st',{},'zlimits',{},'z_st',{},'huecam1',{},'huecam2',{},'cam1view',{},'cam2view',{},'frames1',{},'frames2',{},'total_frames',{});
objects=struct('x',{},'y',{},'z',{},'frames_tracked',{}, 'centro',{});
tracked_objs = cell(1,[]);


% READ IMAGES and GENERATE POINT CLOUDS
%leitura das imagens e geração das point clouds
diretoria='C:\Users\Diogo\Documents\Faculdade\4to Ano\PIV\pivprojects';
%abertura da diretoria e leituras dos ficheiros disponiveis
cd (diretoria);
data_rgb=dir('*.png');
data_depth=dir('*.mat');
load cameraparametersAsus.mat;

%==========================================================================
%calculo do background
%==========================================================================

d1=dir('depth1*');
d2=dir('depth2*');
r1=dir('rgb_image1_*');
r2=dir('rgb_image2_*');
for i=1:length(d1)
    im1(i).rgb=[r1(i).name];
    im2(i).rgb=[r2(i).name];
    im1(i).depth=[d1(i).name];
    im2(i).depth=[d2(i).name];
    im1(i).imagem=[d1(i).name()]
end
back_rgb1=zeros(480,640,length(im1));
back_depth1=zeros(480,640,length(im1));
back_rgb2=zeros(480,640,length(im1));
back_depth2=zeros(480,640,length(im1));
for i=1:length(im1)
    rgb_1=imread(im1(i).rgb);
    rgb_2=imread(im2(i).rgb);
    back_rgb1(:,:,i)=double(rgb2gray(rgb_1));
    back_rgb2(:,:,i)=double(rgb2gray(rgb_2));
    load(im1(i).depth);
    back_depth1(:,:,i)=double(depth_array);
    load(im2(i).depth);
    back_depth2(:,:,i)=double(depth_array);
end
back_rgb1=median(back_rgb1,3);
back_rgb2=median(back_rgb2,3);
back_depth1=median(back_depth1,3);
back_depth2=median(back_depth2,3);

if(parte==1)
    %% Rotações e translações conhecidas
elseif(parte==2)
    %% Cálculo através do ransac
    image1=imread(im1(1).rgb);
    image2=imread(im2(1).rgb);
    load(im1(1).depth);
    depth1=depth_array;
    load(im2(1).depth);
    depth2=depth_array;
    [ rotacao, translacao ] = ransac( image1, image2, depth1, depth2, cam_params );
    
end

%Coordenadas xyz do background
xyz_back1=get_xyzasus(back_depth1(:),[480 640],find(back_depth1>0),cam_params.Kdepth,1,0);
xyz_back2=get_xyzasus(back_depth2(:),[480 640],find(back_depth2>0),cam_params.Kdepth,1,0);

xyz_back2r=(rotacao*(xyz_back2)'+repmat(translacao,1,length(xyz_back1)))';

%Cálculo do RGB
%rgbd_back1 = get_rgbd(xyz_back1, back_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
%rgbd_back2 = get_rgbd(xyz_back2r, back_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);


%==========================================================================
%Ciclo principal do projecto
%==========================================================================
objectstracked=0;
for i=1:length(d2)
    load(strcat('depth',num2str(1),'_',num2str(i),'.mat'));
    im_depth1=depth_array;
    load(strcat('depth',num2str(2),'_',num2str(i),'.mat'));
    im_depth2=depth_array;
    im_rgb1=imread(strcat('rgb_image',num2str(1),'_',num2str(i),'.png'));
    im_rgb2=imread(strcat('rgb_image',num2str(2),'_',num2str(i),'.png'));
    no_back1=im_depth1;
    no_back2=im_depth2;
    %figure(1);
    %imagesc([im_rgb1, im_rgb2]);
    %figure(2);
    %imagesc([im_depth1, im_depth2]);
    
    xyz_1=get_xyzasus(im_depth1(:), [480 640], find(im_depth1(:)>0), cam_params.Kdepth, 1, 0);
    xyz_2=get_xyzasus(im_depth2(:), [480 640], find(im_depth2(:)>0), cam_params.Kdepth, 1, 0);
    %rotação para coordenadas da câmara 2
    xyz_2r=(rotacao*(xyz_2)'+repmat(translacao,1,length(xyz_1)))';
    
    rgbd1=get_rgbd(xyz_1, im_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd2=get_rgbd(xyz_2, im_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
    cl1=reshape(rgbd1,480*640,3);
    cl2=reshape(rgbd2,480*640,3);
    %ptotal=pointCloud([xyz_1;xyz_2r],'Color',[cl1;cl2]);
    %showPointCloud(ptotal);
    
    %% Remoção do background
    no_back1=abs(double(back_depth1)-double(im_depth1))>250;
    no_back2=abs(double(back_depth2)-double(im_depth2))>250;
    %figure(3);
    %imagesc([no_back1,no_back2]);
    
    %% Filtros do gradiente
    %deteta pontos da imagem de profundidade onde a variação de um pixel
    %para o outro seja maior que 25cm
    [fx_1, fy_1]=gradient(mat2gray(im_depth1));
    g_1=((fx_1.^2+fy_1.^2)>(250^2));
    no_back1=no_back1+g_1;
    [fx_2, fy_2]=gradient(mat2gray(im_depth2));
    g_2=((fx_2.^2+fy_2.^2)>(250^2));
    no_back2=no_back2+g_2;
    
    %% Filtros morfológicos
    no_back1=imopen(no_back1,strel('disk',8));
    no_back2=imopen(no_back2,strel('disk',8));
    %no_back1=imclose(no_back1,strel('disk',8));
    %no_back2=imclose(no_back2,strel('disk',8));
 
    
    %% Deteção de componentes conexas
    [l1, n1]=bwlabel(no_back1);
    [l2, n2]=bwlabel(no_back2);
    
    for j=1:n1
       [row, col]=find(l1==j);
       if(length(row)<500)
          for k=1:length(row)
              l1(row(k),col(k))=0;
          end
       end
    end
    for j=1:n2
        [row,col]=find(l2==j);
        if(length(row)<500)
            for k=1:length(row)
               l2(row(k),col(k))=0; 
            end
        end
    end
    [l1, n1]=bwlabel(l1);
    [l2, n2]=bwlabel(l2);
    %n1=numero de objectos detetados na imagem atual da câmara 1
    %n2=numero de objectos detetados na imagem atual da câmara 2
    dados1=regionprops(l1, 'centroid', 'Area');
    dados2=regionprops(l2, 'centroid', 'Area');
    
    %calculo das caixas ao redor dos objectos detetados na imagem 1
    for j=1:n1
       mascara=(abs(l1==j));
       mascara_depth=times(double(mascara),double(im_depth1));
       xyz_ob=get_xyzasus(mascara_depth(:), [480 640], find(mascara_depth(:)>0), cam_params.Kdepth, 1, 0);
       rgdb=get_rgbd(xyz_ob, im_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
       cl_ob=reshape(rgbd1,480*640,3);
       point_ob=pointCloud(xyz_ob,'Color',cl_ob);
       %figure(2);
       %showPointCloud(point_ob);
       %hold on;
       x=find(point_ob.Location(:,1)~=0);
       xmin1(1,j)=min(point_ob.Location(x,1));
       xmax1(1,j)=max(point_ob.Location(x,1));
       x=find(point_ob.Location(:,2)~=0);
       ymin1(1,j)=min(point_ob.Location(x,2));
       ymax1(1,j)=max(point_ob.Location(x,2));
       x=find(point_ob.Location(:,3)~=0);
       zmin1(1,j)=min(point_ob.Location(x,3));
       zmax1(1,j)=max(point_ob.Location(x,3));
       %scatter3(xmin1(1,j),ymin1(1,j),zmin1(1,j),'*');
       %scatter3(xmin1(1,j),ymin1(1,j),zmax1(1,j),'*');
       %scatter3(xmin1(1,j),ymax1(1,j),zmin1(1,j),'*');
       %scatter3(xmin1(1,j),ymax1(1,j),zmax1(1,j),'*');
       %scatter3(xmax1(1,j),ymin1(1,j),zmin1(1,j),'*');
       %scatter3(xmax1(1,j),ymin1(1,j),zmax1(1,j),'*');
       %scatter3(xmax1(1,j),ymax1(1,j),zmin1(1,j),'*');
       %scatter3(xmax1(1,j),ymax1(1,j),zmax1(1,j),'*');
    end
    
    close all;
    
    %calculo das caixas ao redor dos objectos detetados na imagem 2
    for j=1:n2
        mascara=(abs(l2==j));
       mascara_depth=times(double(mascara),double(im_depth2));
       xyz_ob=get_xyzasus(mascara_depth(:), [480 640], find(mascara_depth(:)>0), cam_params.Kdepth, 1, 0);
       rgdb=get_rgbd(xyz_ob, im_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
       cl_ob=reshape(rgbd1,480*640,3);
       point_ob=pointCloud(xyz_ob,'Color',cl_ob);
       %figure(2);
       %showPointCloud(point_ob);
       %hold on;
       x=find(point_ob.Location(:,1)~=0);
       xmin2(1,j)=min(point_ob.Location(x,1));
       xmax2(1,j)=max(point_ob.Location(x,1));
       x=find(point_ob.Location(:,2)~=0);
       ymin2(1,j)=min(point_ob.Location(x,2));
       ymax2(1,j)=max(point_ob.Location(x,2));
       x=find(point_ob.Location(:,3)~=0);
       zmin2(1,j)=min(point_ob.Location(x,3));
       zmax2(1,j)=max(point_ob.Location(x,3));
       %scatter3(xmin2(1,j),ymin2(1,j),zmin2(1,j),'*');
       %scatter3(xmin2(1,j),ymin2(1,j),zmax2(1,j),'*');
       %scatter3(xmin2(1,j),ymax2(1,j),zmin2(1,j),'*');
       %scatter3(xmin2(1,j),ymax2(1,j),zmax2(1,j),'*');
       %scatter3(xmax2(1,j),ymin2(1,j),zmin2(1,j),'*');
       %scatter3(xmax2(1,j),ymin2(1,j),zmax2(1,j),'*');
       %scatter3(xmax2(1,j),ymax2(1,j),zmin2(1,j),'*');
       %scatter3(xmax2(1,j),ymax2(1,j),zmax2(1,j),'*');
    end
    
    %% Tracking de objectos detetados na câmara 1
    if(i==1)
        %primeira imagem, não há com que comparar os objectos no caso de
        %serem detetados
        for j=1:n1
            objectstracked=objectstracked+1;
            objects(j).x=[xmin1(1,j), xmin1(1,j), xmin1(1,j), xmin1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j)];
            objects(j).y=[ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j), ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j)];
            objects(j).z=[zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j)];
            objects(j).frames_tracked=i;
            objects(j).centro=dados1(j).Centroid;
            last_objects(objectstracked,1)=j;
        end
    elseif(i>1)
        %já não é a primeira imagem, pode ser necessário efetuar o
        %algoritmo de hungaro, no caso de termos de comparar objectos, com
        %os detetados no frame anterior
        if(objectstracked==0)
            for j=1:n1
                %ainda não tinha detetado qualquer objecto, deve inserir logo
                objectstracked=objectstracked+1;
                objects(j).x=[xmin1(1,j), xmin1(1,j), xmin1(1,j), xmin1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j)];
                objects(j).y=[ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j), ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j)];
                objects(j).z=[zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j)];
                objects(j).frames_tracked=i;
                objects(j).centro=dados(j).Centroid;
                last_objects(objectstracked,1)=j;
            end
        end
        if(objectstracked>0 && last_n1==0)
            for j=1:n1
                %já há objectos na estrutura, mas na imagem anterior, não
                %detetou objectos, deve inserir na primeira posição livre
                objectstracked=objectstracked+1;
                objects(objectstracked).x=[xmin1(1,j), xmin1(1,j), xmin1(1,j), xmin1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j), xmax1(1,j)];
                objects(objectstracked).y=[ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j), ymin1(1,j), ymin1(1,j), ymax1(1,j), ymax1(1,j)];
                objects(objectstracked).z=[zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j), zmin1(1,j), zmax1(1,j)];
                objects(objectstracked).frames_tracked=i;
                objects(objectstracked).centro=dados1(j).Centroid;
                last_objects(objectstracked,1)=j;
            end
        end
        if (objectstracked>0 && last_n1>0 && n1>0)
            %já há objectos na estrutura, e há objectos detetados an imagem
            %anterior, proceder ao hungaro
            distancia=zeros(last_n1,n1);
            area=zeros(last_n1,n1);  
            for aux1=1:last_n1
                for aux2=1:n1
                    distancia(aux1,aux2)=norm((last_dados1(aux1).Centroid)-(dados1(aux2).Centroid));
                    area(aux1,aux2)=abs(last_dados1(aux1).Area/dados1(aux2).Area);
                end
            end
            
            custos=zeros(last_n1,n1);
            for aux1=1:last_n1
                    for aux2=1:n1
                        custos(aux1,aux2)=peso_area*area(aux1,aux2)+peso_distancia*distancia(aux1,aux2);
                        if (custos(aux1,aux2)>30)
                            custos(aux1,aux2)=custos(aux1,aux2)+250;
                        end
                    end
            end
            %matriz de custos para o hungaro concluida
            [linhas, colunas]=size(custos);
            %linhas=last_n1;
            %colunas=n1;
            
            %corre o hungaro para a nossa matriz de custos
            [assignment,cost]=munkres(custos);
            
            if(colunas>linhas)
                %existem mais objectos nesta imagem que na imagem anterior
                novos=setdiff(linspace(1,colunas,colunas),assignment);
                %novos=objectos que se encontraram novos, procedemos assim
                %á dua inserção
                for aux1=1:length(novos)
                	objectstracked=objectstracked+1;
                    objects(objectstracked).x=[xmin1(1,novos(aux1)), xmin1(1,novos(aux1)), xmin1(1,novos(aux1)), xmin1(1,novos(aux1)), xmax1(1,novos(aux1)), xmax1(1,novos(aux1)), xmax1(1,novos(aux1)), xmax1(1,novos(aux1))];
                    objects(objectstracked).y=[ymin1(1,novos(aux1)), ymin1(1,novos(aux1)), ymax1(1,novos(aux1)), ymax1(1,novos(aux1)), ymin1(1,novos(aux1)), ymin1(1,novos(aux1)), ymax1(1,novos(aux1)), ymax1(1,novos(aux1))];
                    objects(objectstracked).z=[zmin1(1,novos(aux1)), zmax1(1,novos(aux1)), zmin1(1,novos(aux1)), zmax1(1,novos(aux1)), zmin1(1,novos(aux1)), zmax1(1,novos(aux1)), zmin1(1,novos(aux1)), zmax1(1,novos(aux1))];
                    objects(objectstracked).frames_tracked=i;
                    objects(objectstracked).centro=dados1(novos(aux1)).Centroid;
                    last_objects(objectstracked,1)=novos(aux1); 
                end
                %falta atualizar os que já se encontravam na estrutura a
                %ser seguidos
                for aux1=1:length(assignment)
                    %verificação do cumprimento de um treshold, que garante
                    %que não existem objectos a mudar drasticamente
                    if (custos(aux1,assignment(aux1))>30)
                        %Custo demasiado grande, não devo atualizar mas
                        %inserir um novo objecto
                        objectstracked=objectstracked+1;
                        objects(objectstracked).x=[xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1))];
                        objects(objectstracked).y=[ymin1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymax1(1,assignment(aux1))];
                        objects(objectstracked).z=[zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1))];
                        objects(objectstracked).frames_tracked=i;
                        objects(objectstracked).centro=dados1(assignment(aux1)).Centroid;
                        last_objects(objectstracked,1)=assignment(aux1); 
                        
                    else
                        
                        for aux2=1:length(objects)
                            if(objects(aux2).centro(1)==last_dados1(aux1).Centroid(1) && objects(aux2).centro(2)==last_dados1(aux1).Centroid(2))
                                objects(aux2).x=[objects(aux2).x ;xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmin1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1)), xmax1(1,assignment(aux1))];
                                objects(aux2).y=[objects(aux2).y ;ymin1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymin1(1,assignment(aux1)), ymax1(1,assignment(aux1)), ymax1(1,assignment(aux1))];
                                objects(aux2).z=[objects(aux2).z ;zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1)), zmin1(1,assignment(aux1)), zmax1(1,assignment(aux1))];
                                objects(aux2).frames_tracked=[objects(aux2).frames_tracked; i];
                                objects(aux2).centro=dados1(assignment(aux1)).Centroid;
                            end
                        end
                        
                    end
                    
                end
                
            end
            if (linhas>colunas || linhas==colunas)
                %ou tenho o mesmo número, ou existem objectos que
                %desapareceram
                for aux2=1:length(assignment)
                    if (assignment(aux2)==0)
                        %o objecto saiu de cena
                    end
                    if (assignment(aux2)~=0)
                        if (custos(aux1,assignment(aux2))>30)
                            %custo demasiado alto, devo inserir um novo objecto
                            %e assumir que o anterior desapareceu
                            objectstracked=objectstracked+1;
                            objects(objectstracked).x=[xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2))];
                            objects(objectstracked).y=[ymin1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymax1(1,assignment(aux2))];
                            objects(objectstracked).z=[zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2))];
                            objects(objectstracked).frames_tracked=i;
                            objects(objectstracked).centro=dados1(assignment(aux2)).Centroid;
                            last_objects(objectstracked,1)=assignment(aux2);

                        else
                            for aux3=1:length(objects)
                                if(objects(aux3).centro(1)==last_dados1(aux2).Centroid(1) && objects(aux3).centro(2)==last_dados1(aux2).Centroid(2))
                                    objects(aux3).x=[objects(aux3).x ;xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmin1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2)), xmax1(1,assignment(aux2))];
                                    objects(aux3).y=[objects(aux3).y ;ymin1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymin1(1,assignment(aux2)), ymax1(1,assignment(aux2)), ymax1(1,assignment(aux2))];
                                    objects(aux3).z=[objects(aux3).z ;zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2)), zmin1(1,assignment(aux2)), zmax1(1,assignment(aux2))];
                                    objects(aux3).frames_tracked=[objects(aux3).frames_tracked; i];
                                    objects(aux3).centro=dados1(assignment(aux2)).Centroid;
                                end
                            end    
                        end
                    end
                    
                end
                
            end
            
        end
        
    end
    
    last_im_depth1=im_depth1;
    last_im_depth2=im_depth2;
    last_im_rgb1=im_rgb1;
    last_im_rgb2=im_rgb2;
    last_l1=l1;
    last_n1=n1;
    last_l2=l2;
    last_n2=n2;
    last_dados1=dados1;
    last_dados2=dados2;
    if(i==20)
       debug=1;
    end
    
    
end

