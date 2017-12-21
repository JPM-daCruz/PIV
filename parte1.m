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
objectos=0;
last_objects=[];
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
%figure(1)
%imagesc([back1, back2]);
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
        %figure(3);
        %imagesc([im_depth1, im_depth2]);
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
        %figure(4);
        %imagesc([im_depth1, im_depth2]);
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
    
        %figure(7);
        %imagesc([l1, l2]);
        %figure(8);
        %imagesc([im_rgb1, im_rgb2]);
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
        %figure(9);
        %imagesc([im_depth1, G1]);
    
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
        stats = regionprops(l1,'centroid', 'Area');
        
        %fim da an�lise da imagem1
        for aux1=1:num1
            objects(aux1).x=[xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmin_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1), xmax_1(1,aux1)];
            objects(aux1).y=[ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1), ymin_1(1,aux1), ymin_1(1,aux1), ymax_1(1,aux1), ymax_1(1,aux1)];
            objects(aux1).z=[zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1), zmin_1(1,aux1), zmax_1(1,aux1)];
            objects(aux1).frames_tracked=1;
            objectos=objectos+1;
            last_objects(objectos,1)=aux1;
        end
        debug=1;
        
        
    else
%==========================================================================
%==========================================================================
        %a partir da segunda imagem j� � necess�io comparar os objectos que
        %s�o detetados com aqueles que foram detetados anteriormente para
        %efetuar o tracking do objecto ao inv�s de inserir um objecto novo
        %cada vez que um objeto � detetado
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
        %figure(7);
        %imagesc([l1, l2]);
        %figure(8);
        %imagesc([im_rgb1, im_rgb2]);
        %Filtro para garantir que n�o h� pontos incorretamente medidos na
        %extremidade dos objectos que levem, a uma incorreta obten��o da caixa
        l1=imclose(l1, strel('disk',10));
        l2=imclose(l2, strel('disk',10));
        %% C�lculo da caixa em que o onjecto se encontra contido
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
        stats = regionprops(l1,'centroid', 'Area');
               
        %% 
        if(num1>0)
            %significa que temos objetos a serem detetados na imagem atual,
            %logo temos de verificar a correspond�ncia entre objectos.
            
            %necessario correr a fun��o hungaro para verificar as
            %correspond�ncias entre objectos, para isso, s�o avaliados
            %diversos par�metros, como a area do objecto, a dist�ncia entre
            %o centr�ide de cada objecto de imagem para imagem
            %Por enquanto, esta fun��o relaciona a area dos dois objectos
            distancia_entre_centroides = zeros(length(last_stats),length(stats));
            area_dos_objectos = zeros(length(last_stats),length(stats));
             %compara todos os objectos da imagem atual com todos os objectos
            %que existiam na imagem anterior, para elaborar a matriz de
            %rela�oes
            for aux1=1:last_num1
                for aux2=1:num1
                %C�lculo da rela��o entre a dist�ncia dos centroides dos
                %objectos
                distancia_entre_centroides(aux1, aux2)=norm((last_stats(aux1).Centroid)-(stats(aux2).Centroid));
                %C�lculo da rela��o entre as �reas dos objectos detetados
                area_dos_objectos(aux1,aux2)=abs((last_stats(aux1).Area)/(stats(aux2).Area));
                end
            end
            matrix=zeros(last_num1,num1);
            %pesos das diversas rela��es entre os objectos
            peso_distancia=0.6;
            peso_area=0.4;
            
            % Tabela com o custo total de associa��o de cada par de
            % regi�es, tendo em conta todos os par�metros definidos. A
            % soma do valor 500 ocorre para colmatar um erro
            % identificado no algoritmo hungaro (fun��o munkres) que
            % opta pelo menor custo, mesmo que para isso ignore o
            % melhor caso. Com esta soma, apenas o ru�do � 
            % intensificado, de maneira a colmatar o referido erro.
            for aux1=1:last_num1
                for k=1:num1
                    matrix(aux1,k) = peso_area*area_dos_objectos(aux1,k) + peso_distancia*distancia_entre_centroides(aux1,k);
                    if (matrix(aux1,k) > 35)
                    matrix(aux1,k) = matrix(aux1,k) + 500;
                    end
                end
            end
            
            [linhas, colunas]=size(matrix);
            [a,b] = munkres(matrix);
            
            %1� caso, numero de objectos iguais nas duas imagens
            if  (colunas==linhas)
                debug=1;
                for aux1=1:length(a)
                    indice=find(last_objects==aux1);
                    objects(indice).x=[objects(indice).x ;xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1))];
                    objects(indice).y=[objects(indice).y ;ymin_1(1,a(aux1)), ymin_1(1,a(aux1)), ymax_1(1,a(aux1)), ymax_1(1,a(aux1)), ymin_1(1,a(aux1)), ymin_1(1,a(aux1)), ymax_1(1,a(aux1)), ymax_1(1,a(aux1))];
                    objects(indice).z=[objects(indice).z ;zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1))];
                    objects(indice).frames_tracked=[objects(indice).frames_tracked;i]; 
                    last_objects(indice,1)=a(aux1);
                end   
            end
            %2� caso, maior numero de colunas->novos objectos
            if(colunas>linhas)
                %atualiza os objetos ja existente
                for aux1=1:length(a)
                        indice=find(last_objects==aux1);
                        objects(indice).x=[objects(indice).x ;xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmin_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1)), xmax_1(1,a(aux1))];
                        objects(indice).y=[objects(indice).y ;ymin_1(1,a(aux1)), ymin_1(1,a(aux1)), ymax_1(1,a(aux1)), ymax_1(1,a(aux1)), ymin_1(1,a(aux1)), ymin_1(1,a(aux1)), ymax_1(1,a(aux1)), ymax_1(1,a(aux1))];
                        objects(indice).z=[objects(indice).z ;zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1)), zmin_1(1,a(aux1)), zmax_1(1,a(aux1))];
                        objects(indice).frames_tracked=[objects(indice).frames_tracked;i]; 
                        last_objects(indice,1)=a(aux1);
                end
                %insere os nosso objetos
                   novos = setdiff(linspace(1,colunas,colunas),a);
                   for aux1 = 1:length(novos)
                        %novo objecto que ainda nao estava na lista
                        objectos=objectos+1;
                        objects(objectos).x=[xmin_1(1,novos(aux1)), xmin_1(1,novos(aux1)), xmin_1(1,novos(aux1)), xmin_1(1,novos(aux1)), xmax_1(1,novos(aux1)), xmax_1(1,novos(aux1)), xmax_1(1,novos(aux1)), xmax_1(1,novos(aux1))];
                        objects(objectos).y=[ymin_1(1,novos(aux1)), ymin_1(1,novos(aux1)), ymax_1(1,novos(aux1)), ymax_1(1,novos(aux1)), ymin_1(1,novos(aux1)), ymin_1(1,novos(aux1)), ymax_1(1,novos(aux1)), ymax_1(1,novos(aux1))];
                        objects(objectos).z=[zmin_1(1,novos(aux1)), zmax_1(1,novos(aux1)), zmin_1(1,novos(aux1)), zmax_1(1,novos(aux1)), zmin_1(1,novos(aux1)), zmax_1(1,novos(aux1)), zmin_1(1,novos(aux1)), zmax_1(1,novos(aux1))];
                        objects(objectos).frames_tracked=i;
                        last_objects(objectos)=novos(aux1);
                        
                   end
            end
            %3� caso, menor numero de colunas->objectos desaparecam
   
            
            
        end
    
    end
    %% Armazena os dados correspondentes �s duas �ltimas imagens
    last_im_rgb1=im_rgb1;
    last_im_rgb2=im_rgb2;
    last_im_depth1=im_depth1;
    last_im_depth2=im_depth2;
    last_l1=l1;
    last_num1=num1;
    last_l2=l2;
    last_num2=num2;
    last_stats=stats;
    
  
end

%% Fun��es externas
%foram retiradas da internet