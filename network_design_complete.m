
%  clc
clear all
close all
indx_height_centre= 748; % the number of images in centre pointing in each height level
index_height_facade=785;% the number of images in facade pointing in each height level
indx_height_hybrid=3120;
map= imread('refectory_map5.jpg');
map_without_obstacle=imread('output_without_obstacles5.jpg');
img_obstacle=imread('output_obstacles_5.jpg');
level = graythresh(map);
binary_map= im2bw(map,level);
figure, imshow(binary_map)
grid_point=zeros(size(map,1),size(map,2));
for i= 1:20:size(map,2)                  % pixel size in map(x)= 53 mm ---- 76.079/666
    for j=1:20:size(map,1)
        if binary_map(j,i)==1;           % pixel size in map(y)= 53 mm ----  81.287/747
            grid_point(j,i)=255;
        end
    end
end
 figure,imshow(grid_point),title('grid_point')
%% plot grid_point on map
 [r,c]=find(grid_point==255);
 grid_point_show=[r,c];
 figure, imshow(map),title('map');
 hold on
 plot(grid_point_show(:,2),grid_point_show(:,1), 'g*');

%% Optimum Distance Estimation

% f=input('please input the focal length : ');
% f_stop=input('please input the f-stop : ');
% Dtos=input('please input the target object size : ');
% Dtis=input('please input the target image size : ')
% Ires=input('please input the pixel size : ');
% phi=input('please input the angle between ray coming from the camera and target plane : ');
% alpha=input('please input half the FOV angle of camera : ');
% d0=input('please input the minimum frame size of image : ');
% D0=input('please input the maximum length of the object : ');
% k=input('please input the number of images at each points : ');
% q=input('please input the design factor : ');
% Sp=input('please input the specified precision number : ');
% sigma=input('please input the image measurment error : ');
% n= input('please input the number of image points : ');
% a= input('please input the ............... : ');

%% range-related constraint

% D_scale
f=18; %mm
D=10000 %35000 ;          % maximum length to the object
k=1 ;                % number of images at each points
q=0.6 ;             % design factor (0.4 - 0.7)
Sp= 500;  %0.003       % specified precision number(25000-200000)
sigma=0.5 *0.0039;        % image measurment error .... 1.2-1.3 pixel size

Dmax_scale = (D* f * sqrt(k))/(q * Sp *sigma);

% D_spatial resolution
Dtos=100 ;          % target object size - mm
Dtis=2;          % target image size ,minimum number of target's pixel =(5-10)pixel
Ires=0.0039 ;  % pixel size ,
phi=pi/2  ;          % angle between ray coming from the camera and target plane

Dmax_reso =(f* Dtos *sin(phi))/(Ires * Dtis);

% D_FOV
d0=15.6 ;          % minimum frame size of image
D0=12000 %15000;
alpha= atan((0.9*d0)/(2*f));    % FOV angle of camera (F0V/2 )

D_fov = (D0* sin(alpha+phi))/(2*sin(alpha));

% D_MAX
Dmax= min (Dmax_reso , Dmax_scale );
D_max= min(Dmax , D_fov);
%D_max=Dmax;

% D_DOF
f_stop=8;           % aperture stop...f/number(D)...D=1.4 , 2 , 2.8 , 4 , 5.6 , 8 , 11 , 16
bc=f/1720;% 0.022;          % blur circle=~0.2 or = pixel size
D_HF= (f^2)/(f_stop * bc);      %hyperfocal distance
D_opt=D_max;                      %camera distance focus , or >>> D_OP=D_HF+f
Dmin_DOF=(D_opt*D_HF)/(D_HF+(D_opt-f));
Dmax_DOF=(D_opt*D_HF)/(D_HF-(D_opt-f));

D_DOF=(Dmax_DOF - Dmin_DOF);       % Depth Of Field

% D_FOV_min     %% workspace_constaint?
d0=15.6 ;          % minimum frame size of image
D00=8000;
alpha= atan((0.9*d0)/(2*f));    % FOV angle of camera (F0V/2 )

D_fov_min = (D00* sin(alpha+phi))/(2*sin(alpha));

% D_minpoint
n=4 ;             % number of image points
unit_a=3000;              % distance between points in object space

Dmin_point = (unit_a*f*sqrt(n))/(d0);

%D_min
D_min= max (Dmin_DOF , D_fov_min);%Dmin_point );
%D_min= Dmin_DOF;

Range = D_max - D_min ;

D_max_p = D_max/50 ;             % pixel size in map=49.2896 %60.7 for refectory building
D_min_p = D_min/50 ;

DM = round(D_max_p);    % round D_MAX in pixel
Dm = round(D_min_p);    % round D_min in pixel
%% candidate points in Optimum Distance
im_morph_obstacle=255-img_obstacle;  % delete black border
[m , n , p]=size(im_morph_obstacle);
im_morph_obstacle(:,1:40,:)=0;
im_morph_obstacle(:,n-40:n,:)=0;
im_morph_obstacle(1:40,:,:)=0;
im_morph_obstacle(m-40:m,:,:)=0;

se1_obstacle=ones(2*40);      %Dm
dilation1_obstacle =255-imdilate(im_morph_obstacle,se1_obstacle);



im_morph=255-map_without_obstacle;  % delete black border
[m , n , p]=size(im_morph);
im_morph(:,1:40,:)=0;
im_morph(:,n-40:n,:)=0;
im_morph(1:40,:,:)=0;
im_morph(m-40:m,:,:)=0;

se1=ones(2*Dm);      %Dm
se2=ones(2*DM);      %DM
dilation1 = imdilate(im_morph,se1);
dilation2 = imdilate(im_morph,se2);
target_range1=dilation2-dilation1;
target_range=target_range1.*dilation1_obstacle;
figure,imshow(target_range),title('target-range')
candidate_points=(target_range(:,:,1)) & (grid_point) ;
figure,imshow(candidate_points),title('candidate_points')

%% plot candid_points with label
[r,c]=find(candidate_points==1);
candid_points=[r,c];
 labels = cellstr( num2str([1:size(candid_points,1)]'));
 figure, imshow(map),title('map');
 hold on
 %plot(candid_points(:,2),candid_points(:,1), 'g*');text(candid_points(:,2),candid_points(:,1),labels);
plot(candid_points(:,2),candid_points(:,1), 'g*');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% candidate ponits in facad pointing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% compute edge point, minimum distance and select the nearest edge point
% edge detection
im_new= rgb2gray(im_morph);
edge1 = edge(im_new,'canny');
 figure, imshow(edge1);
[re,ce]=find(edge1==1);
edge_points=[re,ce];
edge=1-edge1 ;
% compute distance
nearest_edge=[];
nearest_candid=[];
distance=[];
for i=1:size(candid_points,1)
    a1=candid_points(i,:);
    distance=[];
    for j=1:size(edge_points)
        a2=edge_points(j,:);
        dist=sqrt(sum((a1-a2).^2));
        distance=[distance;dist];
    end
    minmum= min(distance);
    ID_min=find(distance==minmum);
    minID_edge=edge_points(ID_min,:);
    minID_candid=candid_points(i,:);
    ss=size(minID_edge,1);
    
    if ss(1,1)~=1
        for w=1:ss(1,1)
            ID_min_double = ID_min(w) ;
            minID_edge_dou=edge_points(ID_min_double,:);
            nearest_candid=[nearest_candid;minID_candid];
            nearest_edge=[nearest_edge;minID_edge_dou];
        end
    else
        nearest_candid=[nearest_candid;minID_candid];
        nearest_edge=[nearest_edge;minID_edge];
    end
    
end
candidate_edge_image=[nearest_candid,nearest_edge];
%% plot all station on map
 labels = cellstr( num2str([1:size(candidate_edge_image,1)]'));
 figure, imshow(map),title('obstacle');%_without_obstacle
 hold on
 plot(candidate_edge_image(:,2),candidate_edge_image(:,1), 'g*');%text(candidate_edge_image(:,2),candidate_edge_image(:,1),labels);
% %draw axis of sight
 for i=1:size(candidate_edge_image);
 r_P=[candidate_edge_image(i,1),candidate_edge_image(i,3)]';
 c_P=[candidate_edge_image(i,2),candidate_edge_image(i,4)]';
  line(c_P,r_P,'color','black','LineWidth',1);
 end
%%  camputing intersection between camera's ray and obstacle for primary candidate points

levell = graythresh(img_obstacle);
im_ob= im2bw(img_obstacle,levell);
index=1;
index2=1;
for i=1:size(candidate_edge_image,1)
    
    Y_coordinate=[candidate_edge_image(i,1), candidate_edge_image(i,3)];   %image coordinate(candidate point and nearest edge point)
    X_coordinate=[candidate_edge_image(i,2), candidate_edge_image(i,4)];
    D_X= abs(candidate_edge_image(i,2)- candidate_edge_image(i,4));
    D_Y= abs(candidate_edge_image(i,3)-candidate_edge_image(i,1));
    Xc_max=max(candidate_edge_image(i,2),candidate_edge_image(i,4));
    Xc_min=min(candidate_edge_image(i,2),candidate_edge_image(i,4));
    Yc_max=max(candidate_edge_image(i,1),candidate_edge_image(i,3));
    Yc_min=min(candidate_edge_image(i,1),candidate_edge_image(i,3));
    if D_X ~=0 
        X= Xc_min :Xc_max ;
        Y= interp1(X_coordinate,Y_coordinate,X);
        Y= round(Y);                                   % y_coordinate of line point    
    else
        X=repmat(X_coordinate(1,1),1,(D_Y)+1);
         Y=Yc_min :Yc_max ;
        
     end
    pixel_value =diag(im_ob(Y,X)); % Y is row of image and X is column
    if length(find(pixel_value==0))==0
        visible_station_early(index,:)=candidate_edge_image(i,:);   % candidate points without intersection with obstacle
        index=index+1;
    else
        unvisible_station(index2,:)=candidate_edge_image(i,:);
        index2=index2+1;
    end
end
%% PLOT unvisible_station
 labels = cellstr( num2str([1:size(unvisible_station,1)]'));
 figure, imshow(map),title('visible_station_early');%_without_obstacle
 hold on
 plot(unvisible_station(:,4),unvisible_station(:,3), 'r*');%text(unvisible_station(:,4),unvisible_station(:,3),labels);
% %% plot all station on map
 labels = cellstr( num2str([1:size(visible_station_early,1)]'));
 figure, imshow(img_obstacle),title('visible_station_early');%_without_obstacle
 hold on
 plot(visible_station_early(:,2),visible_station_early(:,1), 'g*');text(visible_station_early(:,2),visible_station_early(:,1),labels);
% % draw axis of sight for visible points
 for i=1:size(visible_station_early);
     r_P=[visible_station_early(i,1),visible_station_early(i,3)]';
     c_P=[visible_station_early(i,2),visible_station_early(i,4)]';
     line(c_P,r_P,'color','black','LineWidth',1);
 end

%%  camputing intersection between obstacle and camera's ray from visible_station_early to UnvisibleEdge_corners
corner= corner(im_new);                        % corner detection
 figure,imshow(im_new),title('im_without_abstacle');
 hold on
 plot(corner(:,1),corner(:,2),'r*');
corners=[corner(:,2) corner(:,1)];
UnvisibleEdge_corners=[corners;unvisible_station(:,3:4)];   % merge the unvisible edge and corners
visible_station_candid=visible_station_early(:,1:2);
index3=1;
index4=1;
for i=1:size(UnvisibleEdge_corners,1)
    visible_station=select_visible_station(im_ob,UnvisibleEdge_corners(i,:),visible_station_candid);
    if (size(visible_station,1)==0)
        disp('error!');
        disp(i);
    end
    % compute distance
    
    b1=UnvisibleEdge_corners(i,:);
    distances=[];
    for n=1:size(visible_station,1)
        b2=visible_station(n,:);
        
        distt=sqrt(sum((b1-b2).^2));
        distances=[distances;distt];
    end
    minimum_dist= sort(distances);
    if (size(minimum_dist,1)~=0)
        for k=1:6                            % k is number of nearest cameras
            [ID_minimum,~]=find(minimum_dist(k,1)==distances(:,1));
            minimum_candid(k,:)=visible_station(ID_minimum(1,1),:);
            clear ID_minimum
        end
        near_candid(6*i-5:6*i,:)=minimum_candid;
        clear minimum_dist distances %visible_station
        visibleC_unvisibleEdge_corners(6*i-5:6*i,1:2)=minimum_candid;     % visibleP_unvisibleEdge_corners is include corners , unvisible edge and corresponding visible candidate point
        visibleC_unvisibleEdge_corners(6*i-5:6*i,3:4)=repmat(UnvisibleEdge_corners(i,:),6,1);
    end
end
all_station=[visible_station_early ; visibleC_unvisibleEdge_corners];   % include final candid point
n=1;
while (n<size(all_station,1))
    if (all_station(n,1)^2+all_station(n,2)^2+all_station(n,3)^3+all_station(n,4)^2==0)
        all_station(n,:)=[];
        n=n-1;
    end
    n=n+1;
end

%% plot all station on map
labels = cellstr( num2str([1:size(visible_station_early,1)]'));
figure, imshow(map),title('visible_station_early');%_without_obstacle
hold on
plot(visible_station_early(:,2),visible_station_early(:,1), 'g*');text(visible_station_early(:,2),visible_station_early(:,1),labels);

%draw axis of sight for visible points
 %for i=1:size(visible_station,1);
 %    r_P=[UnvisibleEdge_corners(23,1),visible_station(i,1)]';
  %   c_P=[UnvisibleEdge_corners(23,2),visible_station(i,2)]';
  %   line(c_P,r_P,'color','black','LineWidth',1);
 %end

% draw  visibleC_unvisibleEdge_corners
 %for i=1:size(visibleC_unvisibleEdge_corners);
 %    r_P=[visibleC_unvisibleEdge_corners(i,1),visibleC_unvisibleEdge_corners(i,3)]';
 %    c_P=[visibleC_unvisibleEdge_corners(i,2),visibleC_unvisibleEdge_corners(i,4)]';
 %    line(c_P,r_P,'color','black','LineWidth',1);
 %end

% draw axis of sight for all stations
for i=1:size(all_station);
    r_P=[all_station(i,1),all_station(i,3)]';
    c_P=[all_station(i,2),all_station(i,4)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end

% % draw axis of sight for visible points
 %for i=1:size(visible_station_early);
 %    r_P=[visible_station_early(i,1),visible_station_early(i,3)]';
 %    c_P=[visible_station_early(i,2),visible_station_early(i,4)]';
 %    line(c_P,r_P,'color','black','LineWidth',1);
 %end

%% transformation coordinate of candidate points and edge points from imege space to ground space

icp=textread('icp2.txt');
GCP=textread('GCP2.txt');
tform = maketform('affine',[icp(1:3,1) , icp(1:3,2)],[GCP(1:3,1), GCP(1:3,2)]);
[xtest, ytes]=tformfwd(tform, icp(4,1) , icp(4,2));
[xc, yc] = tformfwd(tform, all_station(:,1), all_station(:,2)) ;
[xe, ye] = tformfwd(tform, all_station(:,3), all_station(:,4)) ;

%% 3D Coordinate (condidate points and edge points)
coordinate=[];
coordinate(:,1)=xc;
coordinate(:,2)=yc;
threeD_coordinate=[];
%height=input('please input height of building : ');
for i=0.4:1:15
    threeD_coordinate=[threeD_coordinate;coordinate,i*ones(size(xc,1),1)];
end
coordinate_edge=[];
coordinate_edge(:,1)=xe;
coordinate_edge(:,2)=ye;
threeD_coordinate_edge=[];
for i=0.4:1:15
    threeD_coordinate_edge=[threeD_coordinate_edge;coordinate_edge,i*ones(size(xe,1),1)];
end
candidate_edge_3D =[threeD_coordinate,threeD_coordinate_edge]; % all point in 3d space

%% Quaternion - computing roll,pitch,yaw for points with height=¿

bx=(xe-xc);
by=(ye-yc);

bz=zeros(size(bx,1),1);
n=1;
norm_b=sqrt(bx.^2+by.^2);
norm_a=1;
unit_bx=bx./norm_b;         % normal
unit_by=by./norm_b;
unit_bz=bz./norm_b;
unit_b=[unit_bx,unit_by,unit_bz];

for n=1:size(bx,1)
    if bx(n)==0
        bx(n)=0.01;
    end
    a=[0 0 -1];                % unit vector(Z vector(camera coordinate system)in global coordinate system)
    arobot=[0,1,0];
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    Outx = a(1,2) * unit_b(n,3) - a(1,3) * unit_b(n,2);
    Outy = a(1,3) * unit_b(n,1) - a(1,1) * unit_b(n,3);
    Outz = a(1,1) * unit_b(n,2) - a(1,2) * unit_b(n,1);
    
    Outx_robot=arobot(1,2) * unit_b(n,3) - arobot(1,3) * unit_b(n,2);
    Outy_robot = arobot(1,3) * unit_b(n,1) - arobot(1,1) * unit_b(n,3);
    Outz_robot = arobot(1,1) * unit_b(n,2) - arobot(1,2) * unit_b(n,1);

    rotation_vector(n,:)=[Outx Outy Outz];
    rotation_angle(n,:)= acos( a(1,1) * unit_b(n,1) + a(1,2) * unit_b(n,2) + a(1,3) * unit_b(n,3));
    rotation_vector_robot(n,:)=[Outx_robot Outy_robot Outz_robot];
    rotation_angle_robot(n,:)= acos( arobot(1,1) * unit_b(n,1) + arobot(1,2) * unit_b(n,2) + arobot(1,3) * unit_b(n,3));
    if (round(Outx_robot) ==0 &&round(Outy_robot)==0 &&round(Outz_robot)==0)
        rotation_vector_robot(n,:)=[0,0,1];
    end
end
Rvector_ex=rotation_vector(:,1);
Rvector_ey=rotation_vector(:,2);
Rvector_ez=rotation_vector(:,3);

q0 = cos(rotation_angle/2);
q1 = Rvector_ex .* sin(rotation_angle/2);
q2 = Rvector_ey .* sin(rotation_angle/2);
q3 = Rvector_ez .* sin(rotation_angle/2);

Rvector_ex_robot=rotation_vector_robot(:,1);
Rvector_ey_robot=rotation_vector_robot(:,2);
Rvector_ez_robot=rotation_vector_robot(:,3);

q0_robot = cos(rotation_angle_robot/2);
q1_robot = Rvector_ex_robot .* sin(rotation_angle_robot/2);
q2_robot = Rvector_ey_robot .* sin(rotation_angle_robot/2);
q3_robot = Rvector_ez_robot .* sin(rotation_angle_robot/2);

jjjj= q0.^2 +q1.^2 +q2.^2 +q3.^2 ;
yaw=atan2((2.*(q1.*q2+q0.*q3)), (q1.^2+q0.^2-q3.^2-q2.^2));       %rotation about the Z-axis (radian)
pitch=asin(2.*(q0.*q2-q1.*q3));                                  %rotation about the Y-axis (radian)
roll=atan2((2.*(q0.*q1+q3.*q2)), (q3.^2-q2.^2-q1.^2+q0.^2));      %rotation about the X-axis (radian)

tz=size(threeD_coordinate,1)/size(coordinate,1) ;
euler = [roll.*180/pi , pitch.*180/pi , yaw.*180/pi];             % orientation in degree
total_euler=repmat(euler,tz,1);

label=(1:size(threeD_coordinate,1))';
%coordinate_rotation=[label , threeD_coordinate , total_euler];
coordinate_rotation=[label(1:size(coordinate,1),:) , (threeD_coordinate(1:size(coordinate,1),:))*1000, total_euler(1:size(coordinate,1),:), q0_robot(1:size(coordinate,1),:),q1_robot(1:size(coordinate,1),:),q2_robot(1:size(coordinate,1),:),q3_robot(1:size(coordinate,1),:)];
%coordinate_rotation=[label(size(all_station,1)+1:2*size(all_station,1),:) , (threeD_coordinate(1:size(all_station,1),:))*1000, total_euler(1:size(all_station,1),:)];
%% plot selected candidate point in IND for facad pointing
output_ind= textread('ind_output_facade pointing_5.txt');
all_candid_pnts=[label(1:2*size(all_station,1),:),(repmat(all_station(:,:),2,1))];
for i=1:size(output_ind,1)
    [m,~] =find(all_candid_pnts(:,1)==output_ind(i,1));
    IND(i,:)=all_candid_pnts(m,:);
end
figure, imshow(map),title('output_ind');
hold on
for i=1:size(IND);
    if (IND(i,1)<index_height_facade)
        h=plot(IND(i,3),IND(i,2),'r*');%text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
        set(h,'linewidth',8,'Color','red');
        hold on
    else
         h=plot(IND(i,3),IND(i,2),'go');%text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
         set(h,'linewidth',8,'Color','green');
         hold on
    end        
end
%plot(IND(:,3),IND(:,2),'r*');text(IND(:,3),IND(:,2),cellstr( num2str([IND(:,1)])));

% draw axis of sight
IND_r=IND;
for i=1:size(IND,1)
   [IND_r(i,2), IND_r(i,3)] = tformfwd(tform, IND(i,2), IND(i,3)) ;
   [IND_r(i,4), IND_r(i,5)] = tformfwd(tform, IND(i,4), IND(i,5)) ;
end
bx=(IND_r(:,4)-IND_r(:,2));
by=(IND_r(:,5)-IND_r(:,3));
bz=zeros(size(IND_r,1),1);
norm_b=sqrt(bx.^2+by.^2);
norm_a=1;
unit_bx=bx./norm_b;         % normal
unit_by=by./norm_b;
unit_bz=bz./norm_b;
unit_b=[unit_bx,unit_by,unit_bz];

for n=1:size(IND_r,1)
    if bx(n)==0
        bx(n)=0.01;
    end
    arobot=[0,1,0];
    
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    
    Outx_robot=arobot(1,2) * unit_b(n,3) - arobot(1,3) * unit_b(n,2);
    Outy_robot = arobot(1,3) * unit_b(n,1) - arobot(1,1) * unit_b(n,3);
    Outz_robot = arobot(1,1) * unit_b(n,2) - arobot(1,2) * unit_b(n,1);
    
	rotation_vector_robot(n,:)=[Outx_robot Outy_robot Outz_robot];
    rotation_angle_robot(n,:)= acos( arobot(1,1) * unit_b(n,1) + arobot(1,2) * unit_b(n,2) + arobot(1,3) * unit_b(n,3));
    if (round(Outx_robot) ==0 &&round(Outy_robot)==0 &&round(Outz_robot)==0)
        rotation_vector_robot(n,:)=[0,0,1];
    end
end
Rvector_ex_robot=rotation_vector_robot(:,1);
Rvector_ey_robot=rotation_vector_robot(:,2);
Rvector_ez_robot=rotation_vector_robot(:,3);

q0_robot = cos(rotation_angle_robot/2);
q1_robot = Rvector_ex_robot .* sin(rotation_angle_robot/2);
q2_robot = Rvector_ey_robot .* sin(rotation_angle_robot/2);
q3_robot = Rvector_ez_robot .* sin(rotation_angle_robot/2);
length_q_robot=sqrt(q0_robot(:,1).^2+q1_robot(:,1).^2+q2_robot(:,1).^2+q3_robot(:,1).^2);


IND_Robot=[IND_r(1:size(IND_r,1),:), q0_robot(1:size(IND_r,1),:),q1_robot(1:size(IND_r,1),:),q2_robot(1:size(IND_r,1),:),q3_robot(1:size(IND_r,1),:)];

for i=1:size(IND);
    r_P=[IND(i,2),IND(i,4)]';
    c_P=[IND(i,3),IND(i,5)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end
%% Path planning viewpoints order in facade pointing
figure;
B(size(IND,1),2)=0;
B2(size(IND,1),2)=0;
Sorted_viewpoints(size(IND,1),4)=0;
Sorted_viewpoints2(size(IND,1),4)=0;
for i=1:size(IND,1);
    if (IND(i,1)<index_height_facade)
        B(i,:) = [IND(i,2),IND(i,3)];
        h= plot(IND(i,3),IND(i,2),'r*');text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
        %set(h,'linewidth',8,'Color','red')
        hold on
    else
        B2(i,:) = [IND(i,2),IND(i,3)];
        h= plot(IND(i,3),IND(i,2),'O');text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
        %set(h,'linewidth',8,'Color','green')
         hold on
    end        
end
i=1;

visi_matrix=zeros(size(B,1));
visi_matrix2=zeros(size(B2,1));
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B, B_C).^2,2));
j=find(distances_bc==min(distances_bc));
while (i<size(IND,1))
    A2 = [IND(j,2),IND(j,3)];
    B(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B, A2).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
    %closest3 = B(j2(1,1),:);
    dx=IND(j2(1,1),2)-IND(j,2);
    dy=IND(j2(1,1),3)-IND(j,3);
    m= atan2(dy,dx);
    y=IND(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND(j,2)+t*cos(m));
        y=round(IND(j,3)+t*sin(m));
        dx=IND(j2(1,1),2)-x;
        dy=IND(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
            obstacle=1;
        end
    end
    if (obstacle==0) 
        r_P=[IND(j,2),IND(j2(1,1),2)]';
        c_P=[IND(j,3),IND(j2(1,1),3)]';        
        line(c_P,r_P,'color','green','LineWidth',1);
        hold on; 
        Sorted_viewpoints(i,:)=[A2,B(j2(1,1),:)];
        visi_matrix(j,j2(1,1))=1; 
        visi_matrix(j2(1,1),j)=1; 
        i=i+1;       
    end
    r_p2=[IND(j,2),IND(j,4)]';
    c_p2=[IND(j,3),IND(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1); 
end

i=1;
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B2, B_C).^2,2));
j_=find(distances_bc==min(distances_bc));
j=j_(1);
while (i<size(IND,1))
    A2 = [IND(j,2),IND(j,3)];
    B2(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B2, A2).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
   %closest3 = B(j2(1,1),:);
    dx=IND(j2(1,1),2)-IND(j,2);
    dy=IND(j2(1,1),3)-IND(j,3);
    m= atan2(dy,dx);
    y=IND(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND(j,2)+t*cos(m));
        y=round(IND(j,3)+t*sin(m));
        dx=IND(j2(1,1),2)-x;
        dy=IND(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
           obstacle=1;
        end
    end
    if (obstacle==0)        
        Sorted_viewpoints2(i,:)=[A2,B2(j2(1,1),:)];
        visi_matrix2(j,j2(1,1))=1; 
        visi_matrix2(j2(1,1),j)=1;
        i=i+1;
        r_P=[IND(j,2),IND(j2(1,1),2)]';
        c_P=[IND(j,3),IND(j2(1,1),3)]';
        line(c_P,r_P,'color','red','LineWidth',1);        
        hold on;
        
    end    
    r_p2=[IND(j,2),IND(j,4)]';
    c_p2=[IND(j,3),IND(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1);
end
figure, imshow(map),title('path planning_sorted viewpoints facade pointing');
hold on
j=1;
t=1;
for i=1:size(Sorted_viewpoints,1)
    if (Sorted_viewpoints(i,1)>0 && Sorted_viewpoints(i,2)>0 &&Sorted_viewpoints(i,3)>0 && Sorted_viewpoints(i,4)>0  )
        r_P=[Sorted_viewpoints(i,1),Sorted_viewpoints(i,3)]';
        c_P=[Sorted_viewpoints(i,2),Sorted_viewpoints(i,4)]';
        h= plot(Sorted_viewpoints(i,2),Sorted_viewpoints(i,1),'r*');text(Sorted_viewpoints(i,2),Sorted_viewpoints(i,1),cellstr( num2str(t)),'fontsize',18);
        set(h,'linewidth',8,'Color','red')
        hold on;
        line(c_P,r_P,'color','red','LineWidth',1);
        if (i<size(Sorted_viewpoints,1)-1 )
            if (Sorted_viewpoints(i+1,2)-Sorted_viewpoints(i,4) )^2+ (Sorted_viewpoints(i+1,1)-Sorted_viewpoints(i,3))^2 >0 || (Sorted_viewpoints(i+1,3) ==0 && Sorted_viewpoints(i+1,4)==0)
                t=t+1;                
                h= plot(Sorted_viewpoints(i,4),Sorted_viewpoints(i,3),'r*');text(Sorted_viewpoints(i,4),Sorted_viewpoints(i,3),cellstr( num2str(t)),'fontsize',18); 
                set(h,'linewidth',8,'Color','red');
                hold on;
            end
        end        
        t=t+1;
    end
end
figure, imshow(map),title('path planning_sorted facade pointing viewpoints_2');
hold on

for i=1:size(Sorted_viewpoints2,1)
    if (Sorted_viewpoints2(i,1)>0 && Sorted_viewpoints2(i,2)>0 &&Sorted_viewpoints2(i,3)>0 && Sorted_viewpoints2(i,4)>0  )
        r_P2=[Sorted_viewpoints2(i,1),Sorted_viewpoints2(i,3)]';
        c_P2=[Sorted_viewpoints2(i,2),Sorted_viewpoints2(i,4)]';
        line(c_P2,r_P2,'color','green','LineWidth',1);
        hold on;
        h= plot(Sorted_viewpoints2(i,2),Sorted_viewpoints2(i,1),'O');text(Sorted_viewpoints2(i,2),Sorted_viewpoints2(i,1),cellstr( num2str(t)),'fontsize',18); 
         set(h,'linewidth',8,'Color','green');
         hold on;
        if (i<size(Sorted_viewpoints2,1)-1 )
            if (Sorted_viewpoints2(i+1,2)-Sorted_viewpoints2(i,4) )^2+ (Sorted_viewpoints2(i+1,1)-Sorted_viewpoints2(i,3))^2 >0 || (Sorted_viewpoints2(i+1,3) ==0 && Sorted_viewpoints2(i+1,4)==0)
                t=t+1;
               
                h= plot(Sorted_viewpoints2(i,4),Sorted_viewpoints2(i,3),'O');text(Sorted_viewpoints2(i,4),Sorted_viewpoints2(i,3),cellstr( num2str(t)),'fontsize',18); 
                set(h,'linewidth',8,'Color','green');
                 hold on;
            end
        end
        t=t+1;
    end
end


%% distance between each candidate point and corresponding edge point
all_coor=[coordinate,coordinate_edge];
D=[];
for i=1:size(all_station,1)
    h1=all_coor(i,1:2);
    h2=all_coor(i,3:4);
    d=sqrt(sum((h1-h2).^2));
    D=[D;d];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% candidate points for central pointing %%%%%%%%%%%%%%%%%%%%%%%%%%%

%center_of_building =round( mean(corners,1));  % mean of each column in corners matrix
map_building_inversted=255-map_without_obstacle(:,:,1);
s = regionprops(map_building_inversted,'centroid', 'Area');
Areaa=0;
icounter=0;
for i=1:size(s)
    if(Areaa<s(i).Area)
        Areaa=s(i).Area;
        icounter=i;
    end
end
center_of_building2=round(s(icounter).Centroid);
center_of_building(1)=center_of_building2(2);
center_of_building(2)=center_of_building2(1);
candidate_center = [candid_points , repmat(center_of_building,size(candid_points,1),1)];  % candid_points there are in optimum distance

%%  camputing intersection between camera's ray in central pointing and obstacle for candid_points in optimum distance
clear  Y_coordinate X_coordinate X Y D_X D_Y Xc_max Xc_min Yc_max Yc_min
indexx1=1;
indexx2=1;
for i=1:size(candidate_center,1)
    
    Y_coordinate=[candidate_center(i,1), candidate_center(i,3)];   %image coordinate(candidate point and nearest edge point)
    X_coordinate=[candidate_center(i,2), candidate_center(i,4)];
    D_X= abs(candidate_center(i,2)- candidate_center(i,4));
    D_Y= abs(candidate_center(i,3)-candidate_center(i,1));
    Xc_max=max(candidate_center(i,2),candidate_center(i,4));
    Xc_min=min(candidate_center(i,2),candidate_center(i,4));
    Yc_max=max(candidate_center(i,1),candidate_center(i,3));
    Yc_min=min(candidate_center(i,1),candidate_center(i,3));
    if D_X ~=0
        X= Xc_min :Xc_max ;
        Y= interp1(X_coordinate,Y_coordinate,X);
        Y= round(Y);                                               % y_coordinate of line point
        else
        X=repmat(X_coordinate(1,1),1,(D_Y)+1);
        Y=Yc_min :Yc_max ;
    end
    Pixel_value =diag(im_ob(Y,X)); % Y is row of image and X is column
    if length(find(Pixel_value==0))==0
        visible_VP(indexx1,:)= candidate_center(i,:);   % candidate points without intersection with obstacle
        indexx1=indexx1+1;
    else
        unvisible_VP(indexx2,:)= candidate_center(i,:);
        indexx2=indexx2+1;
    end
end

% plot visible_VP_early on map
labels = cellstr( num2str([1:size(visible_VP,1)]'));
figure, imshow(map),title('visible_VP_central');%_without_obstacle
hold on
plot(visible_VP(:,2),visible_VP(:,1), 'g*');%text(visible_VP_early(:,2),visible_VP_early(:,1),labels);
plot(center_of_building(2),center_of_building(1), 'r*');
% draw axis of sight for visible points
for i=1:size(visible_VP);
    r_P=[visible_VP(i,1),visible_VP(i,3)]';
    c_P=[visible_VP(i,2),visible_VP(i,4)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end
%% transformation coordinate of candidate points and center of building from imege space to ground space

[Xc, Yc] = tformfwd(tform, visible_VP(:,1), visible_VP(:,2)) ;
[x_center, y_center] = tformfwd(tform, visible_VP(:,3), visible_VP(:,4)) ;

%% 3D Coordinate (condidate points and center of building)
coordinate2=[];
coordinate2(:,1)=Xc;
coordinate2(:,2)=Yc;
threeD_coordinate2=[];      %threeD_coordinate2 for candidate view point in central pointing
%height=input('please input height of building : ');
for i=0.4:1:15
    threeD_coordinate2=[threeD_coordinate2;coordinate2,i*ones(size(Xc,1),1)];
end
coordinate_center=[];
coordinate_center(:,1)=x_center;
coordinate_center(:,2)=y_center;
threeD_coordinate_center=[];
for i=0.4:1:15
    threeD_coordinate_center=[threeD_coordinate_center;coordinate_center,i*ones(size(x_center,1),1)];
end


bx2=(x_center(1)-Xc);
by2=(y_center(1)-Yc);

bz2=zeros(size(bx2,1),1);
n=1;
norm_b2=sqrt(bx2.^2+by2.^2);
norm_a=1;
unit_bx2=bx2./norm_b2;         % normal
unit_by2=by2./norm_b2;
unit_bz2=bz2./norm_b2;
unit_b2=[unit_bx2,unit_by2,unit_bz2];

for n=1:size(bx2,1)
    if bx2(n)==0
        bx2(n)=0.01;
    end
    a2=[0 0 -1];                % unit vector(Z vector(camera coordinate system)in global coordinate system)
    arobot2=[0,1,0];
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    Outx2 = a2(1,2) * unit_b2(n,3) - a2(1,3) * unit_b2(n,2);
    Outy2 = a2(1,3) * unit_b2(n,1) - a2(1,1) * unit_b2(n,3);
    Outz2 = a2(1,1) * unit_b2(n,2) - a2(1,2) * unit_b2(n,1);
    
    Outx_robot2=arobot2(1,2) * unit_b2(n,3) - arobot2(1,3) * unit_b2(n,2);
    Outy_robot2 = arobot2(1,3) * unit_b2(n,1) - arobot2(1,1) * unit_b2(n,3);
    Outz_robot2 = arobot2(1,1) * unit_b2(n,2) - arobot2(1,2) * unit_b2(n,1);

    rotation_vector2(n,:)=[Outx2 Outy2 Outz2];
    rotation_angle2(n,:)= acos( a2(1,1) * unit_b2(n,1) + a2(1,2) * unit_b2(n,2) + a2(1,3) * unit_b2(n,3));
    rotation_vector_robot2(n,:)=[Outx_robot2 Outy_robot2 Outz_robot2];
    rotation_angle_robot2(n,:)= acos( arobot2(1,1) * unit_b2(n,1) + arobot2(1,2) * unit_b2(n,2) + arobot2(1,3) * unit_b2(n,3));
    if (round(Outx_robot2) ==0 &&round(Outy_robot2)==0 &&round(Outz_robot2)==0)
        rotation_vector_robot2(n,:)=[0,0,1];
    end
end
Rvector_ex2=rotation_vector2(:,1);
Rvector_ey2=rotation_vector2(:,2);
Rvector_ez2=rotation_vector2(:,3);

q02 = cos(rotation_angle2/2);
q12 = Rvector_ex2 .* sin(rotation_angle2/2);
q22 = Rvector_ey2 .* sin(rotation_angle2/2);
q32 = Rvector_ez2 .* sin(rotation_angle2/2);

Rvector_ex_robot2=rotation_vector_robot2(:,1);
Rvector_ey_robot2=rotation_vector_robot2(:,2);
Rvector_ez_robot2=rotation_vector_robot2(:,3);

q0_robot2 = cos(rotation_angle_robot2/2);
q1_robot2 = Rvector_ex_robot2 .* sin(rotation_angle_robot2/2);
q2_robot2 = Rvector_ey_robot2 .* sin(rotation_angle_robot2/2);
q3_robot2 = Rvector_ez_robot2 .* sin(rotation_angle_robot2/2);

jjjj2= q02.^2 +q12.^2 +q22.^2 +q32.^2 ;
yaw2=atan2((2.*(q12.*q22+q02.*q32)), (q12.^2+q02.^2-q32.^2-q22.^2));       %rotation about the Z-axis (radian)
pitch2=asin(2.*(q02.*q22-q12.*q32));                                  %rotation about the Y-axis (radian)
roll2=atan2((2.*(q02.*q12+q32.*q22)), (q32.^2-q22.^2-q12.^2+q02.^2));      %rotation about the X-axis (radian)

tz2=size(threeD_coordinate2,1)/size(coordinate2,1) ;
euler2 = [roll2.*180/pi , pitch2.*180/pi , yaw2.*180/pi];             % orientation in degree
total_euler2=repmat(euler2,tz2,1);




%candidate_center_3D =[threeD_coordinate2,threeD_coordinate_center]; % all point in 3d space
labell=(1:size(threeD_coordinate2,1))';
coordinate_rotation2=[labell(1:size(coordinate2,1),:) , (threeD_coordinate2(1:size(coordinate2,1),:))*1000, total_euler2(1:size(coordinate2,1),:), q0_robot2(1:size(coordinate2,1),:),q1_robot2(1:size(coordinate2,1),:),q2_robot2(1:size(coordinate2,1),:),q3_robot2(1:size(coordinate2,1),:)];
%coordinate_rotation2=[labell(size(coordinate2,1)+1:2*size(coordinate2,1),:) , (threeD_coordinate2(1:size(coordinate2,1),:))*1000];

%% Hybrid 
Hybrid_viewspoints_pixels=[all_station;visible_VP];
Hybrid_viewpoints=[coordinate_rotation;coordinate_rotation2];
for i=size(coordinate_rotation,1)+1:size(coordinate_rotation,1)+size(coordinate_rotation2,1)
    Hybrid_viewpoints(i,1)=i;
end

%% plot selected candidate point in IND for central pointing

output_ind= textread('ind_output_central pointing_5.txt');
all_candid_pnts=[label(1:2*size(visible_VP,1),:),(repmat(visible_VP(:,:),2,1))];

for i=1:size(output_ind,1)
    [m,~] =find(all_candid_pnts(:,1)==output_ind(i,1));
    if (isempty(m)~=1)
        IND_2(i,:)=all_candid_pnts(m,:);
    end
end
figure, imshow(map),title('map');
hold on

for i=1:size(IND_2);
    if (IND_2(i,1)<indx_height_centre) % the number 227 is the maximum viewpoints index with the camera height 400 
        h= plot(IND_2(i,3),IND_2(i,2),'r*');%text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
        set(h,'linewidth',8,'Color','red')
        hold on
    else
        h= plot(IND_2(i,3),IND_2(i,2),'O');%text(IND(i,3),IND(i,2),cellstr( num2str([IND(i,1)])));
        set(h,'linewidth',8,'Color','green')
         hold on
    end        
end

%plot(IND(:,3),IND(:,2),'r*');%text(IND(:,3),IND(:,2),cellstr( num2str([IND(:,1)])));

% draw axis of sight

for i=1:size(IND_2);
    r_P=[IND_2(i,2),IND_2(i,4)]';
    c_P=[IND_2(i,3),IND_2(i,5)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end

IND_r2=IND_2;
for i=1:size(IND_2,1)
   [IND_r2(i,2), IND_r2(i,3)] = tformfwd(tform, IND_2(i,2), IND_2(i,3)) ;
   [IND_r2(i,4), IND_r2(i,5)] = tformfwd(tform, IND_2(i,4), IND_2(i,5)) ;
end
bx2=(IND_r2(:,4)-IND_r2(:,2));
by2=(IND_r2(:,5)-IND_r2(:,3));
bz2=zeros(size(IND_r2,1),1);
norm_b2=sqrt(bx2.^2+by2.^2);
norm_a2=1;
unit_bx2=bx2./norm_b2;         % normal
unit_by2=by2./norm_b2;
unit_bz2=bz2./norm_b2;
unit_b2=[unit_bx2,unit_by2,unit_bz2];

for n=1:size(bx2,1)
    if bx2(n)==0
        bx2(n)=0.01;
    end
    arobot2=[0,1,0];
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    
    Outx_robot2=arobot2(1,2) * unit_b2(n,3) - arobot2(1,3) * unit_b2(n,2);
    Outy_robot2 = arobot2(1,3) * unit_b2(n,1) - arobot2(1,1) * unit_b2(n,3);
    Outz_robot2 = arobot2(1,1) * unit_b2(n,2) - arobot2(1,2) * unit_b2(n,1);
    
	rotation_vector_robot2(n,:)=[Outx_robot2 Outy_robot2 Outz_robot2];
    rotation_angle_robot2(n,:)= acos( arobot2(1,1) * unit_b2(n,1) + arobot2(1,2) * unit_b2(n,2) + arobot2(1,3) * unit_b2(n,3));
    if (round(Outx_robot2) ==0 &&round(Outy_robot2)==0 &&round(Outz_robot2)==0)
        rotation_vector_robot2(n,:)=[0,0,1];
    end
end
Rvector_ex_robot2=rotation_vector_robot2(:,1);
Rvector_ey_robot2=rotation_vector_robot2(:,2);
Rvector_ez_robot2=rotation_vector_robot2(:,3);

q0_robot2 = cos(rotation_angle_robot2/2);
q1_robot2 = Rvector_ex_robot2 .* sin(rotation_angle_robot2/2);
q2_robot2 = Rvector_ey_robot2 .* sin(rotation_angle_robot2/2);
q3_robot2 = Rvector_ez_robot2 .* sin(rotation_angle_robot2/2);

IND_Robot_centre_pointing=[IND_r2(1:size(IND_r2,1),:), q0_robot2(1:size(IND_r2,1),:),q1_robot2(1:size(IND_r2,1),:),q2_robot2(1:size(IND_r2,1),:),q3_robot2(1:size(IND_r2,1),:)];



%% Path planning for centre pointing
figure, imshow(map),title('path planning for centre pointing');
hold on
B3(size(IND_2,1),2)=0;
B4(size(IND_2,1),2)=0;
Sorted_viewpoints3(size(IND_2,1),4)=0;
Sorted_viewpoints4(size(IND_2,1),4)=0;
for i=1:size(IND_2,1);
    if (IND_2(i,1)<indx_height_centre)
        B3(i,:) = [IND_2(i,2),IND_2(i,3)];
        h= plot(IND_2(i,3),IND_2(i,2),'r*');text(IND_2(i,3),IND_2(i,2),cellstr( num2str([IND_2(i,1)])));
        %set(h,'linewidth',8,'Color','red')
        hold on
    else
        B4(i,:) = [IND_2(i,2),IND_2(i,3)];
        h= plot(IND_2(i,3),IND_2(i,2),'O');text(IND_2(i,3),IND_2(i,2),cellstr( num2str([IND_2(i,1)])));
        %set(h,'linewidth',8,'Color','green')
         hold on
    end        
end
i=1;
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B3, B_C).^2,2));
j=find(distances_bc==min(distances_bc));
visi_matrix3=zeros(size(B,1));
visi_matrix4=zeros(size(B2,1));
while (i<size(IND_2,1))
    A3 = [IND_2(j,2),IND_2(j,3)];
    B3(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B3, A3).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
    %closest3 = B(j2(1,1),:);
    dx=IND_2(j2(1,1),2)-IND_2(j,2);
    dy=IND_2(j2(1,1),3)-IND_2(j,3);
    m= atan2(dy,dx);
    y=IND_2(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND_2(j,2)+t*cos(m));
        y=round(IND_2(j,3)+t*sin(m));
        dx=IND_2(j2(1,1),2)-x;
        dy=IND_2(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
            obstacle=1;
        end
    end
    if (obstacle==0) 
        r_P=[IND_2(j,2),IND_2(j2(1,1),2)]';
        c_P=[IND_2(j,3),IND_2(j2(1,1),3)]';        
        line(c_P,r_P,'color','green','LineWidth',1);
        hold on; 
        Sorted_viewpoints3(i,:)=[A3,B3(j2(1,1),:)];
        visi_matrix3(j,j2(1,1))=1; 
        visi_matrix3(j2(1,1),j)=1; 
        i=i+1;
    end
    r_p2=[IND_2(j,2),IND_2(j,4)]';
    c_p2=[IND_2(j,3),IND_2(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1);
end

i=1;
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B4, B_C).^2,2));
j=find(distances_bc==min(distances_bc));

while (i<size(IND_2,1))
    A3 = [IND_2(j,2),IND_2(j,3)];
    B4(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B4, A3).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
   %closest3 = B(j2(1,1),:);
    dx=IND_2(j2(1,1),2)-IND_2(j,2);
    dy=IND_2(j2(1,1),3)-IND_2(j,3);
    m= atan2(dy,dx);
    y=IND_2(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND_2(j,2)+t*cos(m));
        y=round(IND_2(j,3)+t*sin(m));
        dx=IND_2(j2(1,1),2)-x;
        dy=IND_2(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
           obstacle=1;
        end
    end
    if (obstacle==0)        
        Sorted_viewpoints4(i,:)=[A3,B4(j2(1,1),:)];
        visi_matrix4(j,j2(1,1))=1; 
        visi_matrix4(j2(1,1),j)=1;
        i=i+1;
        r_P=[IND_2(j,2),IND_2(j2(1,1),2)]';
        c_P=[IND_2(j,3),IND_2(j2(1,1),3)]';
        line(c_P,r_P,'color','red','LineWidth',1);        
        hold on; 
    end    
    r_p2=[IND_2(j,2),IND_2(j,4)]';
    c_p2=[IND_2(j,3),IND_2(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1);
end
figure, imshow(map),title('path planning_sorted viewpoints');
hold on
j=1;
t=1;
for i=1:size(Sorted_viewpoints3,1)
    if (Sorted_viewpoints3(i,1)>0 && Sorted_viewpoints3(i,2)>0 &&Sorted_viewpoints3(i,3)>0 && Sorted_viewpoints3(i,4)>0  )
        r_P=[Sorted_viewpoints3(i,1),Sorted_viewpoints3(i,3)]';
        c_P=[Sorted_viewpoints3(i,2),Sorted_viewpoints3(i,4)]';
        h= plot(Sorted_viewpoints3(i,2),Sorted_viewpoints3(i,1),'gO');text(Sorted_viewpoints3(i,2),Sorted_viewpoints3(i,1),cellstr( num2str(t)),'fontsize',18);
        hold on;
        line(c_P,r_P,'color','red','LineWidth',1);
        t=t+1;
        if (i<size(Sorted_viewpoints2,1)-1 )
            if (Sorted_viewpoints3(i+1,2)-Sorted_viewpoints3(i,4) )^2+ (Sorted_viewpoints3(i+1,1)-Sorted_viewpoints3(i,3))^2 >0 || (Sorted_viewpoints3(i+1,3) ==0 && Sorted_viewpoints3(i+1,4)==0)
                h= plot(Sorted_viewpoints3(i,4),Sorted_viewpoints3(i,3),'go');text(Sorted_viewpoints3(i,4),Sorted_viewpoints3(i,3),cellstr( num2str(t)),'fontsize',18); 
                t=t+1;                                
                hold on;
            end
        end
    end
end
figure, imshow(map),title('path planning_sorted viewpoints_2');
hold on
for i=1:size(Sorted_viewpoints4,1)
    if (Sorted_viewpoints4(i,1)>0 && Sorted_viewpoints4(i,2)>0 &&Sorted_viewpoints4(i,3)>0 && Sorted_viewpoints4(i,4)>0  )
        r_P2=[Sorted_viewpoints4(i,1),Sorted_viewpoints4(i,3)]';
        c_P2=[Sorted_viewpoints4(i,2),Sorted_viewpoints4(i,4)]';
        line(c_P2,r_P2,'color','green','LineWidth',1);
        hold on;
        h= plot(Sorted_viewpoints4(i,2),Sorted_viewpoints4(i,1),'r*');text(Sorted_viewpoints4(i,2),Sorted_viewpoints4(i,1),cellstr( num2str(t)),'fontsize',18);
        t=t+1;
        if (i<size(Sorted_viewpoints4,1)-1 )
            if (Sorted_viewpoints4(i+1,2)-Sorted_viewpoints4(i,4) )^2+ (Sorted_viewpoints4(i+1,1)-Sorted_viewpoints4(i,3))^2 >0 || (Sorted_viewpoints4(i+1,3) ==0 && Sorted_viewpoints4(i+1,4)==0)
                h= plot(Sorted_viewpoints4(i,4),Sorted_viewpoints4(i,3),'r*');text(Sorted_viewpoints4(i,4),Sorted_viewpoints4(i,3),cellstr( num2str(t)),'fontsize',18); 
                t=t+1;                
                hold on;
            end
        end
    end
end

%% transformation for move_base
j=1;
for i=1:size(Sorted_viewpoints,1)
    if (Sorted_viewpoints(i,3)~=0 && Sorted_viewpoints(i,4)~=0)
        [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,1), Sorted_viewpoints(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints(i-1,3)~=0 && Sorted_viewpoints(i-1,4)~=0)&& ((Sorted_viewpoints(i,3)==0 && Sorted_viewpoints(i,4)==0))
            [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,1), Sorted_viewpoints(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints,1)-1 )
         if ((Sorted_viewpoints(i+1,2)-Sorted_viewpoints(i,4) )^2+ (Sorted_viewpoints(i+1,1)-Sorted_viewpoints(i,3))^2 >0) &&((Sorted_viewpoints(i,3)~=0 && Sorted_viewpoints(i,4)~=0))
               [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,3), Sorted_viewpoints(i,4)) ;   
               j=j+1;
         end
     end
end

j=1;
for i=1:size(Sorted_viewpoints2,1)
    if (Sorted_viewpoints2(i,3)~=0 && Sorted_viewpoints2(i,4)~=0)
        [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,1), Sorted_viewpoints2(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints2(i-1,3)~=0 && Sorted_viewpoints2(i-1,4)~=0)&& ((Sorted_viewpoints2(i,3)==0 && Sorted_viewpoints2(i,4)==0))
            [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,1), Sorted_viewpoints2(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints2,1)-1 )
         if ((Sorted_viewpoints2(i+1,2)-Sorted_viewpoints2(i,4) )^2+ (Sorted_viewpoints2(i+1,1)-Sorted_viewpoints2(i,3))^2 >0) &&((Sorted_viewpoints2(i,3)~=0 && Sorted_viewpoints2(i,4)~=0))
               [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,3), Sorted_viewpoints2(i,4)) ;   
               j=j+1;
         end
     end
end
j=1;
for i=1:size(Sorted_viewpoints3,1)
    if (Sorted_viewpoints3(i,3)~=0 && Sorted_viewpoints3(i,4)~=0)
        [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,1), Sorted_viewpoints3(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints3(i-1,3)~=0 && Sorted_viewpoints3(i-1,4)~=0)&& ((Sorted_viewpoints3(i,3)==0 && Sorted_viewpoints3(i,4)==0))
            [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,1), Sorted_viewpoints3(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints3,1)-1 )
         if ((Sorted_viewpoints3(i+1,2)-Sorted_viewpoints3(i,4) )^2+ (Sorted_viewpoints3(i+1,1)-Sorted_viewpoints3(i,3))^2 >0) &&((Sorted_viewpoints3(i,3)~=0 && Sorted_viewpoints3(i,4)~=0))
               [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,3), Sorted_viewpoints3(i,4)) ;   
               j=j+1;
         end
     end
end
j=1;
for i=1:size(Sorted_viewpoints4,1)
    if (Sorted_viewpoints4(i,3)~=0 && Sorted_viewpoints4(i,4)~=0)
        [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,1), Sorted_viewpoints4(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints4(i-1,3)~=0 && Sorted_viewpoints4(i-1,4)~=0)&& ((Sorted_viewpoints4(i,3)==0 && Sorted_viewpoints4(i,4)==0))
            [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,1), Sorted_viewpoints4(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints4,1)-1 )
         if ((Sorted_viewpoints4(i+1,2)-Sorted_viewpoints4(i,4) )^2+ (Sorted_viewpoints4(i+1,1)-Sorted_viewpoints4(i,3))^2 >0) &&((Sorted_viewpoints4(i,3)~=0 && Sorted_viewpoints4(i,4)~=0))
               [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,3), Sorted_viewpoints4(i,4)) ;   
               j=j+1;
         end
     end
end

%% Hybrid pointing
% plot selected candidate point in IND for Hybrid pointing

output_ind_hybrid= textread('ind_output_Hybrid.txt');

all_candid_pnts_hybrid=[label(1:2*size(Hybrid_viewspoints_pixels,1),:),(repmat(Hybrid_viewspoints_pixels(:,:),2,1))];

for i=1:size(output_ind_hybrid,1)
    [m,~] =find(all_candid_pnts_hybrid(:,1)==output_ind_hybrid(i,1));
    if (isempty(m)~=1)
        IND_3(i,:)=all_candid_pnts_hybrid(m,:);
    end
end
figure, imshow(map),title('map');
hold on

for i=1:size(IND_3);
    if (IND_3(i,1)<indx_height_hybrid) 
        h= plot(IND_3(i,3),IND_3(i,2),'r*');
        set(h,'linewidth',8,'Color','red')
        hold on
    else
        h= plot(IND_3(i,3),IND_3(i,2),'O');
        set(h,'linewidth',8,'Color','green')
         hold on
    end        
end

%plot(IND(:,3),IND(:,2),'r*');%text(IND(:,3),IND(:,2),cellstr( num2str([IND(:,1)])));

% draw axis of sight

for i=1:size(IND_3);
    r_P=[IND_3(i,2),IND_3(i,4)]';
    c_P=[IND_3(i,3),IND_3(i,5)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end

IND_r3=IND_3;
for i=1:size(IND_3,1)
   [IND_r3(i,2), IND_r3(i,3)] = tformfwd(tform, IND_3(i,2), IND_3(i,3)) ;
   [IND_r3(i,4), IND_r3(i,5)] = tformfwd(tform, IND_3(i,4), IND_3(i,5)) ;
end
bx3=(IND_r3(:,4)-IND_r3(:,2));
by3=(IND_r3(:,5)-IND_r3(:,3));
bz3=zeros(size(IND_r3,1),1);
norm_b3=sqrt(bx3.^2+by3.^2);
norm_a3=1;
unit_bx3=bx3./norm_b3;         % normal
unit_by3=by3./norm_b3;
unit_bz3=bz3./norm_b3;
unit_b3=[unit_bx3,unit_by3,unit_bz3];

for n=1:size(bx3,1)
    if bx3(n)==0
        bx3(n)=0.01;
    end
    arobot3=[0,1,0];
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    
    Outx_robot3=arobot3(1,2) * unit_b3(n,3) - arobot3(1,3) * unit_b3(n,2);
    Outy_robot3 = arobot3(1,3) * unit_b3(n,1) - arobot3(1,1) * unit_b3(n,3);
    Outz_robot3 = arobot3(1,1) * unit_b3(n,2) - arobot3(1,2) * unit_b3(n,1);
    
	rotation_vector_robot3(n,:)=[Outx_robot3 Outy_robot3 Outz_robot3];
    rotation_angle_robot3(n,:)= acos( arobot3(1,1) * unit_b3(n,1) + arobot3(1,2) * unit_b3(n,2) + arobot3(1,3) * unit_b3(n,3));
    if (round(Outx_robot3) ==0 &&round(Outy_robot3)==0 &&round(Outz_robot3)==0)
        rotation_vector_robot3(n,:)=[0,0,1];
    end
end
Rvector_ex_robot3=rotation_vector_robot3(:,1);
Rvector_ey_robot3=rotation_vector_robot3(:,2);
Rvector_ez_robot3=rotation_vector_robot3(:,3);

q0_robot3 = cos(rotation_angle_robot3/2);
q1_robot3 = Rvector_ex_robot3 .* sin(rotation_angle_robot3/2);
q2_robot3 = Rvector_ey_robot3 .* sin(rotation_angle_robot3/2);
q3_robot3 = Rvector_ez_robot3 .* sin(rotation_angle_robot3/2);

IND_Robot_hybrid_pointing=[IND_r3(1:size(IND_r3,1),:), q0_robot3(1:size(IND_r3,1),:),q1_robot3(1:size(IND_r3,1),:),q2_robot3(1:size(IND_r3,1),:),q3_robot3(1:size(IND_r3,1),:)];



%% Path planning for hybrid pointing
figure, imshow(map),title('path planning for hybrid pointing');
hold on
B5(size(IND_3,1),2)=0;
B6(size(IND_3,1),2)=0;
Sorted_viewpoints5(size(IND_3,1),4)=0;
Sorted_viewpoints6(size(IND_3,1),4)=0;
for i=1:size(IND_3,1);
    if (IND_3(i,1)<indx_height_hybrid)
        B5(i,:) = [IND_3(i,2),IND_3(i,3)];
        h= plot(IND_3(i,3),IND_3(i,2),'r*');text(IND_3(i,3),IND_3(i,2),cellstr( num2str([IND_3(i,1)])));
        %set(h,'linewidth',8,'Color','red')
        hold on
    else
        B6(i,:) = [IND_3(i,2),IND_3(i,3)];
        h= plot(IND_3(i,3),IND_3(i,2),'O');text(IND_3(i,3),IND_3(i,2),cellstr( num2str([IND_3(i,1)])));
        %set(h,'linewidth',8,'Color','green')
         hold on
    end        
end
i=1;
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B5, B_C).^2,2));
j=find(distances_bc==min(distances_bc));
visi_matrix5=zeros(size(B5,1));
visi_matrix6=zeros(size(B6,1));
while (i<size(IND_3,1))
    A4 = [IND_3(j,2),IND_3(j,3)];
    B5(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B5, A4).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
    %closest3 = B(j2(1,1),:);
    dx=IND_3(j2(1,1),2)-IND_3(j,2);
    dy=IND_3(j2(1,1),3)-IND_3(j,3);
    m= atan2(dy,dx);
    y=IND_3(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND_3(j,2)+t*cos(m));
        y=round(IND_3(j,3)+t*sin(m));
        dx=IND_3(j2(1,1),2)-x;
        dy=IND_3(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
            obstacle=1;
        end
    end
    if (obstacle==0) 
        r_P=[IND_3(j,2),IND_3(j2(1,1),2)]';
        c_P=[IND_3(j,3),IND_3(j2(1,1),3)]';        
        line(c_P,r_P,'color','green','LineWidth',1);
        hold on; 
        Sorted_viewpoints5(i,:)=[A4,B5(j2(1,1),:)];
        visi_matrix5(j,j2(1,1))=1; 
        visi_matrix5(j2(1,1),j)=1; 
        i=i+1;
    end
    r_p3=[IND_3(j,2),IND_3(j,4)]';
    c_p3=[IND_3(j,3),IND_3(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1);
end

i=1;
B_C=[1734,2328]; % a point in the corner of building for starting point
distances_bc = sqrt(sum(bsxfun(@minus, B6, B_C).^2,2));
j_=find(distances_bc==min(distances_bc));
j=j_(1);
while (i<size(IND_3,1))
    A4 = [IND_3(j,2),IND_3(j,3)];
    B6(j,:)=0;
    %compute Euclidean distances:
    distances3 = sqrt(sum(bsxfun(@minus, B6, A4).^2,2));
    j2=find(distances3==min(distances3));
    %find the smallest distance and use that as an index into B:
   %closest3 = B(j2(1,1),:);
    dx=IND_3(j2(1,1),2)-IND_3(j,2);
    dy=IND_3(j2(1,1),3)-IND_3(j,3);
    m= atan2(dy,dx);
    y=IND_3(j,3);
    l=sqrt(dx^2+dy^2);
    t=1;
    obstacle=0;
    while(l>0)
        x=round(IND_3(j,2)+t*cos(m));
        y=round(IND_3(j,3)+t*sin(m));
        dx=IND_3(j2(1,1),2)-x;
        dy=IND_3(j2(1,1),3)-y;
        l=sqrt(dx^2+dy^2);
        t=t+1;  
        if (map(x,y)==0)
           obstacle=1;
        end
    end
    if (obstacle==0)        
        Sorted_viewpoints6(i,:)=[A4,B6(j2(1,1),:)];
        visi_matrix6(j,j2(1,1))=1; 
        visi_matrix6(j2(1,1),j)=1;
        i=i+1;
        r_P=[IND_3(j,2),IND_3(j2(1,1),2)]';
        c_P=[IND_3(j,3),IND_3(j2(1,1),3)]';
        line(c_P,r_P,'color','red','LineWidth',1);        
        hold on; 
    end    
    r_p3=[IND_3(j,2),IND_3(j,4)]';
    c_p3=[IND_3(j,3),IND_3(j,5)]';
    %line(c_p2,r_p2,'color','blue','LineWidth',0.2);
    j=j2(1,1);
end
hold off
figure, imshow(map),title('path planning_sorted viewpoints_hybrid');
hold on
j=1;
t=1;
for i=1:size(Sorted_viewpoints5,1)
    if (Sorted_viewpoints5(i,1)>0 && Sorted_viewpoints5(i,2)>0 &&Sorted_viewpoints5(i,3)>0 && Sorted_viewpoints5(i,4)>0  )
        r_P=[Sorted_viewpoints5(i,1),Sorted_viewpoints5(i,3)]';
        c_P=[Sorted_viewpoints5(i,2),Sorted_viewpoints5(i,4)]';
        h= plot(Sorted_viewpoints5(i,2),Sorted_viewpoints5(i,1),'gO');text(Sorted_viewpoints5(i,2),Sorted_viewpoints5(i,1),cellstr( num2str(t)),'fontsize',18);
        hold on;
        line(c_P,r_P,'color','red','LineWidth',1);
        t=t+1;
        if (i<size(Sorted_viewpoints5,1)-1 )
            if (Sorted_viewpoints5(i+1,2)-Sorted_viewpoints5(i,4) )^2+ (Sorted_viewpoints5(i+1,1)-Sorted_viewpoints5(i,3))^2 >0 || (Sorted_viewpoints5(i+1,3) ==0 && Sorted_viewpoints5(i+1,4)==0)
                h= plot(Sorted_viewpoints5(i,4),Sorted_viewpoints5(i,3),'go');text(Sorted_viewpoints5(i,4),Sorted_viewpoints5(i,3),cellstr( num2str(t)),'fontsize',18); 
                t=t+1;                                
                hold on;
            end
        end
    end
end
figure, imshow(map),title('path planning_sorted viewpoints_2_hybrid');
hold on
for i=1:size(Sorted_viewpoints6,1)
    if (Sorted_viewpoints6(i,1)>0 && Sorted_viewpoints6(i,2)>0 &&Sorted_viewpoints6(i,3)>0 && Sorted_viewpoints6(i,4)>0  )
        r_P2=[Sorted_viewpoints6(i,1),Sorted_viewpoints6(i,3)]';
        c_P2=[Sorted_viewpoints6(i,2),Sorted_viewpoints6(i,4)]';
        line(c_P2,r_P2,'color','green','LineWidth',1);
        hold on;
        h= plot(Sorted_viewpoints6(i,2),Sorted_viewpoints6(i,1),'r*');text(Sorted_viewpoints6(i,2),Sorted_viewpoints6(i,1),cellstr( num2str(t)),'fontsize',18);
        t=t+1;
        if (i<size(Sorted_viewpoints6,1)-1 )
            if (Sorted_viewpoints6(i+1,2)-Sorted_viewpoints6(i,4) )^2+ (Sorted_viewpoints6(i+1,1)-Sorted_viewpoints6(i,3))^2 >0 || (Sorted_viewpoints6(i+1,3) ==0 && Sorted_viewpoints6(i+1,4)==0)
                h= plot(Sorted_viewpoints6(i,4),Sorted_viewpoints6(i,3),'r*');text(Sorted_viewpoints6(i,4),Sorted_viewpoints6(i,3),cellstr( num2str(t)),'fontsize',18); 
                t=t+1;                
                hold on;
            end
        end
    end
end

%% transformation for move_base
j=1;
for i=1:size(Sorted_viewpoints,1)
    if (Sorted_viewpoints(i,3)~=0 && Sorted_viewpoints(i,4)~=0)
        [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,1), Sorted_viewpoints(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints(i-1,3)~=0 && Sorted_viewpoints(i-1,4)~=0)&& ((Sorted_viewpoints(i,3)==0 && Sorted_viewpoints(i,4)==0))
            [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,1), Sorted_viewpoints(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints,1)-1 )
         if ((Sorted_viewpoints(i+1,2)-Sorted_viewpoints(i,4) )^2+ (Sorted_viewpoints(i+1,1)-Sorted_viewpoints(i,3))^2 >0) &&((Sorted_viewpoints(i,3)~=0 && Sorted_viewpoints(i,4)~=0))
               [xynavigation1(j,1), xynavigation1(j,2)] = tformfwd(tform, Sorted_viewpoints(i,3), Sorted_viewpoints(i,4)) ;   
               j=j+1;
         end
     end
end

j=1;
for i=1:size(Sorted_viewpoints2,1)
    if (Sorted_viewpoints2(i,3)~=0 && Sorted_viewpoints2(i,4)~=0)
        [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,1), Sorted_viewpoints2(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints2(i-1,3)~=0 && Sorted_viewpoints2(i-1,4)~=0)&& ((Sorted_viewpoints2(i,3)==0 && Sorted_viewpoints2(i,4)==0))
            [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,1), Sorted_viewpoints2(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints2,1)-1 )
         if ((Sorted_viewpoints2(i+1,2)-Sorted_viewpoints2(i,4) )^2+ (Sorted_viewpoints2(i+1,1)-Sorted_viewpoints2(i,3))^2 >0) &&((Sorted_viewpoints2(i,3)~=0 && Sorted_viewpoints2(i,4)~=0))
               [xynavigation2(j,1), xynavigation2(j,2)] = tformfwd(tform, Sorted_viewpoints2(i,3), Sorted_viewpoints2(i,4)) ;   
               j=j+1;
         end
     end
end
j=1;
for i=1:size(Sorted_viewpoints3,1)
    if (Sorted_viewpoints3(i,3)~=0 && Sorted_viewpoints3(i,4)~=0)
        [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,1), Sorted_viewpoints3(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints3(i-1,3)~=0 && Sorted_viewpoints3(i-1,4)~=0)&& ((Sorted_viewpoints3(i,3)==0 && Sorted_viewpoints3(i,4)==0))
            [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,1), Sorted_viewpoints3(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints3,1)-1 )
         if ((Sorted_viewpoints3(i+1,2)-Sorted_viewpoints3(i,4) )^2+ (Sorted_viewpoints3(i+1,1)-Sorted_viewpoints3(i,3))^2 >0) &&((Sorted_viewpoints3(i,3)~=0 && Sorted_viewpoints3(i,4)~=0))
               [xynavigation3(j,1), xynavigation3(j,2)] = tformfwd(tform, Sorted_viewpoints3(i,3), Sorted_viewpoints3(i,4)) ;   
               j=j+1;
         end
     end
end
j=1;
for i=1:size(Sorted_viewpoints4,1)
    if (Sorted_viewpoints4(i,3)~=0 && Sorted_viewpoints4(i,4)~=0)
        [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,1), Sorted_viewpoints4(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints4(i-1,3)~=0 && Sorted_viewpoints4(i-1,4)~=0)&& ((Sorted_viewpoints4(i,3)==0 && Sorted_viewpoints4(i,4)==0))
            [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,1), Sorted_viewpoints4(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints4,1)-1 )
         if ((Sorted_viewpoints4(i+1,2)-Sorted_viewpoints4(i,4) )^2+ (Sorted_viewpoints4(i+1,1)-Sorted_viewpoints4(i,3))^2 >0) &&((Sorted_viewpoints4(i,3)~=0 && Sorted_viewpoints4(i,4)~=0))
               [xynavigation4(j,1), xynavigation4(j,2)] = tformfwd(tform, Sorted_viewpoints4(i,3), Sorted_viewpoints4(i,4)) ;   
               j=j+1;
         end
     end
end


j=1;
for i=1:size(Sorted_viewpoints5,1)
    if (Sorted_viewpoints5(i,3)~=0 && Sorted_viewpoints5(i,4)~=0)
        [xynavigation5(j,1), xynavigation5(j,2)] = tformfwd(tform, Sorted_viewpoints5(i,1), Sorted_viewpoints5(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints5(i-1,3)~=0 && Sorted_viewpoints5(i-1,4)~=0)&& ((Sorted_viewpoints5(i,3)==0 && Sorted_viewpoints5(i,4)==0))
            [xynavigation5(j,1), xynavigation5(j,2)] = tformfwd(tform, Sorted_viewpoints5(i,1), Sorted_viewpoints5(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints5,1)-1 )
         if ((Sorted_viewpoints5(i+1,2)-Sorted_viewpoints5(i,4) )^2+ (Sorted_viewpoints5(i+1,1)-Sorted_viewpoints5(i,3))^2 >0) &&((Sorted_viewpoints5(i,3)~=0 && Sorted_viewpoints5(i,4)~=0))
               [xynavigation5(j,1), xynavigation5(j,2)] = tformfwd(tform, Sorted_viewpoints5(i,3), Sorted_viewpoints5(i,4)) ;   
               j=j+1;
         end
     end
end
j=1;
for i=1:size(Sorted_viewpoints6,1)
    if (Sorted_viewpoints6(i,3)~=0 && Sorted_viewpoints6(i,4)~=0)
        [xynavigation6(j,1), xynavigation6(j,2)] = tformfwd(tform, Sorted_viewpoints6(i,1), Sorted_viewpoints6(i,2)) ;    
        j=j+1;
    end
    if i>1
        if (Sorted_viewpoints6(i-1,3)~=0 && Sorted_viewpoints6(i-1,4)~=0)&& ((Sorted_viewpoints6(i,3)==0 && Sorted_viewpoints6(i,4)==0))
            [xynavigation6(j,1), xynavigation6(j,2)] = tformfwd(tform, Sorted_viewpoints6(i,1), Sorted_viewpoints6(i,2)) ;    
            j=j+1;
        end
    end
    
     if (i<size(Sorted_viewpoints6,1)-1 )
         if ((Sorted_viewpoints6(i+1,2)-Sorted_viewpoints6(i,4) )^2+ (Sorted_viewpoints6(i+1,1)-Sorted_viewpoints6(i,3))^2 >0) &&((Sorted_viewpoints6(i,3)~=0 && Sorted_viewpoints6(i,4)~=0))
               [xynavigation6(j,1), xynavigation6(j,2)] = tformfwd(tform, Sorted_viewpoints6(i,3), Sorted_viewpoints6(i,4)) ;   
               j=j+1;
         end
     end
end


%% Move_base data generation for facade pointing
fileID = fopen('navigation_facade_0.4.txt','w');
formatSpec = 'waypoints:\n';
indx=0;
fprintf(fileID,formatSpec);
for i=1:size(xynavigation1,1)
    [m,~] =find(round(xynavigation1(i,1))==round(coordinate_rotation(:,2)/1000));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=coordinate_rotation(m,:);
        [m2,~] =find(round(xynavigation1(i,2))==round(navigation(:,3)/1000));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);
                       
            if (isnan(q(1))~=1)
                indx=indx+1;
                formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
                fprintf(fileID,formatSpec, indx,round(navigation_final(1,2)/1000), round(navigation_final(1,3)/1000),navigation_final(1,9),navigation_final(1,10),navigation_final(1,11),navigation_final(1,8));
            end
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID,formatSpec, j);
end
fileID2 = fopen('navigation_facade_1.5.txt','w');
formatSpec = 'waypoints:\n';
fprintf(fileID2,formatSpec);
for i=1:size(xynavigation2,1)
    [m,~] =find(round(xynavigation2(i,1))==round(coordinate_rotation(:,2)/1000));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=coordinate_rotation(m,:);
        [m2,~] =find(round(xynavigation2(i,2))==round(navigation(:,3)/1000));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);
            if (isnan(q(1))~=1)
                indx=indx+1;
                formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
                fprintf(fileID2,formatSpec, indx,round(navigation_final(1,2)/1000), round(navigation_final(1,3)/1000),navigation_final(1,9),navigation_final(1,10),navigation_final(1,11),navigation_final(1,8));
            end
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID2,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID2,formatSpec, j);
end


%% Move_base data generation for centre pointing

fileID3 = fopen('navigation_centre_0.4.txt','w');
formatSpec = 'waypoints:\n';
indx=0;
fprintf(fileID3,formatSpec);
for i=1:size(xynavigation3,1)
    [m,~] =find(xynavigation3(i,1)==IND_Robot_centre_pointing(:,2));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=IND_Robot_centre_pointing(m,:);
        [m2,~] =find(xynavigation3(i,2)==navigation(:,3));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);                       
            indx=indx+1;
            formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
            fprintf(fileID3,formatSpec, indx,round(navigation_final(1,2)), round(navigation_final(1,3)),navigation_final(1,7),navigation_final(1,8),navigation_final(1,9),navigation_final(1,6));
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID3,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID3,formatSpec, j);
end
fileID4 = fopen('navigation_centre_1.5.txt','w');
formatSpec = 'waypoints:\n';
fprintf(fileID4,formatSpec);

for i=1:size(xynavigation4,1)
    [m,~] =find(xynavigation4(i,1)==IND_Robot_centre_pointing(:,2));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=IND_Robot_centre_pointing(m,:);
        [m2,~] =find(xynavigation4(i,2)==navigation(:,3));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);
            indx=indx+1;
            formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
            fprintf(fileID4,formatSpec, indx,round(navigation_final(1,2)), round(navigation_final(1,3)),navigation_final(1,7),navigation_final(1,8),navigation_final(1,9),navigation_final(1,6));
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID4,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID4,formatSpec, j);
end

%% Move_base data generation for hybrid pointing

fileID5 = fopen('navigation_hybrid_0.4.txt','w');
formatSpec = 'waypoints:\n';
indx=0;
fprintf(fileID5,formatSpec);
for i=1:size(xynavigation5,1)
    [m,~] =find(xynavigation5(i,1)==IND_Robot_hybrid_pointing(:,2));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=IND_Robot_hybrid_pointing(m,:);
        [m2,~] =find(xynavigation5(i,2)==navigation(:,3));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);                       
            indx=indx+1;
            formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
            fprintf(fileID5,formatSpec, indx,round(navigation_final(1,2)), round(navigation_final(1,3)),navigation_final(1,7),navigation_final(1,8),navigation_final(1,9),navigation_final(1,6));
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID5,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID5,formatSpec, j);
end
fileID6 = fopen('navigation_hybrid_1.5.txt','w');
formatSpec = 'waypoints:\n';
fprintf(fileID6,formatSpec);

for i=1:size(xynavigation6,1)
    [m,~] =find(xynavigation6(i,1)==IND_Robot_hybrid_pointing(:,2));
    clear navigation navigation_final;
    if (isempty(m)~=1)
        navigation=IND_Robot_hybrid_pointing(m,:);
        [m2,~] =find(xynavigation6(i,2)==navigation(:,3));
        if (isempty(m2)~=1)
            navigation_final=navigation(m2,:);
            indx=indx+1;
            formatSpec = '  - name: sta%d\n    frame_id: map\n    pose:\n      position:\n       x: %2.0f\n       y: %2.0f\n       z: 0\n      orientation:\n       x: %4.2f\n       y: %4.2f\n       z: %4.2f\n       w: %4.2f \n';
            fprintf(fileID6,formatSpec, indx,round(navigation_final(1,2)), round(navigation_final(1,3)),navigation_final(1,7),navigation_final(1,8),navigation_final(1,9),navigation_final(1,6));
        end
              
    end
end
formatSpec = 'trajectories:\n  - name: go_to_room1\n    waypoints:';
fprintf(fileID6,formatSpec);
for j=1:indx
    formatSpec='\n    - sta%d';
    fprintf(fileID6,formatSpec, j);
end
