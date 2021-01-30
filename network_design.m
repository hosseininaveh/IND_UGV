
%  clc
clear all
close all
map= imread('output.jpg');
map_without_obstacle=imread('output_without_obstacles.jpg');
level = graythresh(map);
binary_map= im2bw(map,level);
grid_point=zeros(size(map,1),size(map,2));
for i= 1:12:size(map,2)
    for j=1:15:size(map,1)
        if binary_map(j,i)==1;
            grid_point(j,i)=255;
        end
    end
end
% figure,imshow(grid_point),title('grid_point')
%% plot grid_point on map
% [r,c]=find(grid_point==255);
% grid_point=[r,c];
% imshow(map),title('map');
% hold on
% plot(grid_point(:,2),grid_point(:,1), 'g*');

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
D=7000 %35000 ;          % maximum length of the object
k=1 ;                % number of images at each points
q=0.6 ;             % design factor (0.4 - 0.7)
Sp= 14000;  %0.003       % specified precision number(25000-200000)
sigma=0.5 *0.0039;        % image measurment error .... 1.2-1.3 pixel size

Dmax_scale = (D* f * sqrt(k))/(q * Sp *sigma);

% D_spatial resolution
Dtos=4 ;          % target object size - mm
Dtis=1;          % target image size ,minimum number of target's pixel =(5-10)pixel
Ires=0.0039 ;  % pixel size ,
phi=pi/2  ;          % angle between ray coming from the camera and target plane

Dmax_reso =(f* Dtos *sin(phi))/(Ires * Dtis);

% D_FOV
d0=15.6 ;          % minimum frame size of image
D0=5000 %15000;
alpha= atan((0.9*d0)/(2*f));    % FOV angle of camera (F0V/2 )

D_fov = (D0* sin(alpha+phi))/(2*sin(alpha));

% D_MAX
Dmax= min (Dmax_reso , Dmax_scale );
D_max= min(Dmax , D_fov);

% D_DOF
f_stop=8;           % aperture stop...f/number(D)...D=1.4 , 2 , 2.8 , 4 , 5.6 , 8 , 16
bc=f/1720;% 0.022;          % blur circle=~0.2 or = pixel size
D_HF= (f^2)/(f_stop * bc);      %hyperfocal distance
D_opt=D_max;                      %camera distance focus , or >>> D_OP=D_HF+f
Dmin_DOF=(D_opt*D_HF)/(D_HF+(D_opt-f));
Dmax_DOF=(D_opt*D_HF)/(D_HF-(D_opt-f));

D_DOF=(Dmax_DOF - Dmin_DOF);       % Depth Of Field

% D_FOV_min     %% workspace_constaint?
d0=15.6 ;          % minimum frame size of image
D00=2000;
alpha= atan((0.9*d0)/(2*f));    % FOV angle of camera (F0V/2 )

D_fov_min = (D00* sin(alpha+phi))/(2*sin(alpha));

% D_minpoint
n=4 ;             % number of image point
unit_a=3000;              % distance between points in object space

Dmin_point = (unit_a*f*sqrt(n))/(d0);

%D_min
D_min= max (Dmin_DOF , D_fov_min);%Dmin_point );
%D_min= Dmin_DOF;

Range = D_max - D_min ;

D_max_p = D_max/114.23 ;             % pixel size in map=114.23 %60.7
D_min_p = D_min/114.23 ;

DM = round(D_max_p);    % round D_MAX in pixel
Dm = round(D_min_p);    % round D_min in pixel
%% candidate points in Optimum Distance
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
target_range=dilation2-dilation1;
%figure,imshow(target_range),title('target-range')
candidate_points=(target_range(:,:,1)) & (grid_point) ;
%figure,imshow(candidate_points),title('candidate_points')

%% plot candid_points with label
[r,c]=find(candidate_points==1);
candid_points=[r,c];
% labels = cellstr( num2str([1:size(candid_points,1)]'));
% imshow(map),title('map');
% hold on
% plot(candid_points(:,2),candid_points(:,1), 'g*');text(candid_points(:,2),candid_points(:,1),labels);

%% compute edge point, minimum distance and select the nearest edge point
% edge detection
im_new= rgb2gray(im_morph);
edge1 = edge(im_new,'canny');
% imshow(edge1);
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
%labels = cellstr( num2str([1:size(candidate_edge_image,1)]'));
% imshow(map),title('obstacle');%_without_obstacle
% hold on
% plot(candidate_edge_image(:,2),candidate_edge_image(:,1), 'g*');text(candidate_edge_image(:,2),candidate_edge_image(:,1),labels);
% %draw axis of sight
% for i=1:size(candidate_edge_image);
% r_P=[candidate_edge_image(i,1),candidate_edge_image(i,3)]';
% c_P=[candidate_edge_image(i,2),candidate_edge_image(i,4)]';
%  line(c_P,r_P,'color','black','LineWidth',1);
% end
%%  camputing intersection between camera's ray and obstacle for prima candidate points
img_obstacle=imread('output_obstacles.jpg');
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
    if D_X ~=0 %&& candidate_edge_image(i,2)<candidate_edge_image(i,4)
        %X=candidate_edge_image(i,2):candidate_edge_image(i,4);           % X_coordinate of line point
        X= Xc_min :Xc_max ;
        Y= interp1(X_coordinate,Y_coordinate,X);
        Y= round(Y);                                   % y_coordinate of line point
        
        %elseif  D_X~=0 && candidate_edge_image(i,2)>candidate_edge_image(i,4)
        %         X=candidate_edge_image(i,4): candidate_edge_image(i,2);           % X_coordinate of line point
        %         Y=interp1(X_coordinate,Y_coordinate,X);
        %         Y=round(Y);
    else
        X=repmat(X_coordinate(1,1),1,(D_Y)+1);
        %Y=candidate_edge_image(i,1):candidate_edge_image(i,3);
        Y=Yc_min :Yc_max ;
        
        % pixel_value =diag(img_obstacle(Y,X)); % Y is row of image and X is column
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
% labels = cellstr( num2str([1:size(unvisible_station,1)]'));
% imshow(map),title('visible_station_early');%_without_obstacle
% hold on
% plot(unvisible_station(:,4),unvisible_station(:,3), 'r*');text(unvisible_station(:,4),unvisible_station(:,3),labels);
% %% plot all station on map
% labels = cellstr( num2str([1:size(visible_station_early,1)]'));
% imshow(img_obstacle),title('visible_station_early');%_without_obstacle
% hold on
% plot(visible_station_early(:,2),visible_station_early(:,1), 'g*');text(visible_station_early(:,2),visible_station_early(:,1),labels);
% % draw axis of sight for visible points
% for i=1:size(visible_station_early);
%     r_P=[visible_station_early(i,1),visible_station_early(i,3)]';
%     c_P=[visible_station_early(i,2),visible_station_early(i,4)]';
%     line(c_P,r_P,'color','black','LineWidth',1);
% end

%%  camputing intersection between obstacle and camera's ray from visible_station_early to UnvisibleEdge_corners
corner= corner(im_new,'harris',15);                        % corner detection
figure,imshow(im_new),title('im_without_abstacle');
hold on
plot(corner(:,1),corner(:,2),'r*');
corners=[corner(:,2) corner(:,1)];
%UnvisibleEdge_corners=[216 604;644 248;209 509;142 513;665 575;272 606;287 605];
UnvisibleEdge_corners=[corners;unvisible_station(:,3:4)];   % merge the unvisible edge and corners
%UnvisibleEdge_corners=textread('unv.txt');
visible_station_candid=visible_station_early(:,1:2);
index3=1;
index4=1;
%distances=[];
for i=1:size(UnvisibleEdge_corners,1)
    %         for j=1:size(visible_station_candid,1)
    %             Y_coor=[visible_station_candid(j,1), UnvisibleEdge_corners(i,1)];   %image coordinate(candidate point and nearest edge point)
    %             X_coor=[visible_station_candid(j,2), UnvisibleEdge_corners(i,2)];
    %             Delta_X= abs(visible_station_candid(j,2)- UnvisibleEdge_corners(i,2));
    %             Delta_Y= abs(visible_station_candid(j,1)-UnvisibleEdge_corners(i,1));
    %             X_max=max(visible_station_candid(j,2),UnvisibleEdge_corners(i,2));
    %             X_min=min(visible_station_candid(j,2),UnvisibleEdge_corners(i,2));
    %             Y_max=max(visible_station_candid(j,1),UnvisibleEdge_corners(i,1));
    %             Y_min=min(visible_station_candid(j,1),UnvisibleEdge_corners(i,1));
    %             if Delta_X ~=0
    %                 X2= X_min:X_max;
    %                 Y2= interp1(X_coor,Y_coor,X2);
    %                 Y2= round(Y2);                                   % y_coordinate of line point
    %             else
    %                 X2=repmat(X_coor(1,1),1,(Delta_Y)+1);
    %                 Y2= Y_min:Y_max;
    %             end
    %             pixel_valuee =diag(img_obstacle(Y2,X2)); % Y is row of image and X is column
    %             if length(find(pixel_valuee==0))==0
    %                 visible_station(index3,:)=visible_station_candid(j,:);   % candidate points without intersection with obstacle
    %                 index3=index3+1;
    %             else
    %                 unvisible_station2(index4,:)=visible_station_candid(j,:);
    %                 index4=index4+1;
    %             end
    %         end
    
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
all_station=[visible_station_early ; visibleC_unvisibleEdge_corners];   % include final candid point

%% plot all station on map
labels = cellstr( num2str([1:size(visible_station_early,1)]'));
imshow(map),title('visible_station_early');%_without_obstacle
hold on
plot(visible_station_early(:,2),visible_station_early(:,1), 'g*');text(visible_station_early(:,2),visible_station_early(:,1),labels);
%draw axis of sight for visible points
% for i=1:size(visible_station,1);
%     r_P=[UnvisibleEdge_corners(23,1),visible_station(i,1)]';
%     c_P=[UnvisibleEdge_corners(23,2),visible_station(i,2)]';
%     line(c_P,r_P,'color','black','LineWidth',1);
% end

%      %% compute corresponding station(4 cameras) for corners and unvisible edges
% % compute distance
% % distance=[];
% % for i=1:size(UnvisibleEdge_corners,1)
% %     a1=UnvisibleEdge_corners(i,:);
% %     distance=[];
% %     for j=1:size(visible_station_candid)
% %         a2=visible_station_candid(j,1:2);
% %         dist=sqrt(sum((a1-a2).^2));
% %         distance=[distance;dist];
% %     end
% %     minmum= sort(distance);
% %     for k=1:4                            % k is number of nearest cameras
% %         [ID_min,~]=find(minmum(k,1)==distance(:,1));
% %         min_candid(k,:)=visible_station_candid(ID_min(1,1),:);
% %         clear ID_min
% %     end
% %     near_candids(4*i-3:4*i,:)=min_candid;
% %     clear minmum distance
% %     visibleP_unvisibleEdge_corners(4*i-3:4*i,1:2)=min_candid;     % visibleP_unvisibleEdge_corners is include corners , unvisible edge and corresponding visible candidate point
% %     visibleP_unvisibleEdge_corners(4*i-3:4*i,3:4)=repmat(UnvisibleEdge_corners(i,:),4,1);
% % end
% % all_station=[visible_station_early ; visibleP_unvisibleEdge_corners];   % include final candid point
%
% %%
% plot unvisibleEdge_corners
% UnvisibleEdge_corners_small=[131 392;619 250;125 287;142 513;550 255;319 601;319 601;349 599];
% labels = cellstr( num2str([1:size(UnvisibleEdge_corners,1)]'));
% imshow(im_ob),title('unvisibleEdge_corners');
% hold on
% plot(UnvisibleEdge_corners(:,2),UnvisibleEdge_corners(:,1), 'r*');text(UnvisibleEdge_corners(:,2),UnvisibleEdge_corners(:,1),labels);

%plot(UnvisibleEdge_corners_small(:,2),UnvisibleEdge_corners_small(:,1), 'r*');%text(UnvisibleEdge_corners_small(:,2),UnvisibleEdge_corners_small(:,1),labels);

% plot all station on map
labels = cellstr( num2str([1:size(visible_station_early,1)]'));
imshow(map),title('visible_station_early');%_without_obstacle
hold on
plot(visible_station_early(:,2),visible_station_early(:,1), 'g*');%text(visible_station_early(:,2),visible_station_early(:,1),labels);

% draw  visibleC_unvisibleEdge_corners
% for i=1:size(visibleC_unvisibleEdge_corners);
%     r_P=[visibleC_unvisibleEdge_corners(i,1),visibleC_unvisibleEdge_corners(i,3)]';
%     c_P=[visibleC_unvisibleEdge_corners(i,2),visibleC_unvisibleEdge_corners(i,4)]';
%     line(c_P,r_P,'color','black','LineWidth',1);
% end
% draw axis of sight for all stations
% for i=1:size(all_station);
%     r_P=[all_station(i,1),all_station(i,3)]';
%     c_P=[all_station(i,2),all_station(i,4)]';
%     line(c_P,r_P,'color','black','LineWidth',1);
% end
% draw axis of sight for visible points
for i=1:size(visible_station_early);
    r_P=[visible_station_early(i,1),visible_station_early(i,3)]';
    c_P=[visible_station_early(i,2),visible_station_early(i,4)]';
    line(c_P,r_P,'color','black','LineWidth',1);
end

%% transformation coordinate of candidate points and edge points from imege space to ground space

icp=textread('icp.txt');
GCP=textread('GCP.txt');
tform = maketform('affine',[icp(1:3,1) , icp(1:3,2)],[GCP(1:3,1), GCP(1:3,2)]);
[xc, yc] = tformfwd(tform, all_station(:,1), all_station(:,2)) ;
[xe, ye] = tformfwd(tform, all_station(:,3), all_station(:,4)) ;

%% 3D Coordinate (condidate points and edge points)
coordinate=[];
coordinate(:,1)=xc;
coordinate(:,2)=yc;
threeD_coordinate=[];
%height=input('please input height of building : ');
for i=0.7:1:15
    threeD_coordinate=[threeD_coordinate;coordinate,i*ones(size(xc,1),1)];
end
coordinate_edge=[];
coordinate_edge(:,1)=xe;
coordinate_edge(:,2)=ye;
threeD_coordinate_edge=[];
for i=0.7:1:15
    threeD_coordinate_edge=[threeD_coordinate_edge;coordinate_edge,i*ones(size(xe,1),1)];
end
candidate_edge_3D =[threeD_coordinate,threeD_coordinate_edge]; % all point in 3d space

%% Quaternion - computing roll,pitch,yaw for points with height=¿

bx=(xe-xc);
by=(ye-yc);
bz=zeros(size(bx,1),1);
norm_b=sqrt(bx.^2+by.^2);
norm_a=1;
unit_bx=bx./norm_b;         % normal
unit_by=by./norm_b;
unit_bz=bz./norm_b;
unit_b=[unit_bx,unit_by,unit_bz];

for n=1:size(bx,1)
    a=[0 0 -1];                % unit vector(Z vector(camera coordinate system)in global coordinate system)
    %Cross product of two vectors
    % rotation_vector= cross(a,b);                             % quaternion vector
    Outx = a(1,2) * unit_b(n,3) - a(1,3) * unit_b(n,2);
    Outy = a(1,3) * unit_b(n,1) - a(1,1) * unit_b(n,3);
    Outz = a(1,1) * unit_b(n,2) - a(1,2) * unit_b(n,1);
    rotation_vector(n,:)=[Outx Outy Outz];
    rotation_angle(n,:)= acos( a(1,1) * unit_b(n,1) + a(1,2) * unit_b(n,2) + a(1,3) * unit_b(n,3));
end
Rvector_ex=rotation_vector(:,1);
Rvector_ey=rotation_vector(:,2);
Rvector_ez=rotation_vector(:,3);

q0 = cos(rotation_angle/2);
q1 = Rvector_ex .* sin(rotation_angle/2);
q2 = Rvector_ey .* sin(rotation_angle/2);
q3 = Rvector_ez .* sin(rotation_angle/2);

jjjj= q0.^2 +q1.^2 +q2.^2 +q3.^2 ;
yaw=atan2((2.*(q1.*q2+q0.*q3)), (q1.^2+q0.^2-q3.^2-q2.^2));       %rotation about the Z-axis (radian)
pitch=asin(2.*(q0.*q2-q1.*q3));                                  %rotation about the Y-axis (radian)
roll=atan2((2.*(q0.*q1+q3.*q2)), (q3.^2-q2.^2-q1.^2+q0.^2));      %rotation about the X-axis (radian)

tz=size(threeD_coordinate,1)/size(all_station,1) ;
euler = [roll.*180/pi , pitch.*180/pi , yaw.*180/pi];             % orientation in degree
total_euler=repmat(euler,tz,1);

label=(1:size(threeD_coordinate,1))';
%coordinate_rotation=[label , threeD_coordinate , total_euler];
coordinate_rotation=[label(1:size(all_station,1),:) , (threeD_coordinate(1:size(all_station,1),:))*1000, total_euler(1:size(all_station,1),:)];
%coordinate_rotation=[label(size(all_station,1)+1:2*size(all_station,1),:) , (threeD_coordinate(1:size(all_station,1),:))*1000, total_euler(1:size(all_station,1),:)];
%% plot selected candidate point in IND
output_ind= textread('output_f18_fs8_i40_facad.txt');
all_candid_pnts=[label(1:2*size(all_station,1),:),(repmat(all_station(:,:),2,1))];

for i=1:size(output_ind,1)
    [m,~] =find(all_candid_pnts(:,1)==output_ind(i,1));
    IND(i,:)=all_candid_pnts(m,:);
end
figure, imshow(map),title('output_ind');
hold on
plot(IND(:,3),IND(:,2),'r*');text(IND(:,3),IND(:,2),cellstr( num2str([IND(:,1)])));

% draw axis of sight

for i=1:size(IND);
    r_P=[IND(i,2),IND(i,4)]';
    c_P=[IND(i,3),IND(i,5)]';
    line(c_P,r_P,'color','black','LineWidth',1);
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

