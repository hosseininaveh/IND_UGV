function [visible_station]=select_visible_station(im_ob,UnvisibleEdge_corners,visible_station_candid)
visible_station=zeros(1000,2);
index3=1;
    %for i=1:size(UnvisibleEdge_corners,1)
    for j=1:size(visible_station_candid,1)
        Y_coor=[visible_station_candid(j,1), UnvisibleEdge_corners(1)];   %image coordinate(candidate point and nearest edge point)
        X_coor=[visible_station_candid(j,2), UnvisibleEdge_corners(2)];
        Delta_X= abs(visible_station_candid(j,2)- UnvisibleEdge_corners(2));
        Delta_Y= abs(visible_station_candid(j,1)-UnvisibleEdge_corners(1));
        X_max=max(visible_station_candid(j,2),UnvisibleEdge_corners(2));
        X_min=min(visible_station_candid(j,2),UnvisibleEdge_corners(2));
        Y_max=max(visible_station_candid(j,1),UnvisibleEdge_corners(1));
        Y_min=min(visible_station_candid(j,1),UnvisibleEdge_corners(1));
        if Delta_X ~=0
            X2= X_min:X_max;
            Y2= interp1(X_coor,Y_coor,X2);
            Y2= round(Y2);                                   % y_coordinate of line point
        else
            X2=repmat(X_coor(1,1),1,(Delta_Y)+1);
            Y2= Y_min:Y_max;
        end
        pixel_valuee =diag(im_ob(Y2,X2)); % Y is row of image and X is column
        if length(find(pixel_valuee==0))==0
            visible_station(index3,:)=visible_station_candid(j,:);   % candidate points without intersection with obstacle
            index3=index3+1;
        end
    end
    
    visible_station = visible_station(1:index3-1,:);
    %end
end
