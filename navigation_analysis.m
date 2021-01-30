clear all
output_ind= textread('ind_output_facade pointing.txt');
agi_poses=textread('facadepointing_agisoft_camera_poses.txt');
i=1;
distance(size(agi_poses,1),size(output_ind,1))=0;
angular_distance(size(agi_poses,1),size(output_ind,1))=0;
final(size(agi_poses,1),21)=0;
while (i<size(agi_poses,1))
    j=1;
    while (j< size(output_ind,1))
        if (agi_poses(i,4)<1000)
            agi_poses(i,4)=400;
        else
            agi_poses(i,4)=1600;
        end
        distance(i,j)=sqrt((agi_poses(i,2)-output_ind(j,2))^2+(agi_poses(i,3)-output_ind(j,3))^2+(agi_poses(i,4)-output_ind(j,4))^2+(3.1415*(agi_poses(i,6)-output_ind(j,6))/180)^2);
        angular_distance(i,j) =abs(agi_poses(i,6)-output_ind(j,6));
        j=j+1;
    end
    j2=find(distance(i,:)==min(distance(i,1:size(output_ind,1)-1)));
    final(i,:)=[agi_poses(i,:),output_ind(j2,:)];
    i=i+1;
    
end
test=1;

    
