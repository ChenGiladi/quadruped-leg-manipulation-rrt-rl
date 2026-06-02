function finel_metrix=makeparab(T)
data_x=[T.leg_1_x T.leg_2_x T.leg_3_x T.leg_4_x];
data_y=[T.leg_1_y T.leg_2_y T.leg_3_y T.leg_4_y];
leg=1;
num_of_sam=50;
length_of_the_data=length(data_x(:,1));
finel_metrix=zeros(num_of_sam*length_of_the_data,12);
k=1;n=num_of_sam+1;
for i=1:(length_of_the_data-1)
    x_1=[data_x(i,1),data_x(i,2),data_x(i,3),data_x(i,4)];
    y_1=[data_y(i,1),data_y(i,2),data_y(i,3),data_y(i,4)];
    x_2=[data_x(i+1,1),data_x(i+1,2),data_x(i+1,3),data_x(i+1,4)];
    y_2=[data_y(i+1,1),data_y(i+1,2),data_y(i+1,3),data_y(i+1,4)];
    matrix=make_steps(x_1,y_1,x_2,y_2,leg,num_of_sam);
    leg=leg+1;
    if leg>4
        leg=1;
    end
    finel_metrix(k:n,1:12)=matrix;
    k=k+num_of_sam;
    n=n+num_of_sam;
end