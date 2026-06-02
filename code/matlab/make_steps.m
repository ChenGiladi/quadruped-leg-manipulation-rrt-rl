function matrixdata=make_steps(x_1,y_1,x_2,y_2,leg,num_of_sam)
vec_data_x_1=x_1;
vec_data_y_1=y_1;
vec_data_x_2=x_2;
vec_data_y_2=y_2;
ones_metrix=ones(1,num_of_sam+1);
z_zeros=zeros(1,num_of_sam+1);
h=0.0016;
si=1;
if leg==1
    diff_x=vec_data_x_2(1)-vec_data_x_1(1);
    diff_y=vec_data_y_2(1)-vec_data_y_1(1);
    addindata_x=vec_data_x_1(1):diff_x/num_of_sam:vec_data_x_2(1);
    addindata_y=vec_data_y_1(1):diff_y/num_of_sam:vec_data_y_2(1);
    z=si*addindata_x.^2+si*addindata_y.^2+3;
    z(1)=0;z(end)=0;z=z/norm(z);
%     meansamp_1=(vec_data_x_2(1)-vec_data_x_1(1))/2;
%     meansamp_2=(vec_data_y_2(1)-vec_data_y_1(1))/2;
%     mat_of_interp=[vec_data_x_1(1) vec_data_y_1(1) 0;...
%                    meansamp_1 meansamp_2 h;...
%                    vec_data_x_2(1) vec_data_x_2(1) 0];
%     temp=[1;25;50];vec=1:1:(num_of_sam+1);
%     x_from_interp=interp1(temp,mat_of_interp(:,1),vec);
%     y_from_interp=interp1(temp,mat_of_interp(:,2),vec);
%     z_from_interp=interp1(temp,mat_of_interp(:,3),vec);
%     matrixdata=[x_from_interp;y_from_interp;z_from_interp;...
    matrixdata=[addindata_x;addindata_y;z;...
        vec_data_x_1(2)*ones_metrix;vec_data_y_1(2)*ones_metrix;z_zeros;...
        vec_data_x_1(3)*ones_metrix;vec_data_y_1(3)*ones_metrix;z_zeros;...
        vec_data_x_1(4)*ones_metrix;vec_data_y_1(4)*ones_metrix;z_zeros]';
    figure(2);hold on
%     plot3(x_from_interp,y_from_interp,z_from_interp,'r');hold on;grid on;
    plot3(addindata_x,addindata_y,z,'r');hold on;grid on;
elseif leg==2
    diff_x=vec_data_x_2(2)-vec_data_x_1(2);
    diff_y=vec_data_y_2(2)-vec_data_y_1(2);
    addindata_x=vec_data_x_1(2):diff_x/num_of_sam:vec_data_x_2(2);
    addindata_y=vec_data_y_1(2):diff_y/num_of_sam:vec_data_y_2(2);
    z=si*addindata_x.^2+si*addindata_y.^2+3;z(1)=0;z(end)=0;
    z=z/norm(z);
%     meansamp_1=(vec_data_x_2(2)-vec_data_x_1(2))/2;
%     meansamp_2=(vec_data_y_2(2)-vec_data_y_1(2))/2;
%     mat_of_interp=[vec_data_x_1(2) vec_data_y_1(2) 0;...
%                    meansamp_1 meansamp_2 h;...
%                    vec_data_x_2(2) vec_data_x_2(2) 0];
%     temp=[1;25;50];vec=1:1:(num_of_sam+1);
%     x_from_interp=interp1(temp,mat_of_interp(:,1),vec);
%     y_from_interp=interp1(temp,mat_of_interp(:,2),vec);
%     z_from_interp=interp1(temp,mat_of_interp(:,3),vec);
    matrixdata=[vec_data_x_1(1)*ones_metrix;vec_data_y_1(1)*ones_metrix;z_zeros;...
        addindata_x;addindata_y;z;...
        vec_data_x_1(3)*ones_metrix;vec_data_y_1(3)*ones_metrix;z_zeros;...
        vec_data_x_1(4)*ones_metrix;vec_data_y_1(4)*ones_metrix;z_zeros]';
    figure(2);hold on
%     plot3(x_from_interp,y_from_interp,z_from_interp,'r');hold on;grid on;
    plot3(addindata_x,addindata_y,z,'g');hold on;grid on;
elseif leg==3
    diff_x=vec_data_x_2(3)-vec_data_x_1(3);
    diff_y=vec_data_y_2(3)-vec_data_y_1(3);
    addindata_x=vec_data_x_1(3):diff_x/num_of_sam:vec_data_x_2(3);
    addindata_y=vec_data_y_1(3):diff_y/num_of_sam:vec_data_y_2(3);
    z=si*addindata_x.^2+si*addindata_y.^2+3;z(1)=0;z(end)=0;
    z=z/norm(z);
%     meansamp_1=(vec_data_x_2(3)-vec_data_x_1(3))/2;
%     meansamp_2=(vec_data_y_2(3)-vec_data_y_1(3))/2;
%     mat_of_interp=[vec_data_x_1(3) vec_data_y_1(3) 0;...
%                    meansamp_1 meansamp_2 h;...
%                    vec_data_x_2(3) vec_data_x_2(3) 0];
%     temp=[1;25;50];vec=1:1:(num_of_sam+1);
%     x_from_interp=interp1(temp,mat_of_interp(:,1),vec);
%     y_from_interp=interp1(temp,mat_of_interp(:,2),vec);
%     z_from_interp=interp1(temp,mat_of_interp(:,3),vec);
    matrixdata=[vec_data_x_1(1)*ones_metrix;vec_data_y_1(1)*ones_metrix;z_zeros;...
        vec_data_x_1(2)*ones_metrix;vec_data_y_1(2)*ones_metrix;z_zeros;...
        addindata_x;addindata_y;z;...
        vec_data_x_1(4)*ones_metrix;vec_data_y_1(4)*ones_metrix;z_zeros]';
    figure(2);hold on
%     plot3(x_from_interp,y_from_interp,z_from_interp,'r');hold on;grid on;
    plot3(addindata_x,addindata_y,z,'b');hold on;grid on;
elseif leg==4
    diff_x=vec_data_x_2(4)-vec_data_x_1(4);
    diff_y=vec_data_y_2(4)-vec_data_y_1(4);
    addindata_x=vec_data_x_1(4):diff_x/num_of_sam:vec_data_x_2(4);
    addindata_y=vec_data_y_1(4):diff_y/num_of_sam:vec_data_y_2(4);
    z=si*addindata_x.^2+si*addindata_y.^2+3;z(1)=0;z(end)=0;
    z=z/norm(z);
%     meansamp_1=(vec_data_x_2(4)-vec_data_x_1(4))/2;
%     meansamp_2=(vec_data_y_2(4)-vec_data_y_1(4))/2;
%     mat_of_interp=[vec_data_x_1(4) vec_data_y_1(4) 0;...
%                    meansamp_1 meansamp_2 h;...
%                    vec_data_x_2(4) vec_data_x_2(4) 0];
%     temp=[1;25;50];vec=1:1:(num_of_sam+1);
%     x_from_interp=interp1(temp,mat_of_interp(:,1),vec);
%     y_from_interp=interp1(temp,mat_of_interp(:,2),vec);
%     z_from_interp=interp1(temp,mat_of_interp(:,3),vec);
    matrixdata=[vec_data_x_1(1)*ones_metrix;vec_data_y_1(1)*ones_metrix;z_zeros;...
        vec_data_x_1(2)*ones_metrix;vec_data_y_1(2)*ones_metrix;z_zeros;...
        vec_data_x_1(3)*ones_metrix;vec_data_y_1(3)*ones_metrix;z_zeros;...
        addindata_x;addindata_y;z]';
    figure(2);hold on
%     plot3(x_from_interp,y_from_interp,z_from_interp,'r');hold on;grid on;
    plot3(addindata_x,addindata_y,z,'m');hold on;grid on;
end
hold off;
end