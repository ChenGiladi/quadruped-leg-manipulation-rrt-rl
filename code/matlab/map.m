function w = map()
w=zeros(60,60);
for i=1:60
    for j=1:60
        if (i==1)||(j==1)||(i==60)||(j==60)
            w(i,j)=1;
        end
        if (i==10)&&(j<=15)||(i==11)&&(j<=15)
            w(i,j)=1;
        end
        if (i==50)&&(j<=30)||(i==51)&&(j<=30)
            w(i,j)=1;
        end
        if (i==30)&&(j>=20)||(i==31)&&(j>=20)
            w(i,j)=1;
        end
    end
end