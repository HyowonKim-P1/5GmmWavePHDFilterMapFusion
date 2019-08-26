function AOA = getAOA(BS,UE,point,type)
   xUE=UE(1:3);                                
   alpha=UE(4);
   switch(type)
       case('LOS')                      
           AOA=[pi+atan2(UE(2)-point(2),UE(1)-point(1))-alpha  asin((point(3)-UE(3))/norm(point-xUE))];
       case('VA')
            VA=point; 
            AOA=[atan2(VA(2)-UE(2),VA(1)-UE(1))-alpha asin((VA(3)-UE(3))/(norm(VA-xUE)))];
       case('SP')
           SP=point;
           AOA=[atan2(SP(2)-UE(2),SP(1)-UE(1))-alpha asin((SP(3)-UE(3))/(norm(SP-xUE)))];
       otherwise
           error('unknown type')
   end
end