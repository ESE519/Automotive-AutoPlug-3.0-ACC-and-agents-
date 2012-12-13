clear all;
s = serial('COM6','baudrate',115200);
set(s,'InputBufferSize',32);
fopen(s);

%kk = fread(s,100);
count=200;
acceleration=zeros(4000,1);
brake=zeros(4000,1);
speed=zeros(4000,1);
distance=zeros(4000,1);
error = zeros(4000,1);
errorInteg = zeros(4000,1);
setSpeed = zeros(4000,1);
radarerror = zeros(4000,1);
carid = zeros(4000,1);
angle = zeros(4000,1);
isTrack = zeros(4000,1);
caryaw = zeros(4000,1);
a=zeros(200,1);
kk=zeros(32,1);

% 
%   figure;
%   subplot(1,2,1);
%   distance_handle=plot(a);title('CAR DISTANCE')
%   ylim([-10,200])
%   xlim([0,200])
%   sprintf('Car Distance')
%   
%   subplot(1,2,2);
%   speed_handle=plot(linspace(0,200,200),speed(count-199:count),'r');title('CAR SPEED');
% 
%   ylim([-10,200])
% 

 % subplot(1,3,3);
  

  

  figure;
  subplot(2,3,1);
  acceleration_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('CAR ACCELERATION');
  ylim([-10,120])

  subplot(2,3,2);
  brake_handle=plot(linspace(0,200,200),brake(count-199:count),'r');title('CAR BRAKE');
  ylim([-10,120])
 
  subplot(2,3,3);
  speed_handle=plot(linspace(0,200,200),speed(count-199:count),'r');title('CAR SPEED');

  ylim([-100,300])
 
  subplot(2,3,4);
  distance_handle=plot(a);title('CAR DISTANCE')
  xlim([0,200])
  ylim([-10,500])
  sprintf('Car Distance')
  
  subplot(2,3,5);
  error_handle=plot(a);title('ERROR')
  ylim([-50,50])
  sprintf('ERROR')
  
  subplot(2,3,6);
  errorInteg_handle=plot(a);title('ERRORINTEG')
  xlim([0,200])
  ylim([-3000,3000])
  sprintf('ERRORINTEG')
  
  figure;
  subplot(2,3,1);
  radarerror_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('RADAR ERROR');
  ylim([-3,3]);
  
  subplot(2,3,2);
  carid_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('CAR ID');
  ylim([-0,3]);
  
  subplot(2,3,3);
  angle_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('ANGLE');
  ylim([-10,60]);
  
  subplot(2,3,4);
  isTrack_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('WHETHER TRACKED');
  ylim([-1,2]);

  subplot(2,3,5);
  caryaw_handle = plot(linspace(0,200,200),linspace(0,200,200),'r');title('CAR YAW');
  ylim([-180,180]);

  
  



state=0;


while(1)
kk = fread(s,32);

'reading' 


for i=1:32
    k=kk(i);
    if(k==170)
        chksum = 0;
       state=1; 'state1'
    end
    
    if(state==1)
        if(k==204)
            state=2;    
            chksum = bitxor(170,204);
       continue;
       'got one'      
        else
         
        end
    end
    
    if(state>=2)
        state=state+1;
        if (state<16)
            chksum = bitxor(k,chksum);
        end
        
        if(state==3)
        accl=typecast(uint8(k),'int8');  
        acceleration(count)=double(accl);
        continue;
        end
        
        if(state==4)
            br=typecast(uint8(k),'int8');
            brake(count)=double(br);
            continue;
        end
        
        if(state==5)
           sp=typecast(uint8(k),'int8');
           speed(count)=double(sp)*3.6;
           continue;
        end
        
        if(state==6)
           dh=typecast(uint8(k),'int8');
           distanceh=double(dh);
           continue;
        end
        
        if(state==7)
            dl=typecast(uint8(k),'int8');
            distancel=double(dl);
            if(distancel)<0 distancel = distancel+256;end
            distance(count) = distanceh*256 + distancel;
            continue;
        end
        
        if(state==8)
            err=typecast(uint8(k),'int8');
            error(count) = double(err);
            continue;
        end
        
        if(state==9)
            eH=typecast(uint8(k),'int8');
            errIntH = double(eH);
            continue;
        end
        
        if(state==10)
            eL=typecast(uint8(k),'int8');
            errIntL = double(eL);
            if(errIntL<0) errIntL = errIntL+256; end
            errorInteg(count) = 256*errIntH + errIntL;
            continue;
        end

        if(state==11)
            raer=typecast(uint8(k),'int8');
            radarerror(count) = double(raer)/50;
            continue;
        end
        
        if(state==12)
            id=typecast(uint8(k),'int8');
            carid(count) = double(id);
            continue;
        end
        
        if(state==13)
            ang=typecast(uint8(k),'int8');
            if(ang<0) ang=ang+256; end
            angle(count) = double(ang);
            continue;
        end
        
       if(state==14)
            isT=typecast(uint8(k),'int8');
            isTrack(count) = double(isT);
            continue;
       end

       if(state==15)
            carY=typecast(uint8(k),'int8');
            caryaw(count) = double(carY);
            continue;
       end
        
        if(state==16)
           state=0; 
           if(chksum == k)
            set(acceleration_handle,'ydata',acceleration(count-199:count));
            set(brake_handle,'ydata',brake(count-199:count));
            set(speed_handle,'ydata',speed(count-199:count));
            set(distance_handle,'ydata',distance(count-199:count));
            set(error_handle,'ydata',error(count-199:count));
            set(errorInteg_handle,'ydata',errorInteg(count-199:count));
            set(radarerror_handle,'ydata',radarerror(count-199:count));
            set(carid_handle,'ydata',carid(count-199:count));
            set(angle_handle,'ydata',angle(count-199:count));
            set(isTrack_handle,'ydata',isTrack(count-199:count));
            set(caryaw_handle,'ydata',caryaw(count-199:count));
%             set(setSpeed_handle,'ydata',setSpeed(count-199:count));
            count=count+1;
           pause(0.00001);
           end
          
        end
        
    end
    if(count>=1600)
     acceleration(1:200)= acceleration(count-199:count);
     brake(1:200)= brake(count-199:count);
     speed(1:200)= speed(count-199:count);
     distance(1:200)= distance(count-199:count);
     error(1:200) = error(count-199:count);
     errorInteg(1:200) = errorInteg(count-199:count);
     radarerror(1:200) = radarerror(count-199:count);
     carid(1:200) = carid(count-199:count);
     angle(1:200) = angle(count-199:count);       
     isTrack(1:200) = isTrack(count-199:count);
     caryaw(1:200) = caryaw(count-199:count);
     %setSpeed(1:200) = setSpeed(count-199:count);
     count = 200;
    end
        
        
        
    
end


end



fclose(s);