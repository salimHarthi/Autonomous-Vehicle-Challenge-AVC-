//servo and motor driving function
int s;
int servo_control(float D,float B){
  if(D>B){//left
   s=D-B;
   s=abs(s);
  if(s>25){
    s=25;
    }
  s=90+s;
 return  s;}
  
else if(D<B){// right
  
  s=D-B;
  s=abs(s);
  if(s>15){
    s=15;
    }
    s=90-s;
    
    return  s; 
  }
else{
  s=90;
  return  s;
  }  
}


  
