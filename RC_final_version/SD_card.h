// this is debug where we right data to SD card 
int sd_count=0;
  File file;
 void sd_write(float sd_lat, float sd_lng, float sd_lat2, float sd_lng2, float sd_distance ,int sd_satallite, float sd_dgree, float sd_dg2, int sd_point_shift){
file = SD.open("data.txt", FILE_WRITE); // open "file.txt" to write data
if (file) {
  file.println(sd_count);
	file.println("GPS lat");
	file.println(sd_lat,6);
	file.println("GPS lng");
	file.println(sd_lng,6);
	file.println("input lat");
	file.println(sd_lat2,6);
	file.println("input lng");
	file.println(sd_lng2,6);
	file.println("distance");
	file.println(sd_distance);
	file.println("satallite");
	file.println(sd_satallite);
	file.println("dgree");
	file.println(sd_dgree);
	file.println("dg2");
	file.println(sd_dg2);
  file.println("point shift");
  file.println(sd_point_shift);
  file.println();
	file.close();
 sd_count+=1 ;// just to know when the data were registered
  }}
  
void sd_write_position(float sd_lat, float sd_lng, int sd_point_shift,float sd_distance){
file = SD.open("position.txt", FILE_WRITE); // open "file.txt" to write data
if (file) {
  file.println("distance");
  file.println(sd_distance);
  if(sd_point_shift==0){
  file.println("");
  file.print("going to point A from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  if(sd_point_shift==1){
  file.println("");
  file.print("going to point B from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
 if(sd_point_shift==2){
 file.println("");
  file.print("going to point C from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  if(sd_point_shift==3){
file.println("");
file.print("going to point D from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  if(sd_point_shift==4){
 file.println("");
  file.print("going to point E from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  if(sd_point_shift==5){
 file.println("");
  file.print("going to point F from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  if(sd_point_shift==6){
  file.println("");
  file.print("going to point G from ");
  file.print("GPS lat = ");
  file.print(sd_lat,6);
  file.print("GPS lng = ");
  file.print(sd_lng,6);
    }
  file.close();
  }}

  void sd_write_IMU(float sd_accel, long sd_encoder, float sd_degreeimu, float sd_speed, int sd_point){
file = SD.open("IMU.txt", FILE_WRITE); // open "file.txt" to write data
if (file) {
  file.println("");
  file.print("pint ");
  file.print(sd_point); 
  file.println("");
  file.print("accel ");
  file.print(sd_accel);
  file.println("");
  file.print("encoder distanc ");
  file.print(sd_encoder);
  file.println("");
  file.print("IMU degree ");
  file.print(sd_degreeimu); 
  file.println("");
  file.print("IMU speed ");
  file.print(sd_speed); 
  file.close();
  }}
