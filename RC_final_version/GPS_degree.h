/*getDegrees will get the dgree from your pint and the input point 
(bearing ) it is just some math */

#include <math.h>
float getDegrees(float lat1, float lon1, float lat2, float lon2) {

    float dLat = ((lat2-lat1)*71) / 4068.0;
    float dLon = ((lon2-lon1)*71) / 4068.0;

    lat1 = ((lat1)*71) / 4068.0;
    lat2 = ((lat2)*71) / 4068.0;

     float y = sin(dLon) * cos(lat2);
     float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
     float brng = (atan2(y, x)* 57296) / 1000;

    // fix negative degrees
    if(brng<0) {
        brng=360-abs(brng);
    }
    return brng;
}


/*calcDistance will calculate distance between your point and 
next point   */
float calcDistance(float latHome, float lonHome, float latDest, float lonDest) {
    float R = 6371000; //Radius of the Earth
    latHome = (M_PI/180)*(latHome);
    latDest = (M_PI/180)*(latDest);
    float differenceLon = (M_PI/180)*(lonDest - lonHome);
    float differenceLat = (M_PI/180)*(latDest - latHome);
    float a = sin(differenceLat/2) * sin(differenceLat/2) + 
    cos(latHome) * cos(latDest) * 
    sin(differenceLon/2) * sin(differenceLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = R * c;
   // Serial.println( distance);
    
    return distance;
}
