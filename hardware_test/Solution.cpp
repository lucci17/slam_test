#include "Solution.h"
#include "macro_defines.h"
#include <stdlib.h>

#define dmm2deg(dmm) (floor(dmm / 100.0) + fmod(dmm, 100.0) / 60.0)
namespace hardware
{
namespace gnss
{
int Solution::readByte(const char *buffer, int length)
{
  for (int i = 0; i < length; i++) {
    char input = buffer[i];

    if (readBuffer.size() < max_size_) {
      if (input != '\n') {
        readBuffer.push_back(input);
        continue;
      }
      readBuffer.push_back(input);
      // message received, try to decode NMEA messages
      std::vector<std::string> pars = decodeLine(readBuffer);
      if (pars.size() != 0) {
        if (pars[0] == "$GPGGA" || pars[0] == "$GNGGA") {
          decodeGPGGA(pars);
        } else if (pars[0] == "$GPNTR") {
          decodeGPNTR(pars);
        } else if (pars[0] == "$PTNL") {
          if (pars[1] == "PJK") {
            decodePTNL_PJK(pars);
          } else if (pars[1] == "VGK") {
            decodePTNL_VGK(pars);
          } else if (pars[1] == "AVR") {
            decodePTNL_AVR(pars);
          } else if (pars[1] == "VHD") {
            decodePTNL_VHD(pars);
          }
        } else {
          PRINT_WARNING("Invalid nmea format");
          // printf("Invalid nmea format!\n");
          readBuffer.clear();
          return BGS_ERROR;
        }
      }
      readBuffer.clear();
    } else {
      readBuffer.clear();
    }
  }
  return readBuffer.size();
}

std::vector<std::string> Solution::decodeLine(std::string line)
{
  char checksum = 0;
  char delim = ',';
  std::vector<std::string> elems;

  if (*(line.end() - 1) != '\n' || *(line.end() - 2) != '\r') {
    PRINT_WARNING("Wrong NMEA sentences");
    return elems;
  }

  // TODO remove white spaces

  // checksum validation
  auto pos = line.find_last_of('*');
  if (pos == std::string::npos) {
    PRINT_WARNING("NMEA without CHECKSUM");
  }

  for (int i = 0; i < (int)pos; i++) {
    checksum = checksum ^ line[i];
  }

  line = line.substr(0, pos);

  auto end = line.find(delim);
  auto start = 0;
  while (end != std::string::npos) {
    elems.push_back(line.substr(start, end - start));
    start = end + 1;
    end = line.find(delim, start);
  }

  return elems;
}

int Solution::decodeGPGGA(const std::vector<std::string> &parameters)
{
  double /*utcTime = 0.0,*/ latitude = 0.0, longitude = 0.0,
                            /*hdop = 0.0,*/ altitude = 0.0, height = 0.0;
  char northSouth, eastWest;
  int mode;

  if (parameters.size() < 14) {
    PRINT_ERROR("GGA missing data.");
    return BGS_ERROR;
  }

  // 		for( auto para : parameters )
  // 			std::cout << para;
  // 		std::cout << std::endl;

  // 		utcTime 	= atof(parameters[1].c_str());
  latitude = atof(parameters[2].c_str());
  northSouth = (parameters[3])[0];
  longitude = atof(parameters[4].c_str());
  eastWest = (parameters[5])[0];
  mode = atoi(parameters[6].c_str());
  // 		this->numberOfSats	  = atoi(parameters[7].c_str());
  // 		hdop = atof(parameters[8].c_str());
  altitude = atof(parameters[9].c_str());
  height = atof(parameters[11].c_str());

  // 	//printf("%s %d\n",parameters[6].c_str(),this->mode);
  // 	    if
  // ((northSouth!='N'&&northSouth!='S')||(eastWest!='E'&&eastWest!='W'))
  // 	    {
  // 	         PRINT_ERROR("Invalid nmea gpgga format");
  // 	         return BGS_ERROR;
  // 	    }

  latitude = (northSouth == 'N' ? 1.0 : -1.0) * dmm2deg(latitude);
  longitude = (eastWest == 'E' ? 1.0 : -1.0) * dmm2deg(longitude);
  height = altitude + height;

  positionData.enu.x = latitude;
  positionData.enu.y = longitude;
  positionData.enu.z = height;
  positionData.mode = mode;
  //     this->llaPosition.phi    = latitude;
  //     this->llaPosition.lambda = longitude;
  //     this->llaPosition.height = height;

  //     llaPosition.phi=llaPosition.phi/180*PI;
  //     llaPosition.lambda=llaPosition.lambda/180*PI;

  //     this->time  	= utcTime;
  //     this->hdop      = hdop;

  //     temp=lla2ecef(llaPosition);
  //     enu_t target;
  //     lla_t
  //     reference={31.29910126895*PI/180.0,121.51710258881*PI/180.0,82.0912};
  //     ecef2enu(temp,reference, &target);
  //     positionData.enu.x  = target.e;
  //     positionData.enu.y 	= target.n;
  // 	positionData.enu.z 	= target.u;
  //     positionData.mode   = mode;

  //     //printf("e %f n %f u %f mode %d
  //     \n",target.e,target.n,target.u,positionData.mode);
  //     // else sol->time=time;
  //     // pos2ecef(pos,sol->rr);
  //     // sol->stat=0<=solq&&solq<=8?solq_nmea[solq]:SOLQ_NONE;
  //     // sol->ns=nrcv
  //     // sol->type=0; /* postion type = xyz */

  //     //printf("time %f latitude %f lon %f mode %d ns %d hdop %f altitude %f
  //     msl %f ew%c
  //     ns%c\n",utcTime,latitude,longitude,mode,numberOfSats,hdop,altitude,height,eastWest,northSouth);

  return BGS_OK;
}

int Solution::decodeGPNTR(const std::vector<std::string> &parameters)
{
  double utcTime = 0.0, east = 0.0, north = 0.0, up = 0.0, baseline = 0.0;
  int mode = 0, stationID = -1;
  ;

  if (parameters.size() < 7) {
    PRINT_WARNING("NTR missing data");
    return BGS_ERROR;
  }

  utcTime = atof(parameters[1].c_str());
  mode = atoi(parameters[2].c_str());
  baseline = atof(parameters[3].c_str());
  east = atof(parameters[4].c_str());
  north = atof(parameters[5].c_str());
  up = atof(parameters[6].c_str());

  (void)utcTime;
  (void)stationID;
  (void)baseline;

  positionData.enu.x = east;
  positionData.enu.y = north;
  positionData.enu.z = up;
  positionData.mode = mode;

  // printf("time %f mode %d baseline %f east %f north %f up %f stationID
  // %d\n",utcTime,mode,baseline,east,north,up,stationID);

  return BGS_OK;
}

int Solution::decodePTNL_PJK(const std::vector<std::string> &parameters)
{
  double north = 0.0, east = 0.0, hdop = 0.0;
  int utcTime = 0, mode = 0, numberOfSats = 0;
  std::string height;
  double heightNum;

  if (parameters.size() < 12) {
    PRINT_WARNING("GGA missing data");
    return BGS_ERROR;
  }

  utcTime = atoi(parameters[3].c_str());
  north = atof(parameters[4].c_str());
  east = atof(parameters[6].c_str());
  mode = atoi(parameters[8].c_str());
  numberOfSats = atoi(parameters[9].c_str());
  hdop = atof(parameters[10].c_str());
  height = parameters[11];

  if (height.size() > 0) {
    heightNum = atof((height.substr(4)).c_str());
  } else {
    heightNum = 0.0;
  }

  (void)utcTime;
  (void)numberOfSats;
  (void)hdop;

  positionData.enu.x = east;
  positionData.enu.y = north;
  positionData.enu.z = heightNum;
  positionData.mode = mode;

  // printf("e %f n %f u %f mode %d \n",positionData.enu.x, positionData.enu.y,
  // positionData.enu.z, positionData.mode);

  return BGS_OK;
}

int Solution::decodePTNL_VGK(const std::vector<std::string> &parameters)
{
  double north = 0.0, east = 0.0, hdop = 0.0;
  int utcTime = 0, mode = 0, numberOfSats = 0;
  double height;

  if (parameters.size() < 10) {
    PRINT_WARNING("GGA missing data");
    return BGS_ERROR;
  }

  utcTime = atoi(parameters[3].c_str());
  east = atof(parameters[4].c_str());
  north = atof(parameters[5].c_str());
  height = atof(parameters[6].c_str());
  mode = atoi(parameters[7].c_str());
  numberOfSats = atoi(parameters[8].c_str());
  hdop = atof(parameters[9].c_str());

  (void)utcTime;
  (void)numberOfSats;
  (void)hdop;

  positionData.enu.x = east;
  positionData.enu.y = north;
  positionData.enu.z = height;
  positionData.mode = mode;
  // printf("e %f n %f u %f mode %d \n",positionData.enu.x, positionData.enu.y,
  // positionData.enu.z, positionData.mode);

  return BGS_OK;
}

int Solution::decodePTNL_AVR(const std::vector<std::string> &parameters)
{
  double yaw = 0.0, tile = 0.0, hdop = 0.0;
  int mode = 0; //, numberOfSats = 0;

  if (parameters.size() < 12) {
    PRINT_WARNING("GGA missing data");
    return BGS_ERROR;
  }

  yaw = atof(parameters[3].c_str());
  tile = atof(parameters[5].c_str());
  mode = atoi(parameters[10].c_str());
  hdop = atof(parameters[11].c_str());

  (void)hdop;

  gestureData.yaw = yaw;
  gestureData.tile = tile;
  gestureData.mode = mode;

  // printf("yaw %f tile %f mode %d \n",gestureData.yaw, gestureData.tile,
  // gestureData.mode);
  return BGS_OK;
}

int Solution::decodePTNL_VHD(const std::vector<std::string> &parameters)
{
  (void)parameters;
  // double utcPosition = 0.0, azimuth = 0.0, azimuthTime = 0.0, vertical = 0.0,
  // verticalTime = 0.0;
  // double range = 0.0, rangeTime = 0.0, pdop = 0.0;
  // int utcTime = 0, mode = 0, numberOfSats = 0;

  // if(parameters.size() < 12)
  // {
  // 	LogFile(WARNING,"GGA missing data \n");
  // 	return BGS_ERROR;
  // }

  // utcPosition			  = atof(parameters[2].c_str());
  // utcTime 			  = atoi(parameters[3].c_str());
  // azimuth      		  = atof(parameters[4].c_str());
  // azimuthTime      	  = atof(parameters[5].c_str());
  // vertical			  = atof(parameters[6].c_str());
  // verticalTime		  = atof(parameters[7].c_str());
  // range			      = atof(parameters[8].c_str());
  // rangeTime		      = atof(parameters[9].c_str());
  // mode        		  = atoi(parameters[10].c_str());
  // numberOfSats     	  = atoi(parameters[11].c_str());

  return BGS_OK;
}
}
} /// namespace hardware
