/*
*
*  GNSS Data Analysis (NMEA-0183 sentence)
*  AUTHOR: He Guojian
*  2019-01
*/
#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <cstdio>
using namespace std;

//单选：使用的是哪种语句
#define USING_GPGGA
//#define USING_GPRMC

//GPGGA sentence
//$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77\r
class CGPGGA
{
public:
	double timeUTC;//<1> UTC日期， hhmmss.sss(时分秒)格式
	double latitude;//<2> ddmm.mmmm(度分格式)
	char northOrSouth;//<3>
	double longitude;//<4> dddmm.mmmm(度分格式)
	char eastOrWest;//<5>
	int GPSstatus;//<6> 质量因子——0=未定位，1=GPS单点定位固定解，2=差分定位，
								//3=PPS解；4=RTK固定解；5=RTK浮点解；6=估计值；7=手工输入模式；
								//8=模拟模式；
	int satelliteNum;//<7>应用解算位置的卫星数
	double HDOP;//<8>水平图形强度因子——0.500~99.000 ；大于6不可用
	double altitude;//<9>天线高程（海平面）——－9999.9～99999.9
	string altitudeUnit;// 天线高程单位(m) ——m
	double geiod;//<10>大地水准面起伏——地球椭球面相对大地水准面的高度
	string geiodUnit;//大地水准面起伏单位(m)   ——m
	int rtkAge;//<11>  差分GPS数据期——差分时间（从最近一次接收到差分信号开始的秒数，
						 //如果不是差分定位将为空），不使用DGPS时为空
	string rtkID;//<12> 基准站号——0000~1023；不使用DGPS时为空
	//int    bccVal; //校验
};

//GPRMC sentence ((Recommended Minimum Specific GPS/TRANSIT Data))
//$GPRMC,024813.640,A,3158.4608,N,11848.3737,E,10.05,324.27,150706,,,A*50\r
class CGPRMC
{
public:
	double timeUTC;//<1>UTC日期， hhmmss.sss(时分秒)格式
	char status;	//<2> 定位状态，A=有效定位，V=无效定位
	double latitude;//<3> ddmm.mmmm(度分格式)
	char northOrSouth;//<4>
	double longitude;//<5> dddmm.mmmm(度分格式)
	char eastOrWest;//<6>
	double velocity; //<7> 地面速率(000.0~999.9节)
	double yaw; 	//<8> 地面航向(000.0~359.9度，以正北为参考基准)
	string dayUTC; //<9>UTC日期，ddmmyy(日月年)格式
	double magneticYaw; //<10> 磁偏角(000.0~180.0度，前面的0也将被传输)
	char magneticEorW;//<11> 磁偏角方向，E(东)或W(西)
	char mode;  //<12> 模式指示(仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效)
	//int    bccVal; //校验
};

//处理NEMA-0183语句的类
class CCopeNmea
{
public:
	int 	CopeData( char* );
	void 	DisplayAll();
	CCopeNmea();
	~CCopeNmea();

public:
	CGPGGA *p_gpgga;
	CGPRMC *p_gprmc;
	int num_new;  //当前处理完，产生了几条新语句
	
private:
	void SpliceData(char*);
	bool BccCheck( string& str );
	void ExtractData( string& str );

private:
	string raw_data;
	string a_frame;
};


CCopeNmea::CCopeNmea()
{
	raw_data = "";
	a_frame = "";
	p_gpgga = NULL;
	p_gprmc = NULL;
#ifdef USING_GPGGA
	p_gpgga = new CGPGGA[10];
#endif
#ifdef USING_GPRMC
	p_gprmc = new CGPRMC[10];
#endif
	num_new = 0;
}
CCopeNmea::~CCopeNmea()
{
	if(p_gpgga) delete []p_gpgga;
	if(p_gprmc) delete []p_gprmc;
}


void CCopeNmea::SpliceData(char* str_in) {
	raw_data += str_in;
	if(raw_data.size() > 4096)
	{
		raw_data = raw_data.substr(raw_data.size()-4096);
	}
}

bool CCopeNmea::BccCheck( string& str ){//"$...."
	//printf("bccCheck.\n");
	if (str.empty())
		return false;
	
	int a = str[1], i=2;
	while(str[i] != '*')
	{
		a ^= str[i];
		++i;
	}
	
	int    bccVal; //校验
	stringstream ss;
	ss << str[i+1] << str[i+2];
	ss >> hex >> bccVal;//hex
	ss.clear();
	if ( bccVal == a )
		return true;
	else
		return false;
}

void CCopeNmea::ExtractData( string& str )
{
	//protect the program, no more than 10 for once time!
	if(num_new > 10) return;
	
	//将字符串的分隔符由逗号变为空格（空内容则添加逗号），便于处理
	string str_copy;
	for (unsigned int i=0; i<str.size(); ++i)
	{
		if(str[i] == ',' || str[i] == '*')
		{
				str_copy.push_back(' '); //更改为用“空格”做分隔符
				if(i>0 && str[i-1] == ',') //如果此项是空，用逗号代替实际内容
				{
					str_copy.push_back(','); 
					str_copy.push_back(' '); 
				}
		}
		else	
			str_copy.push_back(str[i]);
	}
	
#ifdef USING_GPGGA
	//===============================================
	stringstream ss(str_copy);
	string tmp;
	
	//head：$GPGGA
	ss >> tmp;
	
	//UTC <1>: hhmmss.sss (时分秒，转换为秒)
	ss >> tmp;
	if(tmp.size() >=6 && tmp != ",")
		//只用了分、秒，为了便于与lidar时间戳做对比（lidar的也没有小时及以上的）
		p_gpgga[num_new].timeUTC = /*stod(tmp.substr(0,2))*3600.0 +*/ 
				stod(tmp.substr(2,2))*60.0 + stod(tmp.substr(4));
	else
		p_gpgga[num_new].timeUTC = 0.0;
	
	//latitude<2><3>: ddmm.mmmm（度分格式，转换为度）
	ss >> tmp >> p_gpgga[num_new].northOrSouth;
	if(tmp.size() >=4 && tmp != ",")
	{
		p_gpgga[num_new].latitude = stod(tmp.substr(0,2)) + 
													 stod(tmp.substr(2))/60.0;
		if(p_gpgga[num_new].northOrSouth == 'S')
			p_gpgga[num_new].latitude *= -1.0;
	}
	else p_gpgga[num_new].latitude = 0.0;
	if(p_gpgga[num_new].northOrSouth == ',')
		p_gpgga[num_new].northOrSouth = '\0';

	
	//longitude<4><5>: dddmm.mmmm（度分格式，转换为度）
	ss >> tmp >> p_gpgga[num_new].eastOrWest;
	if(tmp.size() >=5 && tmp != ",")
	{
		p_gpgga[num_new].longitude = stod(tmp.substr(0,3)) + 
												 stod(tmp.substr(3))/60.0;
		if(p_gpgga[num_new].eastOrWest == 'W')
			p_gpgga[num_new].longitude *= -1.0;
	}
	else p_gpgga[num_new].longitude = 0.0;
	if(p_gpgga[num_new].eastOrWest == ',')
		p_gpgga[num_new].eastOrWest = '\0';

	
	//GPSstatus: //<6> 质量因子——0=未定位，1=GPS单点定位固定解，2=差分定位，
	//3=PPS解；4=RTK固定解；5=RTK浮点解；6=估计值；7=手工输入模式；
	//8=模拟模式；9WAAS差分。
	ss>>tmp;
	if(tmp != ",")
		p_gpgga[num_new].GPSstatus = stoi(tmp);
	else p_gpgga[num_new].GPSstatus = 0;
	
	//satelliteNum: <7>应用解算位置的卫星数，从00到12
	ss>>tmp;
	if(tmp != ",")
		p_gpgga[num_new].satelliteNum = stoi(tmp);
	else p_gpgga[num_new].satelliteNum = 0;
 
 	//HDOP: <8>水平图形强度因子——0.500~99.000 ；一般认为HDOP越小，质量越好，大于6不可用。
	ss>>tmp;
	if(tmp != ",")
		p_gpgga[num_new].HDOP = stod(tmp);
	else p_gpgga[num_new].HDOP = 99.0;
	
	//altitude: <9>天线高程（海平面）——－9999.9～99999.9
	ss>>p_gpgga[num_new].altitude;
	ss>>p_gpgga[num_new].altitudeUnit;
	
	//geiod: <10>大地水准面起伏——地球椭球面相对大地水准面的高度，-9999.9到9999.9米
	ss>>p_gpgga[num_new].geiod;
	ss>>p_gpgga[num_new].geiodUnit;
	
	//rtkAge: <11>差分GPS数据期限(RTCM SC-104)，最后设立RTCM传送的秒数量，如不是差分定位则为空
	ss>>p_gpgga[num_new].rtkAge;
	
	//rtkID: <12> 差分参考基站标号，从0000到1023(首位0也将传送)。
	ss>>p_gpgga[num_new].rtkID;
	//===============================================
#endif

#ifdef USING_GPRMC
	//===============================================
	stringstream ss(str_copy);
	string tmp;
	
	//head：$GPRMC
	ss >> tmp;
	
	//UTC <1>: hhmmss.sss (时分秒，转换为秒)
	ss >> tmp;
	if(tmp.size() >=6  && tmp != ",")
		p_gprmc[num_new].timeUTC = /*stod(tmp.substr(0,2))*3600.0 + */
				stod(tmp.substr(2,2))*60.0 + stod(tmp.substr(4));
	else
		p_gprmc[num_new].timeUTC = 0.0;
			
	//<2> 定位状态，A=有效定位，V=无效定位
	ss >> p_gprmc[num_new].status;
	
	//latitude<3><4>: ddmm.mmmm（度分格式，转换为度）
	ss >> tmp >> p_gprmc[num_new].northOrSouth;
	if(tmp.size() >=4  && tmp != ",")
	{
		p_gprmc[num_new].latitude = stod(tmp.substr(0,2)) + 
													 stod(tmp.substr(2))/60.0;
		if(p_gprmc[num_new].northOrSouth == 'S')
			p_gprmc[num_new].latitude *= -1.0;		
	}
	else p_gprmc[num_new].latitude = 0.0;
	if(p_gprmc[num_new].northOrSouth == ',')
		p_gprmc[num_new].northOrSouth = '\0';

	
	//longitude<5><6>: dddmm.mmmm（度分格式，转换为度）
	ss >> tmp >> p_gprmc[num_new].eastOrWest;
	if(tmp.size() >=5  && tmp != ",")
	{
		p_gprmc[num_new].longitude = stod(tmp.substr(0,3)) + 
												 stod(tmp.substr(3))/60.0;
		if(p_gprmc[num_new].eastOrWest == 'W')
			p_gprmc[num_new].longitude *= -1.0;
	}
	else p_gprmc[num_new].longitude = 0.0;
	if(p_gprmc[num_new].eastOrWest == ',')
		p_gprmc[num_new].eastOrWest = '\0';

	
	//<7> 地面速率(000.0~999.9节)
	ss >> p_gprmc[num_new].velocity;
	
	//<8> 地面航向(000.0~359.9度，以正北为参考基准)
	ss >> p_gprmc[num_new].yaw;
	
	//<9> UTC日期，ddmmyy(日月年)格式
	ss >> p_gprmc[num_new].dayUTC;
	
	//<10> 磁偏角(000.0~180.0度，前面的0也将被传输)
	ss >> p_gprmc[num_new].magneticYaw;
	
	//<11> 磁偏角方向，E(东)或W(西)
	ss >> p_gprmc[num_new].magneticEorW;
	
	//<12> 模式指示(仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效)
	ss >> p_gprmc[num_new].mode;
	//===============================================
#endif

	//num + 1
	num_new++;
}



int CCopeNmea::CopeData( char* str_in )
{
	//set 0
	num_new = 0;
	
	//Splice with the old sentence
	SpliceData(str_in);  
	if(raw_data.size() < 20)
		return 0;
	
	//iteratively find a whole NEMA sentence, and analysis
	int pos_start = 0, pos_end = 0;
	while(pos_start!=string::npos && pos_end!=string::npos)
	{
		// find start
#ifdef USING_GPGGA
		pos_start = raw_data.find("$GPGGA", pos_end);
#endif
#ifdef USING_GPRMC
		pos_start = raw_data.find("$GPRMC", pos_end);
#endif
		if (pos_start == string::npos)
		{
			raw_data = raw_data.substr(pos_end+1);
			continue;
		}
		
		//find end
		pos_end = raw_data.find("\r", pos_start);
		if (pos_end == string::npos)
		{
			raw_data = raw_data.substr(pos_start);
			continue;
		}
		
		//get the whole sentence, and check
		a_frame.clear();
		a_frame = raw_data.substr(pos_start, pos_end-pos_start+1);
		if(!BccCheck(a_frame))  //check if ok!
		{
			//std::cout<<"check false!!!!!"<<std::endl;
			continue;
		}
		
		//analysis
		ExtractData(a_frame);
	}

	return num_new;
}


#endif


