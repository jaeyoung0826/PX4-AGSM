# Anti GPS Spoofing Module 제작

연구 목적: 본 연구는 UAV의 운행과정에서 발생 할 수 있는 GPS-Spoofing 공격을 시연해보고 이에 대응 할 수 있는 AGSM을 제작 하고자 합니다.


# px4 toolchain install (Ubuntu 16.04)

참고 사이트:https://docs.px4.io/master/en/dev_setup/dev_env.html
```
$ git clone https://github.com/PX4/firmware.git --recursive
$ cd /firmware
$ bash./Tools/setup/ubunth.sh
$ sudo reboot now
```

# QgroundControl install

```
$ sudo apt-get remove modemanager –y
$ sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
$ wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
$ chmod +x ./QGroundControl.AppImage
$ ./QGroundControl.AppImage
```

# GPS-SPoofing 구현

1. Gps Spoofing : 인공 위성보다 더 강한 신호를 전달하여 위성 신호의 정보를 누락 시키고 외부의 신호를 인식하여 기존의 정보를 변형 
2. FakeGps :기존의 위치 정보와는 다른 가상 위치 정보를 생성
3. FakeGps는 새로운 가상 정보를 생성할 뿐 드론의 헤딩 정보를  변경하지 않음
4. 단순히 FakeGps만을 실행 할 경우 기존의 정보를 따라감
5. failure gps off를 통해 기존 정보를 누락시켜 가상 정보로 헤딩 할 수 있도록 진행함
6. Failure과 FakeGps를 통해 Gps Spoofing 상황을 구상

```
 pxh> param set SYS_FAILURE_EN 1 // injection on
 pxh> fake_gps start             // fake_gps on -> make gps2_raw parameter
 pxh> failure gps off            // GPS_RAW_INT off
```

# GPS-Spoofing Demo image (영상은 발표자료 참고)

![GPS-Spoofing](/uploads/3a686c20d33071d578fd4689515f5f87/GPS-Spoofing.png)


# AGSM 제작 이론


![기울기그림](/uploads/ae587ae1ba0872102fc9224d38d0c9da/기울기그림.png)

1. 파란색의 두 점을 각각 waypoint1, waypoint2 라고 하였을 때, 두 waypoint를 통하여 기울기를 계산할 수 있다. 
2. 계산된 기울기를 통하여 그림과 같이 오차범위에 해당되는 기울기 2개를 추가로 생성한다. 
3. 3개의 기울기와 waypoints를 이용하여 회색 영역과 같은 드론의 유효 범위를 지정한다. 만약 드론이 유효 범위 밖에서 지속적으로 머물러 
   있다면 경고메세지를 출력한다. 


# 두 waypoint의 위치 정보를 통한 기울기 생성 구현 
```

void EKF2::calculate_inclination_target()
{
	double x,y; // latitude increment, longitude increment
	inclination=new double[int(mission.count)]; // dynamic allocation by waypoints for inclinations
	for (int i = 0; i < int(mission.count)-1; i++) {
		struct mission_item_s mission_item {}; // struct for current mission
		struct mission_item_s next_mission_item {}; // struct for next mission
		dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)); // get current mission info
		dm_read((dm_item_t)mission.dataman_id, i+1, &next_mission_item, sizeof(mission_item_s));  // get next mission info
		x=next_mission_item.lat-mission_item.lat;
		y=next_mission_item.lon-mission_item.lon;
		std::cout.setf(std::ios::fixed);
		std::cout.precision(7);
		if(x!=0.0){
			inclination[i]=y/x;
		}
		else
		{
			inclination[i]=0.0;
		}
		}
}

```


# 드론의 mission waypoint(출발지) 위치 저장 코드
```

void EKF2::set_target_gps()
{
	std::cout.unsetf(std::ios::fixed);
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s mission_item {}; // struct for current mission
		if(int(mission.current_seq)==int(i)){  // get waypoint info which is same as current sequence
			dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s));
			target_lat=int32_t(mission_item.lat * long(pow(10,7))); // change number of digits lat degrees 10e-7
			target_lon=int32_t(mission_item.lon * long(pow(10,7))); // change number of digits lon degrees 10e-7
			target_alt=int32_t(mission_item.altitude * long(pow(10,4))); // change number of digits lon degrees 10e-4
		}
	}

}
```


# 현재 드론의 위치와 mission waypoint 사이의 기울기 생성 코드
```

double EKF2::calculate_inclination_current(gps_message gps_msg) // gps_msg contains current gps info
{
	double x,y; // latitude increment, longitude increment
	double check_inclination=0; // current inclination initialized zero
	x=target_lat-gps_msg.lat;
	y=target_lon-gps_msg.lon;
	if(x!=0.0){
		check_inclination=y/x;
	}
	std::cout.setf(std::ios::fixed);
	std::cout.precision(7);
	return check_inclination; // calculated inclination
}

```

# 목적지간 기울기와 현재 비행 중 기울기 비교
```

void EKF2::check_inclination(double check) // get current inclination from calculate_inclination_current
{
	double origin=inclination[int(mission.current_seq)-1]; // inclination between current mission's starting point and end point
	std:: cout << "(origin : "<<origin<<" | check : "<<check<<")";
	if(origin-1<=check && check <= origin+1) // compare two inclination
	{
		std::cout<<"\t--- Good moving"<<std::endl; // subtracting isn't over 1 then good flight
		warning_count=0; // reset warning count 0
	}
	else
	{
		std::cout<<"\t--- Bad moving"<<std::endl; // subtracting is over 1 then bad flight
		warning_count++; // increasing warning count
	}
}

```

# AGSM을 이용한 GPS-SPoofing 탐지 (영상은 발표자료 참고)

![AGSM](/uploads/31223259dd1646173809a2a0e9a98094/AGSM.png)


# Reference
[1] QGroundControl User Guide, Dronecode, 2021.04.26. 접속,https://docs.qgroundcontrol.com/master/en/index.html

[2] PX4 Autopilot User Guide Introduction, PX4 Autopilot 2021.06.14. 접속, https://docs.px4.io/master/en/ 

[3] 조승민, “드론 보안에 적용된 암호기술 현황”, 정보보호학회지 vol.30-2, 2020.4

[4] A. Koubâa, "Micro Air Vehicle Link (MAVlink) in a Nutshell: A Survey," in IEEE Access, vol. 7, pp.  87658-87680, 2019

[5] 류해원, 최성한, 하일규, “드론 운용의 보안 위협과 대응 방안", 한국정보처리학회, 2018.10

[6] 서진범, “GPS 스니핑을 이용한 안티 드론 알고리즘”, 한국정보통신학회, pp. 63 –66, 2019.05.23

