#### Start
ssh pi@10.42.0.32
cd /home/pi/Lab3
cd /home/garza/rpiZero/piDuino
scp -r ./* pi@10.42.0.32:/home/pi/Lab3
/home/garza/Documents/cse237B/Lab3_CSE237B

#### Compile
g++ -lpthread piDuino.h piDuino.cpp imu_test.cpp -o imu_test