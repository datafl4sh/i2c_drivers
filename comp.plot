set terminal png medium size 640,480
set output 'comp.png'

set grid

set multiplot layout 2,1

plot 'sensordata.dat' using (($0)*5):1 w l title 'Temperature BME280', \
     'sensordata.dat' using (($0)*5):4 w l title 'Temperature DHT12'

plot 'sensordata.dat' using (($0)*5):3 w l title 'Humidity BME280', \
     'sensordata.dat' using (($0)*5):5 w l title 'Humidity DHT12'

