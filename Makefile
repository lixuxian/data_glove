CXX=g++
CXXFLAGS= -Wall -g -O2 
CXX_OPTS= -Wall -g -O2 -I./service/

SOURCES=SensorListener.cpp sensor_service.cpp main.cpp udp_sender.cpp local_data_dumper.cpp bt_send.cpp

INSTALL=install

PROG=sensor_app

%.o: %.cpp                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: $(PROG).o 
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) -o $(PROG) \
		$(SOURCES) \
		service/libsensorservice.a \
		libs/libI2Cdev.a \
		-lwiringPi

$(PROG).o: libs/libI2Cdev.a service/libsensorservice.a


service/libsensorservice.a:
	$(MAKE) -C service/

libs/libI2Cdev.a:
	$(MAKE) -C libs/I2Cdev

install:
	$(INSTALL) -m 755 $(PROG) $(DESTDIR)/usr/local/bin/

clean:
	cd service && $(MAKE) clean
	cd libs/I2Cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(PROG)
